"""
Run a PCLA leaderboard agent (e.g. tfv6_visiononly) from spawn 0 to spawn 10.

Uses PCLA (Pretrained CARLA Leaderboard Agents) from third_party/PCLA. Builds a
Leaderboard-format route XML from spawn 0 → spawn 10 (same A→B as closed_loop_drive),
spawns the ego vehicle, and runs the agent in a loop until max steps or Ctrl+C.

Prerequisites:
  - CARLA 0.9.16 running (e.g. ./CarlaUE4.sh)
  - PCLA cloned in third_party/PCLA
  - PCLA venv activated: source /workspace/venvs/pcla/bin/activate
  - TransfuserV6 weights downloaded: bash infra/download_tfv6_weights.sh

Usage:
  python scripts/run_pcla_vision.py [--agent tfv6_visiononly] [--max-steps N] [--hud]
"""
import argparse
import os
import sys
import threading

import numpy as np
from PIL import Image

_script_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.dirname(_script_dir)
_pcla_dir = os.path.join(_repo_root, "third_party", "PCLA")
if os.path.isdir(_pcla_dir) and _pcla_dir not in sys.path:
    sys.path.insert(0, _pcla_dir)

sys.path.insert(0, _script_dir)
import route_plan
from actor_capture import (
    get_walker_crossing_fixed,
    get_walker_crossing_fixed_with_offset,
    WALKER_SETTLE_TICKS,
)

import carla

FIXED_DELTA_SECONDS = 0.1
DEFAULT_ROUTE_PATH = "data/pcla_route.xml"
DEFAULT_OUT_DIR = "data/pcla_frames"
CHASE_OUT_DIR = "data/pcla_chase_frames"
SOURCE_OUT_DIR = "data/pcla_source_frames"
MAX_STEPS_DEFAULT = 5000

# Walker loop: cross → pause → recross (distance-based arrival, then pause ticks)
WALKER_ARRIVAL_THRESHOLD_M = 0.5
WALKER_PAUSE_TICKS = 50  # ~5 s at 10 Hz
# Start crossing when ego is within this distance of the crossing waypoint (sync with drive)
WALKER_TRIGGER_DISTANCE_M = 25.0
# Multiple walkers: spread along crosswalk, staggered start
NUM_WALKERS = 10
WALKER_SPACING_M = 1.2  # meters between walkers along the crosswalk
WALKER_STAGGER_TICKS = 15  # ticks between each walker starting to cross


def main():
    # Suppress Pygame welcome message before any pygame import (e.g. when using --hud).
    if "PYGAME_HIDE_SUPPORT_PROMPT" not in os.environ:
        os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

    parser = argparse.ArgumentParser(
        description="Run PCLA vision-only (or other) leaderboard agent from spawn 0 to spawn 10."
    )
    parser.add_argument(
        "--agent",
        default="tfv6_visiononly",
        help="PCLA agent name (default: tfv6_visiononly)",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="CARLA host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=2000,
        help="CARLA port (default: 2000)",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=MAX_STEPS_DEFAULT,
        metavar="N",
        help="Max simulation steps (default: %d). 0 = no limit (Ctrl+C to stop)." % MAX_STEPS_DEFAULT,
    )
    parser.add_argument(
        "--route",
        default=DEFAULT_ROUTE_PATH,
        metavar="PATH",
        help="Path to write or read route XML (default: %s)" % DEFAULT_ROUTE_PATH,
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=60.0,
        metavar="SEC",
        help="CARLA client timeout in seconds (default: 60)",
    )
    parser.add_argument(
        "--hud",
        action="store_true",
        help="Bake throttle/brake/steer overlay onto saved frames (headless-safe).",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Force headless Pygame (SDL_VIDEODRIVER=dummy). Use when no DISPLAY.",
    )
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUT_DIR,
        metavar="DIR",
        help="Directory for captured frames (default: %s)" % DEFAULT_OUT_DIR,
    )
    parser.add_argument(
        "--no-walker",
        action="store_true",
        help="Do not spawn pedestrian; run without walker.",
    )
    parser.add_argument(
        "--walker-waypoint-index",
        type=int,
        default=None,
        metavar="N",
        help="Waypoint index for pedestrian crossing (default: len(waypoints)//3).",
    )
    parser.add_argument(
        "--log-walkers",
        action="store_true",
        help="Log walker and ego positions every 20 steps (for debugging crossing placement).",
    )
    args = parser.parse_args()

    # ---- PCLA checks ----
    if not os.path.isdir(_pcla_dir):
        print("PCLA not found at %s. Clone it with:" % _pcla_dir, file=sys.stderr)
        print("  git submodule update --init third_party/PCLA", file=sys.stderr)
        sys.exit(1)

    try:
        from PCLA import PCLA, route_maker
    except ImportError as e:
        print("Failed to import PCLA: %s" % e, file=sys.stderr)
        print("Ensure third_party/PCLA is present and dependencies are installed.", file=sys.stderr)
        print("  source /workspace/venvs/pcla/bin/activate", file=sys.stderr)
        print("  bash infra/install_pcla_tfv6.sh", file=sys.stderr)
        sys.exit(1)

    # Check for pretrained weights so we fail fast with a clear message
    if args.agent == "tfv6_visiononly":
        _visiononly_config = os.path.join(
            _pcla_dir, "pcla_agents", "transfuserv6_pretrained", "visiononly_resnet34", "config.json"
        )
        if not os.path.isfile(_visiononly_config):
            print("Pretrained weights for tfv6_visiononly not found.", file=sys.stderr)
            print("Expected: %s" % _visiononly_config, file=sys.stderr)
            print("Download them with:", file=sys.stderr)
            print("  bash infra/download_tfv6_weights.sh", file=sys.stderr)
            sys.exit(1)

    # ---- HUD setup ----
    use_hud = args.hud
    hud = None
    last_control = {"control": None}
    if use_hud:
        sys.path.insert(0, _script_dir)
        from carla_hud import (
            MinimalHUD,
            composite_hud_on_rgb,
            init_pygame_for_headless,
            init_pygame_for_hud,
        )
        if args.headless or not os.environ.get("DISPLAY"):
            init_pygame_for_headless()
        init_pygame_for_hud()
        import pygame
        pygame.init()
        hud = MinimalHUD(1280, 720)

    out_dir = args.output_dir
    data_dir = os.path.dirname(os.path.abspath(out_dir)) or "data"
    chase_dir = os.path.join(data_dir, "pcla_chase_frames")
    source_dir = os.path.join(data_dir, "pcla_source_frames")
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(chase_dir, exist_ok=True)
    os.makedirs(source_dir, exist_ok=True)

    def _log(msg):
        print(msg, flush=True)

    _log("Connecting to CARLA at %s:%d (timeout=%.0fs)..." % (args.host, args.port, args.timeout))
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    map_name = route_plan.select_map_name(client.get_available_maps())
    world = client.load_world(map_name)
    _log("Loaded map: %s" % map_name)

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)

    carla_map = world.get_map()
    try:
        start_wp, end_wp, end_idx = route_plan.get_start_end_waypoints(
            carla_map, route_plan.END_SPAWN_INDEX
        )
    except RuntimeError as e:
        _log("%s" % e)
        sys.exit(1)
    _log("Building route from spawn 0 to spawn %d..." % end_idx)
    _log("  start_wp: %s" % start_wp.transform.location)
    _log("  end_wp:   %s" % end_wp.transform.location)

    try:
        route_trace = route_plan.trace_route(carla_map, start_wp, end_wp)
    except ImportError as e:
        print("Failed to import GlobalRoutePlanner: %s" % e, file=sys.stderr)
        print("Ensure third_party/carla_pythonapi is present.", file=sys.stderr)
        sys.exit(1)
    if len(route_trace) <= 1:
        _log("Route has too few waypoints.")
        sys.exit(1)
    waypoints = route_plan.waypoints_from_trace(route_trace)

    route_path = os.path.abspath(args.route)
    route_dir = os.path.dirname(route_path)
    if route_dir:
        os.makedirs(route_dir, exist_ok=True)
    route_maker(waypoints, route_path)
    _log("Wrote route to %s (%d waypoints)" % (route_path, len(waypoints)))

    _log("Destroying existing vehicles...")
    for actor in world.get_actors().filter("vehicle.*"):
        try:
            actor.destroy()
        except Exception:
            pass
    world.tick()

    bp = world.get_blueprint_library()
    vehicle_bp = bp.filter("vehicle.tesla.model3")[0]
    if not vehicle_bp:
        vehicle_bp = bp.filter("vehicle.*")[0]
    # Spawn at the snapped driving-lane waypoint (correct orientation)
    spawns = carla_map.get_spawn_points()
    vehicle = world.try_spawn_actor(vehicle_bp, start_wp.transform)
    if vehicle is None:
        _log("Spawn at driving-lane waypoint failed (collision), using spawn point 0.")
        vehicle = world.spawn_actor(vehicle_bp, spawns[0])
    _log("Spawned vehicle at route start. Setting up PCLA agent '%s'..." % args.agent)

    pcla = PCLA(args.agent, vehicle, route_path, client)
    _log("Agent ready. Running (max_steps=%s). Ctrl+C to stop." % (args.max_steps if args.max_steps else "inf"))

    # ---- Observer camera (dashboard-style) -> out_dir (e.g. pcla_frames) ----
    camera_tf = carla.Transform(
        carla.Location(x=2.0, z=1.55), carla.Rotation(pitch=0, yaw=0, roll=0)
    )
    cam_bp = bp.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", "1280")
    cam_bp.set_attribute("image_size_y", "720")
    cam_bp.set_attribute("fov", "120")
    camera = world.spawn_actor(cam_bp, camera_tf, attach_to=vehicle)

    # Step index for this tick; set right before world.tick() so all callbacks save the same frame number
    save_step = [0]
    frame_ready = threading.Event()

    def on_image(image):
        i = save_step[0]
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
            (image.height, image.width, 4)
        )
        rgb = arr[:, :, :3][:, :, ::-1]
        if use_hud and hud is not None:
            snapshot = world.get_snapshot()
            ts = getattr(snapshot, "timestamp", None)
            sim_time = getattr(ts, "elapsed_seconds", 0) if ts else 0
            current_map = world.get_map().name if world else None
            snap_velocity = None
            snap_transform = None
            try:
                for actor_snap in snapshot:
                    if getattr(actor_snap, "id", None) == vehicle.id:
                        snap_velocity = actor_snap.get_velocity()
                        snap_transform = actor_snap.get_transform()
                        break
            except (TypeError, AttributeError):
                pass
            rgb = composite_hud_on_rgb(
                rgb,
                hud,
                vehicle,
                last_control["control"],
                image.frame,
                sim_time,
                map_name=current_map,
                velocity=snap_velocity,
                transform=snap_transform,
            )
        path = os.path.join(out_dir, "frame_%04d.png" % i)
        Image.fromarray(rgb).save(path)
        frame_ready.set()

    camera.listen(on_image)
    _log("Observer camera attached. Capturing frames to %s/" % out_dir)

    # ---- Chase camera (third-person follow) -> pcla_chase_frames ----
    chase_cam_bp = bp.find("sensor.camera.rgb")
    chase_cam_bp.set_attribute("image_size_x", "1280")
    chase_cam_bp.set_attribute("image_size_y", "720")
    chase_cam_bp.set_attribute("fov", "90")
    chase_tf = carla.Transform(
        carla.Location(x=-10.0, y=0.0, z=4.0),
        carla.Rotation(pitch=-15.0, yaw=0.0, roll=0.0),
    )
    chase_camera = world.spawn_actor(chase_cam_bp, chase_tf, attach_to=vehicle)
    chase_ready = threading.Event()

    def on_chase_image(image):
        i = save_step[0]
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
            (image.height, image.width, 4)
        )
        rgb = arr[:, :, :3][:, :, ::-1]
        path = os.path.join(chase_dir, "frame_%04d.png" % i)
        Image.fromarray(rgb).save(path)
        chase_ready.set()

    chase_camera.listen(on_chase_image)
    _log("Chase camera attached. Capturing frames to %s/" % chase_dir)

    # ---- Source cameras (agent sensor view) -> pcla_source_frames ----
    source_cameras = []
    source_buffer = {}
    source_cam_ids = []
    source_ready = threading.Event()
    source_lock = threading.Lock()

    for spec in getattr(pcla.agent_instance, "sensors", lambda: [])():
        if "sensor.camera.rgb" not in spec.get("type", ""):
            continue
        cam_id = spec.get("id", "rgb")
        w = int(spec.get("width", 384))
        h = int(spec.get("height", 384))
        fov = float(spec.get("fov", 60))
        loc = carla.Location(
            x=float(spec.get("x", 0)),
            y=float(spec.get("y", 0)),
            z=float(spec.get("z", 2.25)),
        )
        rot = carla.Rotation(
            roll=float(spec.get("roll", 0)),
            pitch=float(spec.get("pitch", 0)),
            yaw=float(spec.get("yaw", 0)),
        )
        sc_bp = bp.find("sensor.camera.rgb")
        sc_bp.set_attribute("image_size_x", str(w))
        sc_bp.set_attribute("image_size_y", str(h))
        sc_bp.set_attribute("fov", str(fov))
        sc_actor = world.spawn_actor(sc_bp, carla.Transform(loc, rot), attach_to=vehicle)
        source_cam_ids.append(cam_id)

        def _make_callback(sid):
            def _cb(image):
                arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                    (image.height, image.width, 4)
                )
                rgb = arr[:, :, :3][:, :, ::-1].copy()
                with source_lock:
                    source_buffer[sid] = rgb
                    if len(source_buffer) == len(source_cam_ids):
                        source_ready.set()
            return _cb

        sc_actor.listen(_make_callback(cam_id))
        source_cameras.append({"id": cam_id, "actor": sc_actor})
    if source_cameras:
        _log("Source cameras (%d) attached. Capturing agent view to %s/" % (len(source_cameras), source_dir))

    # ---- Walkers: NUM_WALKERS at same crossing, spread along crosswalk, staggered start ----
    walkers = []
    crossing_ref_loc = None
    trigger_step = None

    if not args.no_walker:
        # Default: crossing at route start so walkers are next to the car at start
        crossing_waypoint_index = (
            args.walker_waypoint_index
            if args.walker_waypoint_index is not None
            else 0
        )
        crossing_transform = waypoints[crossing_waypoint_index].transform
        crossing_ref_loc = waypoints[crossing_waypoint_index].transform.location
        all_walker_bps = list(bp.filter("walker.pedestrian.*"))
        walker_bp = next(
            (b for b in all_walker_bps if b.id == "walker.pedestrian.0001"),
            all_walker_bps[0],
        )
        controller_bp = bp.find("controller.ai.walker")
        for i in range(NUM_WALKERS):
            offset_m = (i - (NUM_WALKERS - 1) / 2.0) * WALKER_SPACING_M
            start_loc, target_loc = get_walker_crossing_fixed_with_offset(
                crossing_transform, offset_m
            )
            # Copy locations: CARLA can reuse the same object, so without copies
            # all walkers end up at the last iteration's position (e.g. 0, 0.7).
            start_loc = carla.Location(x=start_loc.x, y=start_loc.y, z=start_loc.z)
            target_loc = carla.Location(x=target_loc.x, y=target_loc.y, z=target_loc.z)
            actor = world.try_spawn_actor(walker_bp, carla.Transform(start_loc))
            if actor is None:
                _log("Failed to spawn walker %d at crossing waypoint %d." % (i, crossing_waypoint_index))
                continue
            controller = world.spawn_actor(controller_bp, carla.Transform(), actor)
            controller.start()
            controller.go_to_location(start_loc)
            controller.set_max_speed(1.0)
            walkers.append({
                "actor": actor,
                "controller": controller,
                "state": "waiting",
                "target": start_loc,
                "pause_ticks_left": 0,
                "start_loc": start_loc,
                "target_loc": target_loc,
                "index": i,
            })
        if walkers:
            for _ in range(WALKER_SETTLE_TICKS):
                frame_ready.clear()
                world.tick()
                frame_ready.wait(timeout=5.0)
            _log("Spawned %d walkers at waypoint %d. Crossing when ego within %.0fm, staggered %d ticks." % (
                len(walkers), crossing_waypoint_index, WALKER_TRIGGER_DISTANCE_M, WALKER_STAGGER_TICKS))
            # Log positions once so you can check crossing vs route (e.g. in map or frames)
            ego_loc = vehicle.get_location()
            _log("  crossing_ref_loc (waypoint %d): x=%.1f y=%.1f z=%.1f" % (
                crossing_waypoint_index, crossing_ref_loc.x, crossing_ref_loc.y, crossing_ref_loc.z))
            _log("  ego (route start):              x=%.1f y=%.1f z=%.1f  dist_to_crossing=%.1fm" % (
                ego_loc.x, ego_loc.y, ego_loc.z, ego_loc.distance(crossing_ref_loc)))
            for w in walkers:
                loc = w["actor"].get_location()
                _log("  walker %d start: x=%.1f y=%.1f z=%.1f  (target x=%.1f y=%.1f)  dist_to_ego=%.1fm" % (
                    w["index"], loc.x, loc.y, loc.z,
                    w["target_loc"].x, w["target_loc"].y, loc.distance(ego_loc)))
        else:
            crossing_ref_loc = None

    try:
        step = 0
        while True:
            if args.max_steps and step >= args.max_steps:
                _log("Reached max steps %d." % args.max_steps)
                break
            frame_ready.clear()
            chase_ready.clear()
            if source_cameras:
                with source_lock:
                    source_buffer.clear()
                source_ready.clear()
            ego_action = pcla.get_action()
            last_control["control"] = ego_action
            vehicle.apply_control(ego_action)

            # Walker state machine (per walker): waiting → cross → pause → recross
            if crossing_ref_loc is not None and walkers:
                ego_dist = vehicle.get_location().distance(crossing_ref_loc)
                if ego_dist < WALKER_TRIGGER_DISTANCE_M and trigger_step is None:
                    trigger_step = step
                for w in walkers:
                    if w["state"] == "waiting":
                        if trigger_step is not None and step >= trigger_step + w["index"] * WALKER_STAGGER_TICKS:
                            w["controller"].go_to_location(w["target_loc"])
                            w["state"] = "cross_to_left"
                            w["target"] = w["target_loc"]
                    elif w["state"] in ("cross_to_left", "cross_to_right"):
                        dist = w["actor"].get_location().distance(w["target"])
                        if dist < WALKER_ARRIVAL_THRESHOLD_M:
                            w["state"] = "pause"
                            w["pause_ticks_left"] = WALKER_PAUSE_TICKS
                    elif w["state"] == "pause":
                        w["pause_ticks_left"] -= 1
                        if w["pause_ticks_left"] <= 0:
                            if w["target"] == w["target_loc"]:
                                next_target = w["start_loc"]
                                w["state"] = "cross_to_right"
                            else:
                                next_target = w["target_loc"]
                                w["state"] = "cross_to_left"
                            w["controller"].go_to_location(next_target)
                            w["target"] = next_target

            save_step[0] = step  # lock step for this tick so all callbacks save same index
            world.tick()
            # Wait for camera frames
            frame_ready.wait(timeout=max(10.0, args.timeout))
            chase_ready.wait(timeout=max(10.0, args.timeout))
            if source_cameras:
                source_ready.wait(timeout=max(10.0, args.timeout))
                with source_lock:
                    parts = [source_buffer[cid] for cid in source_cam_ids if cid in source_buffer]
                if parts:
                    concat = np.concatenate(parts, axis=1)
                    path = os.path.join(source_dir, "frame_%04d.png" % save_step[0])
                    Image.fromarray(concat).save(path)
            step += 1
            if step <= 10 or step % 100 == 0:
                _log("  step %d" % step)
            # Optional: log walker and ego positions every 20 steps for debugging
            if args.log_walkers and walkers and step % 20 == 0:
                ego_loc = vehicle.get_location()
                dist_cross = ego_loc.distance(crossing_ref_loc) if crossing_ref_loc else float("nan")
                _log("  [walkers] step %d  ego x=%.1f y=%.1f  dist_to_crossing=%.1fm  trigger_step=%s" % (
                    step, ego_loc.x, ego_loc.y, dist_cross, trigger_step))
                for w in walkers:
                    loc = w["actor"].get_location()
                    _log("    walker %d %s  x=%.1f y=%.1f  dist_to_ego=%.1fm" % (
                        w["index"], w["state"], loc.x, loc.y, loc.distance(ego_loc)))
    except KeyboardInterrupt:
        _log("Interrupted by user.")
    finally:
        _log("Cleaning up...")
        for w in walkers:
            try:
                w["controller"].stop()
                w["controller"].destroy()
            except Exception:
                pass
            try:
                w["actor"].destroy()
            except Exception:
                pass
        try:
            camera.destroy()
        except Exception:
            pass
        try:
            chase_camera.destroy()
        except Exception:
            pass
        for sc in source_cameras:
            try:
                sc["actor"].destroy()
            except Exception:
                pass
        pcla.cleanup()
        try:
            world.apply_settings(original_settings)
        except Exception:
            pass
        _log("Captured %d frames to %s/" % (step, out_dir))
        _log("Done.")


if __name__ == "__main__":
    main()
