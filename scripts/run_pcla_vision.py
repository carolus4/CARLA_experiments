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

# Import carla after PCLA is on path so we use the same client
import carla

# Route: same as closed_loop_drive / 2Dplot_route
PREFERRED_MAPS = ["Town10", "Town10HD", "Town10_Opt", "Town01", "Town02"]
END_SPAWN_INDEX = 10
FIXED_DELTA_SECONDS = 0.1
DEFAULT_ROUTE_PATH = "data/pcla_route.xml"
DEFAULT_OUT_DIR = "data/pcla_frames"
MAX_STEPS_DEFAULT = 5000


def _short_map_name(path):
    return path.split("/")[-1] if "/" in path else path


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
    args = parser.parse_args()

    # ---- PCLA checks ----
    if not os.path.isdir(_pcla_dir):
        print("PCLA not found at %s. Clone it with:" % _pcla_dir, file=sys.stderr)
        print("  git submodule update --init third_party/PCLA", file=sys.stderr)
        sys.exit(1)

    try:
        from PCLA import PCLA, location_to_waypoint, route_maker
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
    os.makedirs(out_dir, exist_ok=True)

    def _log(msg):
        print(msg, flush=True)

    _log("Connecting to CARLA at %s:%d (timeout=%.0fs)..." % (args.host, args.port, args.timeout))
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    available = [_short_map_name(m) for m in client.get_available_maps()]
    map_name = next(
        (m for m in PREFERRED_MAPS if m in available),
        available[0] if available else "Town01",
    )
    world = client.load_world(map_name)
    _log("Loaded map: %s" % map_name)

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)

    carla_map = world.get_map()
    spawns = carla_map.get_spawn_points()
    if not spawns:
        _log("No spawn points on this map.")
        sys.exit(1)
    end_idx = min(END_SPAWN_INDEX, len(spawns) - 1)
    start_loc = spawns[0].location
    end_loc = spawns[end_idx].location

    _log("Building route from spawn 0 to spawn %d..." % end_idx)
    waypoints = location_to_waypoint(client, start_loc, end_loc, distance=2.0, draw=False)
    if len(waypoints) <= 1:
        _log("Route has too few waypoints.")
        sys.exit(1)
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
    start_transform = waypoints[0].transform
    vehicle = world.try_spawn_actor(vehicle_bp, start_transform)
    if vehicle is None:
        _log("Spawn at route waypoint failed (collision), using spawn point 0.")
        vehicle = world.spawn_actor(vehicle_bp, spawns[0])
    _log("Spawned vehicle at route start. Setting up PCLA agent '%s'..." % args.agent)

    pcla = PCLA(args.agent, vehicle, route_path, client)
    _log("Agent ready. Running (max_steps=%s). Ctrl+C to stop." % (args.max_steps if args.max_steps else "inf"))

    # ---- Attach observer camera for frame capture ----
    camera_tf = carla.Transform(
        carla.Location(x=2.0, z=1.55), carla.Rotation(pitch=0, yaw=0, roll=0)
    )
    cam_bp = bp.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", "1280")
    cam_bp.set_attribute("image_size_y", "720")
    cam_bp.set_attribute("fov", "120")
    camera = world.spawn_actor(cam_bp, camera_tf, attach_to=vehicle)

    frame_index = [0]
    frame_ready = threading.Event()

    def on_image(image):
        i = frame_index[0]
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

    try:
        step = 0
        while True:
            if args.max_steps and step >= args.max_steps:
                _log("Reached max steps %d." % args.max_steps)
                break
            frame_index[0] = step
            frame_ready.clear()
            ego_action = pcla.get_action()
            last_control["control"] = ego_action
            vehicle.apply_control(ego_action)
            world.tick()
            # Wait for camera frame
            frame_ready.wait(timeout=max(10.0, args.timeout))
            step += 1
            if step <= 10 or step % 100 == 0:
                _log("  step %d" % step)
    except KeyboardInterrupt:
        _log("Interrupted by user.")
    finally:
        _log("Cleaning up...")
        try:
            camera.destroy()
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
