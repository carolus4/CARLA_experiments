"""
Drive ego from spawn 0 to spawn 10 (A→B) with autopilot, render camera view.
Uses BasicAgent when available for route following; otherwise falls back to
vehicle.set_autopilot(True) (route not guaranteed A→B).
Captures the full trip until the agent reaches B (or MAX_FRAMES at 10 fps).
Output: data/closed_loop_frames/frame_0000.png, frame_0001.png, ...

Use --hud to bake a throttle/brake/steer overlay onto saved frames (headless-safe).
"""
import argparse
import os
import sys
import threading
import numpy as np
from PIL import Image
import carla

# Prepend vendored CARLA PythonAPI (agents) so BasicAgent is importable
_script_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.dirname(_script_dir)
_vendored = os.path.join(_repo_root, "third_party", "carla_pythonapi")
if os.path.isdir(_vendored) and _vendored not in sys.path:
    sys.path.insert(0, _vendored)

sys.path.insert(0, _script_dir)
from set_scene import (
    DEFAULT_CLOUDINESS,
    DEFAULT_PRECIPITATION,
    DEFAULT_WETNESS,
    DEFAULT_SUN_ALTITUDE_ANGLE,
)

OUT_DIR = "data/closed_loop_frames"
FIXED_DELTA_SECONDS = 0.1  # 10 fps
WARMUP_TICKS = 10
MAX_FRAMES = 6000  # safety cap: 10 min at 10 fps
# Agent: lower speed and denser waypoints for reliable turns
TARGET_SPEED_KMH = 15
SAMPLING_RESOLUTION = 1.0
# Route A→B: spawn 0 = A, spawn END_SPAWN_INDEX = B (match scripts/2Dplot_route.py)
END_SPAWN_INDEX = 10


def _get_basic_agent(vehicle, world, target_speed=20, opt_dict=None):
    """Import BasicAgent and return (agent, True) or (None, False) if unavailable."""
    opt_dict = opt_dict or {}
    try:
        from agents.navigation.basic_agent import BasicAgent
        return BasicAgent(vehicle, target_speed=target_speed, opt_dict=opt_dict), True
    except ImportError:
        pass
    # Try adding CARLA PythonAPI path (agents live alongside carla package)
    try:
        carla_root = os.environ.get("CARLA_ROOT")
        if carla_root:
            api_path = os.path.join(carla_root, "PythonAPI", "carla")
            if os.path.isdir(api_path):
                if api_path not in sys.path:
                    sys.path.insert(0, api_path)
                from agents.navigation.basic_agent import BasicAgent
                return BasicAgent(vehicle, target_speed=target_speed, opt_dict=opt_dict), True
    except ImportError:
        pass
    try:
        # Parent of carla package often contains 'agents' in CARLA source layout
        carla_dir = os.path.dirname(os.path.abspath(carla.__file__))
        parent = os.path.dirname(carla_dir)
        if parent not in sys.path:
            sys.path.insert(0, parent)
        from agents.navigation.basic_agent import BasicAgent
        return BasicAgent(vehicle, target_speed=target_speed, opt_dict=opt_dict), True
    except ImportError:
        pass
    return None, False


def main():
    # Suppress Pygame welcome message before any pygame import (e.g. when using --hud).
    if "PYGAME_HIDE_SUPPORT_PROMPT" not in os.environ:
        os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

    parser = argparse.ArgumentParser(description="Closed-loop drive A→B with optional HUD overlay.")
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
        "--timeout",
        type=float,
        default=60.0,
        metavar="SEC",
        help="CARLA client timeout in seconds (default: 60). Increase if the simulator is slow.",
    )
    args = parser.parse_args()
    use_hud = args.hud

    def _handle_carla_timeout(exc):
        if "time-out" in str(exc).lower() or "timeout" in str(exc).lower():
            print(
                "CARLA simulator did not respond in time. "
                "Is the server running on 127.0.0.1:2000? "
                "Try --timeout 120 or restart the simulator.",
                file=sys.stderr,
            )
            raise SystemExit(1) from exc
        raise

    def _log(msg):
        print(msg, flush=True)

    os.makedirs(OUT_DIR, exist_ok=True)

    hud = None
    last_tick_data = {"control": None}
    if use_hud:
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

    try:
        _log("Connecting to CARLA at 127.0.0.1:2000 (timeout={}s)...".format(args.timeout))
        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(args.timeout)
        _log("Waiting for world...")
        world = client.get_world()
        _log("Connected. Loading world settings...")

        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
        world.apply_settings(settings)

        # Default scene weather
        weather = world.get_weather()
        weather.cloudiness = DEFAULT_CLOUDINESS
        weather.precipitation = DEFAULT_PRECIPITATION
        weather.wetness = DEFAULT_WETNESS
        weather.sun_altitude_angle = DEFAULT_SUN_ALTITUDE_ANGLE
        world.set_weather(weather)

        _log("Starting fresh: destroying existing vehicles...")
        # Start fresh: destroy any existing vehicles (e.g. from a previous run that didn't cleanup)
        for actor in world.get_actors().filter("vehicle.*"):
            try:
                actor.destroy()
            except Exception:
                pass
        # One tick so the server processes the destruction before we spawn
        world.tick()
        _log("Loading blueprint library and map...")
        bp = world.get_blueprint_library()
        carla_map = world.get_map()
        spawns = carla_map.get_spawn_points()
        if not spawns:
            raise RuntimeError("No spawn points on this map.")
        _log("Spawning vehicle at spawn 0...")
        start_spawn = spawns[0]
        end_idx = min(END_SPAWN_INDEX, len(spawns) - 1)
        end_spawn = spawns[end_idx]

        # Prefer driving-lane waypoint for correct orientation; fall back to spawn if collision
        # Use same waypoint logic as 2Dplot_route so the planned route matches the map (same A→B).
        start_wp = carla_map.get_waypoint(
            start_spawn.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        end_wp = carla_map.get_waypoint(
            end_spawn.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        vehicle_bp = bp.filter("vehicle.tesla.model3")[0]
        vehicle = world.try_spawn_actor(vehicle_bp, start_wp.transform)
        if vehicle is None:
            vehicle = world.spawn_actor(vehicle_bp, start_spawn)
        _log("Vehicle spawned. Setting up agent and camera...")

        agent, use_agent = _get_basic_agent(
            vehicle,
            world,
            target_speed=TARGET_SPEED_KMH,
            opt_dict={"sampling_resolution": SAMPLING_RESOLUTION},
        )
        if use_agent:
            # Destination must be the driving-lane waypoint at end spawn (same as 2Dplot_route),
            # not the raw spawn location, so the planner computes the same route (e.g. two lefts).
            agent.set_destination(
                end_wp.transform.location,
                start_location=start_wp.transform.location,
            )
        else:
            vehicle.set_autopilot(True)
            print("Warning: BasicAgent not found; using plain autopilot. Route is not guaranteed A→B.")

        camera_tf = carla.Transform(
            carla.Location(x=2.0, z=1.55), carla.Rotation(pitch=0, yaw=0, roll=0)
        )
        cam_bp = bp.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "1280")
        cam_bp.set_attribute("image_size_y", "720")
        cam_bp.set_attribute("fov", "120")
        camera = world.spawn_actor(cam_bp, camera_tf, attach_to=vehicle)
        _log("Camera attached. Warm-up: waiting for first frame (up to {:.0f}s per tick)...".format(max(10.0, args.timeout)))

        # Timeout for waiting for one camera frame (warm-up or main loop). Slow sims may need more.
        frame_wait_timeout = max(10.0, args.timeout)

        frame_index = [0]
        frame_ready = threading.Event()
        capturing = [False]

        def on_image(image):
            if not capturing[0]:
                frame_ready.set()
                return
            i = frame_index[0]
            arr = np.frombuffer(
                image.raw_data, dtype=np.uint8
            ).reshape((image.height, image.width, 4))
            rgb = arr[:, :, :3][:, :, ::-1]
            if use_hud and hud is not None:
                snapshot = world.get_snapshot()
                ts = getattr(snapshot, "timestamp", None)
                sim_time = getattr(ts, "elapsed_seconds", 0) if ts else 0
                map_name = world.get_map().name if world else None
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
                    last_tick_data["control"],
                    image.frame,
                    sim_time,
                    map_name=map_name,
                    velocity=snap_velocity,
                    transform=snap_transform,
                )
            path = os.path.join(OUT_DIR, f"frame_{i:04d}.png")
            Image.fromarray(rgb).save(path)
            frame_ready.set()

        camera.listen(on_image)

        # Warm-up and main loop: always destroy camera/vehicle and restore world on exit.
        try:
            # Warm-up: wait for at least one camera frame so the sensor is producing.
            for w in range(WARMUP_TICKS):
                _log("  Warm-up tick {}/{}...".format(w + 1, WARMUP_TICKS))
                frame_ready.clear()
                try:
                    if use_agent:
                        c = agent.run_step()
                        last_tick_data["control"] = c
                        vehicle.apply_control(c)
                    world.tick()
                except RuntimeError as e:
                    _handle_carla_timeout(e)
                if not frame_ready.wait(timeout=frame_wait_timeout):
                    print(
                        "No camera frame received in time. "
                        "Check that the simulator is ticking and the camera is attached. "
                        "Try --timeout with a larger value.",
                        file=sys.stderr,
                    )
                    raise SystemExit(1) from RuntimeError("Timeout during warm-up tick")

            _log("Warm-up done. Capturing frames (max {})...".format(MAX_FRAMES))
            capturing[0] = True
            i = 0
            done_early = False
            while i < MAX_FRAMES:
                frame_index[0] = i
                frame_ready.clear()
                try:
                    if use_agent:
                        control = agent.run_step()
                        last_tick_data["control"] = control
                        vehicle.apply_control(control)
                        if agent.done():
                            done_early = True
                            break
                    else:
                        last_tick_data["control"] = None
                    world.tick()
                except RuntimeError as e:
                    _handle_carla_timeout(e)
                if not frame_ready.wait(timeout=frame_wait_timeout):
                    _log("Timeout waiting for frame {}.".format(i))
                    raise RuntimeError(f"Timeout waiting for frame {i}")
                i += 1
                # Progress every 50 frames, or every 10 for the first 100
                if i <= 100 and i % 10 == 0:
                    _log("  Frame {}...".format(i))
                elif i > 100 and i % 50 == 0:
                    _log("  Frame {}...".format(i))
            num_captured = (i + 1) if done_early else i
        finally:
            try:
                camera.destroy()
            except Exception:
                pass
            try:
                vehicle.destroy()
            except Exception:
                pass
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass
            try:
                weather = world.get_weather()
                weather.cloudiness = DEFAULT_CLOUDINESS
                weather.precipitation = DEFAULT_PRECIPITATION
                weather.wetness = DEFAULT_WETNESS
                weather.sun_altitude_angle = DEFAULT_SUN_ALTITUDE_ANGLE
                world.set_weather(weather)
            except Exception:
                pass

        print(f"Captured {num_captured} frames to {OUT_DIR}")
    except RuntimeError as e:
        _handle_carla_timeout(e)


if __name__ == "__main__":
    main()
