"""
Drive ego from spawn 0 to spawn 10 (A→B) with autopilot, render camera view.
Uses BasicAgent when available for route following; otherwise falls back to
vehicle.set_autopilot(True) (route not guaranteed A→B).
Captures the full trip until the agent reaches B (or MAX_FRAMES at 10 fps).
Output: data/closed_loop_frames/frame_0000.png, frame_0001.png, ...
"""
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
    os.makedirs(OUT_DIR, exist_ok=True)

    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(30.0)
    world = client.get_world()

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

    bp = world.get_blueprint_library()
    carla_map = world.get_map()
    spawns = carla_map.get_spawn_points()
    start_spawn = spawns[0]
    end_idx = min(10, len(spawns) - 1)
    end_spawn = spawns[end_idx]

    # Prefer driving-lane waypoint for correct orientation; fall back to spawn if collision
    start_wp = carla_map.get_waypoint(
        start_spawn.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    vehicle_bp = bp.filter("vehicle.tesla.model3")[0]
    vehicle = world.try_spawn_actor(vehicle_bp, start_wp.transform)
    if vehicle is None:
        vehicle = world.spawn_actor(vehicle_bp, start_spawn)

    agent, use_agent = _get_basic_agent(
        vehicle,
        world,
        target_speed=TARGET_SPEED_KMH,
        opt_dict={"sampling_resolution": SAMPLING_RESOLUTION},
    )
    if use_agent:
        # Route from correct lane waypoint so planner doesn't start on wrong lane
        agent.set_destination(
            end_spawn.location,
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
        path = os.path.join(OUT_DIR, f"frame_{i:04d}.png")
        Image.fromarray(rgb).save(path)
        frame_ready.set()

    camera.listen(on_image)

    # Warm-up
    for _ in range(WARMUP_TICKS):
        frame_ready.clear()
        if use_agent:
            vehicle.apply_control(agent.run_step())
        world.tick()
        if not frame_ready.wait(timeout=5.0):
            raise RuntimeError("Timeout during warm-up tick")

    capturing[0] = True
    try:
        i = 0
        done_early = False
        while i < MAX_FRAMES:
            frame_index[0] = i
            frame_ready.clear()
            if use_agent:
                vehicle.apply_control(agent.run_step())
                if agent.done():
                    done_early = True
                    break
            world.tick()
            if not frame_ready.wait(timeout=5.0):
                raise RuntimeError(f"Timeout waiting for frame {i}")
            i += 1
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


if __name__ == "__main__":
    main()
