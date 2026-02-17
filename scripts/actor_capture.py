"""
Capture ~5s of frames with one animated actor: walker crossing the road, or oncoming vehicle.
Uses the same front_wide camera as weather_capture. Output: data/<preset>_frames/frame_0000.png ...
"""
import argparse
import math
import os
import random
import sys
import threading
import numpy as np
from PIL import Image
import carla

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from set_scene import (
    DEFAULT_CLOUDINESS,
    DEFAULT_PRECIPITATION,
    DEFAULT_WETNESS,
    DEFAULT_SUN_ALTITUDE_ANGLE,
)

# ~5s at 10 FPS
DEFAULT_NUM_FRAMES = 50
FIXED_DELTA_SECONDS = 0.1

PRESETS = {
    "walker": {
        "out_dir": "data/walker_frames",
        "num_frames": DEFAULT_NUM_FRAMES,
    },
    "vehicle": {
        "out_dir": "data/oncoming_vehicle_frames",
        "num_frames": DEFAULT_NUM_FRAMES,
    },
}

# Fixed walker placement: sidewalk on the right in front, then cross to left (meters)
# Further ahead = more of the walk in shot before they pass the car
WALKER_AHEAD_M = 15.0
WALKER_LATERAL_M = 4.0
# Ticks to run (no capture) after spawn so walker reaches sidewalk; then crossing starts = recording starts
WALKER_SETTLE_TICKS = 15
# Ticks to run (no capture) after spawning oncoming vehicle so it settles on road before we record
VEHICLE_SETTLE_TICKS = 10
# Oncoming vehicle: straight line at constant speed (m/s) instead of autopilot for deterministic path
ONCOMING_SPEED_MPS = 7.0
DIST_MIN_VEHICLE = 20.0
DIST_MAX_VEHICLE = 60.0
SEED = 42


def _ego_forward(ego_transform: carla.Transform) -> carla.Vector3D:
    return ego_transform.get_forward_vector()

def _ego_right(ego_transform: carla.Transform) -> carla.Vector3D:
    return ego_transform.get_right_vector()

def _dot(v: carla.Vector3D, w: carla.Vector3D) -> float:
    return v.x * w.x + v.y * w.y + v.z * w.z


def get_walker_crossing_fixed(ego_transform: carla.Transform):
    """
    Walker start = sidewalk on the right in front of the car; target = same distance ahead, left side.
    Uses fixed offsets so the pedestrian is explicitly in view (right sidewalk → cross to left).
    """
    ego_loc = ego_transform.location
    forward = _ego_forward(ego_transform)
    right = _ego_right(ego_transform)
    # Start: WALKER_AHEAD_M ahead, WALKER_LATERAL_M to the right (sidewalk on right)
    start_loc = carla.Location(
        x=ego_loc.x + WALKER_AHEAD_M * forward.x + WALKER_LATERAL_M * right.x,
        y=ego_loc.y + WALKER_AHEAD_M * forward.y + WALKER_LATERAL_M * right.y,
        z=ego_loc.z,
    )
    # Target: same distance ahead, to the left (crossing)
    target_loc = carla.Location(
        x=ego_loc.x + WALKER_AHEAD_M * forward.x - WALKER_LATERAL_M * right.x,
        y=ego_loc.y + WALKER_AHEAD_M * forward.y - WALKER_LATERAL_M * right.y,
        z=ego_loc.z,
    )
    return start_loc, target_loc


def get_walker_crossing_fixed_with_offset(ego_transform: carla.Transform, offset_along_road_m: float = 0.0):
    """
    Same as get_walker_crossing_fixed, but shift both start and target along the road
    by offset_along_road_m (in the forward direction). Use to place multiple walkers
    spread along the same crosswalk.
    """
    start_loc, target_loc = get_walker_crossing_fixed(ego_transform)
    if offset_along_road_m == 0.0:
        return start_loc, target_loc
    forward = _ego_forward(ego_transform)
    start_loc = carla.Location(
        x=start_loc.x + offset_along_road_m * forward.x,
        y=start_loc.y + offset_along_road_m * forward.y,
        z=start_loc.z,
    )
    target_loc = carla.Location(
        x=target_loc.x + offset_along_road_m * forward.x,
        y=target_loc.y + offset_along_road_m * forward.y,
        z=target_loc.z,
    )
    return start_loc, target_loc


def get_walker_crossing_from_waypoint(waypoint):
    """
    Prefer map sidewalk lanes for walker start (right) and target (left).
    From a driving-waypoint, follow get_right_lane() / get_left_lane() until
    lane_type == Sidewalk; use those waypoint locations so the pedestrian
    spawns on a real crosswalk/sidewalk. Falls back to get_walker_crossing_fixed
    if no sidewalk lanes are found on either side.
    """
    right = waypoint.get_right_lane()
    while right is not None and right.lane_type != carla.LaneType.Driving:
        if right.lane_type == carla.LaneType.Sidewalk:
            start_loc = right.transform.location
            left = waypoint.get_left_lane()
            while left is not None and left.lane_type != carla.LaneType.Driving:
                if left.lane_type == carla.LaneType.Sidewalk:
                    target_loc = left.transform.location
                    return start_loc, target_loc
                left = left.get_left_lane()
            break
        right = right.get_right_lane()
    return get_walker_crossing_fixed(waypoint.transform)


def find_oncoming_spawn_point(world: carla.World, ego_transform: carla.Transform):
    """
    Pick a vehicle spawn point ahead of ego with rotation roughly opposite (oncoming).
    """
    ego_loc = ego_transform.location
    ego_yaw = ego_transform.rotation.yaw
    forward = _ego_forward(ego_transform)
    spawn_points = world.get_map().get_spawn_points()

    candidates = []
    for i, sp in enumerate(spawn_points):
        if i == 0:
            continue
        to_spawn = carla.Vector3D(
            sp.location.x - ego_loc.x,
            sp.location.y - ego_loc.y,
            sp.location.z - ego_loc.z,
        )
        dist = math.sqrt(to_spawn.x ** 2 + to_spawn.y ** 2 + to_spawn.z ** 2)
        if dist < DIST_MIN_VEHICLE or dist > DIST_MAX_VEHICLE:
            continue
        if _dot(to_spawn, forward) <= 0:
            continue
        # Spawn should face back toward ego: spawn's forward ~ opposite to (spawn -> ego)
        sp_forward = sp.get_forward_vector()
        # We want sp_forward pointing toward ego, i.e. -to_spawn
        toward_ego = carla.Vector3D(-to_spawn.x, -to_spawn.y, -to_spawn.z)
        dot_f = _dot(sp_forward, toward_ego)
        # Prefer spawns that already face toward ego (dot_f > 0); or at least not too far
        sp_yaw = sp.rotation.yaw
        yaw_diff = abs((sp_yaw - (ego_yaw + 180)) % 360)
        if yaw_diff > 180:
            yaw_diff = 360 - yaw_diff
        candidates.append((yaw_diff, dot_f, sp))

    if not candidates:
        raise RuntimeError(
            "Could not find an oncoming vehicle spawn point ahead of ego. "
            "Try a different map or ego spawn."
        )
    # Prefer similar yaw (opposite to ego) and forward pointing toward ego
    candidates.sort(key=lambda x: (x[0], -x[1]))
    sp = candidates[0][2]
    # Override rotation so vehicle faces exactly toward ego (straight path)
    yaw_toward_ego = (ego_yaw + 180.0) % 360.0
    straight_transform = carla.Transform(
        sp.location,
        carla.Rotation(pitch=0, yaw=yaw_toward_ego, roll=0),
    )
    return straight_transform


def main():
    parser = argparse.ArgumentParser(
        description="Capture ~5s with one actor: walker crossing road or oncoming vehicle."
    )
    parser.add_argument(
        "preset",
        choices=list(PRESETS),
        help="Scenario: walker (crossing) or vehicle (oncoming).",
    )
    parser.add_argument(
        "-n", "--num-frames",
        type=int,
        default=None,
        help=f"Number of frames to capture (default: preset, ~5s = {DEFAULT_NUM_FRAMES} at 10 FPS).",
    )
    args = parser.parse_args()
    preset = PRESETS[args.preset]
    out_dir = preset["out_dir"]
    num_frames = args.num_frames if args.num_frames is not None else preset["num_frames"]

    random.seed(SEED)
    os.makedirs(out_dir, exist_ok=True)

    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(30.0)
    world = client.get_world()

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)

    # Default weather
    weather = world.get_weather()
    weather.cloudiness = DEFAULT_CLOUDINESS
    weather.precipitation = DEFAULT_PRECIPITATION
    weather.wetness = DEFAULT_WETNESS
    weather.sun_altitude_angle = DEFAULT_SUN_ALTITUDE_ANGLE
    world.set_weather(weather)

    bp = world.get_blueprint_library()
    vehicle_bp = bp.filter("vehicle.tesla.model3")[0]
    ego_spawn = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(vehicle_bp, ego_spawn)
    vehicle.set_autopilot(False)

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
        path = os.path.join(out_dir, f"frame_{i:04d}.png")
        Image.fromarray(rgb).save(path)
        frame_ready.set()

    camera.listen(on_image)

    # Warm-up
    for _ in range(10):
        frame_ready.clear()
        world.tick()
        if not frame_ready.wait(timeout=5.0):
            raise RuntimeError("Timeout during warm-up tick")

    walker_actor = None
    walker_controller = None
    oncoming_vehicle = None

    try:
        if args.preset == "walker":
            start_loc, target_loc = get_walker_crossing_fixed(ego_spawn)
            # Prefer a standard adult pedestrian so the walker is clearly visible (no props)
            all_walkers = list(bp.filter("walker.pedestrian.*"))
            walker_bp = next((b for b in all_walkers if b.id == "walker.pedestrian.0001"), all_walkers[0])
            walker_actor = world.try_spawn_actor(
                walker_bp, carla.Transform(start_loc)
            )
            if walker_actor is None:
                raise RuntimeError("Failed to spawn walker at chosen location.")
            controller_bp = bp.find("controller.ai.walker")
            walker_controller = world.spawn_actor(
                controller_bp, carla.Transform(), walker_actor
            )
            world.tick()
            if not frame_ready.wait(timeout=5.0):
                pass
            walker_controller.start()
            # First send walker to sidewalk (start_loc); sim often spawns them on the road
            walker_controller.go_to_location(start_loc)
            walker_controller.set_max_speed(1.0)
            for _ in range(WALKER_SETTLE_TICKS):
                frame_ready.clear()
                world.tick()
                if not frame_ready.wait(timeout=5.0):
                    pass
            # Now start crossing and recording together (animation start = recording start)
            walker_controller.go_to_location(target_loc)
            walker_controller.set_max_speed(1.0)

        elif args.preset == "vehicle":
            oncoming_spawn = find_oncoming_spawn_point(world, ego_spawn)
            oncoming_vehicle = world.spawn_actor(vehicle_bp, oncoming_spawn)
            # Constant velocity straight toward ego (deterministic, no autopilot path variation)
            oncoming_vehicle.enable_constant_velocity(carla.Vector3D(ONCOMING_SPEED_MPS, 0, 0))
            for _ in range(VEHICLE_SETTLE_TICKS):
                frame_ready.clear()
                world.tick()
                if not frame_ready.wait(timeout=5.0):
                    pass

        capturing[0] = True
        for i in range(num_frames):
            frame_index[0] = i
            frame_ready.clear()
            world.tick()
            if not frame_ready.wait(timeout=5.0):
                raise RuntimeError(f"Timeout waiting for frame {i}")
    finally:
        capturing[0] = False
        if walker_controller is not None:
            try:
                walker_controller.stop()
                walker_controller.destroy()
            except Exception:
                pass
        if walker_actor is not None:
            try:
                walker_actor.destroy()
            except Exception:
                pass
        if oncoming_vehicle is not None:
            try:
                oncoming_vehicle.destroy()
            except Exception:
                pass
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

    print(f"Captured {num_frames} frames to {out_dir}")


if __name__ == "__main__":
    main()
