"""
Capture static frames while sweeping one weather parameter (preset: sunset | cloudiness | wetness | precipitation).
Output: data/<preset>_frames/frame_0000.png ...
"""
import argparse
import os
import threading
import numpy as np
from PIL import Image
import carla

PRESETS = {
    "sunset": {
        "param": "sun_altitude_angle",
        "start": 45,
        "end": -18,
        "step": 1,
        "num_frames": 64,
        "out_dir": "data/sunset_frames",
    },
    "cloudiness": {
        "param": "cloudiness",
        "start": 0,
        "end": 100,
        "step": 2,
        "num_frames": 51,
        "out_dir": "data/cloudiness_frames",
    },
    "wetness": {
        "param": "wetness",
        "start": 0,
        "end": 100,
        "step": 2,
        "num_frames": 51,
        "out_dir": "data/wetness_frames",
    },
    "precipitation": {
        "param": "precipitation",
        "start": 0,
        "end": 100,
        "step": 2,
        "num_frames": 51,
        "out_dir": "data/precipitation_frames",
    },
}


def get_value_for_frame(preset: dict, i: int) -> float:
    start, end, step = preset["start"], preset["end"], preset["step"]
    if start > end:
        return float(start - i * step)
    return float(start + i * step)


def main():
    parser = argparse.ArgumentParser(
        description="Capture frames while sweeping a weather parameter (preset)."
    )
    parser.add_argument(
        "preset",
        choices=list(PRESETS),
        help="Weather sweep preset: sunset (45→-18°), or cloudiness/wetness/precipitation (0→100).",
    )
    args = parser.parse_args()
    preset = PRESETS[args.preset]
    param = preset["param"]
    num_frames = preset["num_frames"]
    out_dir = preset["out_dir"]

    os.makedirs(out_dir, exist_ok=True)

    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(30.0)
    world = client.get_world()

    original_settings = world.get_settings()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.1
    world.apply_settings(settings)

    bp = world.get_blueprint_library()

    vehicle_bp = bp.filter("vehicle.tesla.model3")[0]
    spawn = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn)

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

    # Warm-up: let vehicle settle for 10 ticks before capturing
    for _ in range(10):
        frame_ready.clear()
        world.tick()
        if not frame_ready.wait(timeout=5.0):
            raise RuntimeError("Timeout during warm-up tick")

    capturing[0] = True
    try:
        for i in range(num_frames):
            frame_index[0] = i
            weather = world.get_weather()
            value = get_value_for_frame(preset, i)
            setattr(weather, param, value)
            world.set_weather(weather)
            frame_ready.clear()
            world.tick()
            if not frame_ready.wait(timeout=5.0):
                raise RuntimeError(f"Timeout waiting for frame {i}")
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

    print(f"Captured {num_frames} frames to {out_dir}")


if __name__ == "__main__":
    main()
