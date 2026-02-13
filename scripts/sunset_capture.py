"""
Capture static frames with sun altitude 45° → -18° (1° per tick).
Output: data/sunset_frames/frame_0000.png … frame_0062.png.
"""
import os
import threading
import numpy as np
from PIL import Image
import carla

OUT_DIR = "data/sunset_frames"
NUM_FRAMES = 63  # 45 - (-18) = 63, 1° per tick
os.makedirs(OUT_DIR, exist_ok=True)

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(30.0)
world = client.get_world()

original_settings = world.get_settings()

# Enable sync mode
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.1
world.apply_settings(settings)

bp = world.get_blueprint_library()

vehicle_bp = bp.filter("vehicle.tesla.model3")[0]
spawn = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn)

# Single camera (front_wide style)
camera_tf = carla.Transform(carla.Location(x=2.0, z=1.55), carla.Rotation(pitch=0, yaw=0, roll=0))
cam_bp = bp.find("sensor.camera.rgb")
cam_bp.set_attribute("image_size_x", "1280")
cam_bp.set_attribute("image_size_y", "720")
cam_bp.set_attribute("fov", "120")
camera = world.spawn_actor(cam_bp, camera_tf, attach_to=vehicle)

# Coordinate: main loop sets frame index, tick; callback saves and signals
frame_index = [0]
frame_ready = threading.Event()


def on_image(image):
    i = frame_index[0]
    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    rgb = arr[:, :, :3][:, :, ::-1]
    path = os.path.join(OUT_DIR, f"frame_{i:04d}.png")
    Image.fromarray(rgb).save(path)
    frame_ready.set()


camera.listen(on_image)

try:
    for i in range(NUM_FRAMES):
        frame_index[0] = i
        weather = world.get_weather()
        weather.sun_altitude_angle = 45.0 - i
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

print(f"Captured {NUM_FRAMES} frames to {OUT_DIR}")
