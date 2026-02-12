import os
import time
import numpy as np
from PIL import Image
import carla

OUT_DIR = "data/renders"
os.makedirs(OUT_DIR, exist_ok=True)

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(30.0)
world = client.get_world()

bp = world.get_blueprint_library()

# Spawn vehicle
vehicle_bp = bp.filter("vehicle.tesla.model3")[0]
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

# Common camera transform
camera_transform = carla.Transform(carla.Location(x=1.5, z=1.7))

# RGB
rgb_bp = bp.find("sensor.camera.rgb")
rgb_bp.set_attribute("image_size_x", "1280")
rgb_bp.set_attribute("image_size_y", "720")
rgb = world.spawn_actor(rgb_bp, camera_transform, attach_to=vehicle)

# Depth
depth_bp = bp.find("sensor.camera.depth")
depth_bp.set_attribute("image_size_x", "1280")
depth_bp.set_attribute("image_size_y", "720")
depth = world.spawn_actor(depth_bp, camera_transform, attach_to=vehicle)

# Segmentation
seg_bp = bp.find("sensor.camera.semantic_segmentation")
seg_bp.set_attribute("image_size_x", "1280")
seg_bp.set_attribute("image_size_y", "720")
seg = world.spawn_actor(seg_bp, camera_transform, attach_to=vehicle)

frames = {"rgb": None, "depth": None, "seg": None}

def save_rgb(image):
    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    rgb_img = arr[:, :, :3][:, :, ::-1]
    Image.fromarray(rgb_img).save(os.path.join(OUT_DIR, f"rgb_{image.frame}.png"))
    frames["rgb"] = image.frame
    rgb.stop()

def save_depth(image):
    image.convert(carla.ColorConverter.LogarithmicDepth)
    image.save_to_disk(os.path.join(OUT_DIR, f"depth_{image.frame}.png"))
    frames["depth"] = image.frame
    depth.stop()

def save_seg(image):
    image.save_to_disk(os.path.join(OUT_DIR, f"seg_{image.frame}.png"))
    frames["seg"] = image.frame
    seg.stop()

rgb.listen(save_rgb)
depth.listen(save_depth)
seg.listen(save_seg)

vehicle.set_autopilot(True)

timeout = time.time() + 10
while not all(frames.values()) and time.time() < timeout:
    time.sleep(0.1)

rgb.destroy()
depth.destroy()
seg.destroy()
vehicle.destroy()

print("Captured frames:", frames)