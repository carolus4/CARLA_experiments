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

# Spawn RGB camera
camera_bp = bp.find("sensor.camera.rgb")
camera_bp.set_attribute("image_size_x", "1280")
camera_bp.set_attribute("image_size_y", "720")
camera_bp.set_attribute("fov", "90")

camera = world.spawn_actor(
    camera_bp,
    carla.Transform(carla.Location(x=1.5, z=1.7)),
    attach_to=vehicle
)

saved = {"done": False}

def save_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    rgb = array[:, :, :3][:, :, ::-1]  # BGRA → RGB

    img = Image.fromarray(rgb)
    path = os.path.join(OUT_DIR, f"frame_{image.frame}.png")
    img.save(path)

    saved["done"] = True
    camera.stop()

camera.listen(save_image)
vehicle.set_autopilot(True)

# Wait for first frame
timeout = time.time() + 10
while not saved["done"] and time.time() < timeout:
    time.sleep(0.1)

camera.destroy()
vehicle.destroy()

print("Saved first rendered frame.")