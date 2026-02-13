import os
import time
import numpy as np
from PIL import Image
import carla

from world_inspect import inspect_world

OUT_DIR = "data/forward_stack"
os.makedirs(OUT_DIR, exist_ok=True)

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(30.0)
world = client.get_world()

original_settings = world.get_settings()

# Enable sync mode (so frames are aligned)
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.1
world.apply_settings(settings)

bp = world.get_blueprint_library()

vehicle_bp = bp.filter("vehicle.tesla.model3")[0]
spawn = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn)

# Tesla-ish forward mount: behind windshield-ish
camera_tf = carla.Transform(carla.Location(x=2.0, z=1.55), carla.Rotation(pitch=0, yaw=0, roll=0))

def make_rgb_camera(fov, width=1280, height=720):
    cam_bp = bp.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(width))
    cam_bp.set_attribute("image_size_y", str(height))
    cam_bp.set_attribute("fov", str(fov))
    return world.spawn_actor(cam_bp, camera_tf, attach_to=vehicle)

# Tesla-ish FOVs (approx): wide / main / narrow
cams = {
    "front_wide": make_rgb_camera(120),
    "front_main": make_rgb_camera(50),
    "front_narrow": make_rgb_camera(35),
}

# Add a chase camera so you can SEE the car and camera marker
chase_tf = carla.Transform(
    carla.Location(x=-8.0, z=4.0),
    carla.Rotation(pitch=-15, yaw=0, roll=0),
)
chase_cam_bp = bp.find("sensor.camera.rgb")
chase_cam_bp.set_attribute("image_size_x", "1280")
chase_cam_bp.set_attribute("image_size_y", "720")
chase_cam_bp.set_attribute("fov", "90")
cams["chase"] = world.spawn_actor(chase_cam_bp, chase_tf, attach_to=vehicle)

frames = {}

def save_image(name):
    def _cb(image):
        if name in frames:
            return
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        rgb = arr[:, :, :3][:, :, ::-1]
        Image.fromarray(rgb).save(os.path.join(OUT_DIR, f"{name}_{image.frame}.png"))
        frames[name] = image.frame
        cams[name].stop()
    return _cb

# Debug marker at the camera mount location (visible in chase view)
mount_world = cams["front_main"].get_transform().location
world.debug.draw_point(mount_world, size=0.18, color=carla.Color(255, 0, 255), life_time=10.0)
world.debug.draw_string(mount_world + carla.Location(z=0.3), "FORWARD_CAM_MOUNT",
                        draw_shadow=False, color=carla.Color(255, 255, 255), life_time=10.0)

for name, cam in cams.items():
    cam.listen(save_image(name))

try:
    # Tick until we got all 4 images
    timeout = time.time() + 10
    while len(frames) < len(cams) and time.time() < timeout:
        world.tick()

    if len(frames) < len(cams):
        missing = set(cams.keys()) - set(frames.keys())
        raise RuntimeError(f"Timeout waiting for frames from: {missing}")

finally:
    # Inspect the world after capture
    inspect_world(world)
    # Cleanup
    for cam in cams.values():
        try:
            cam.destroy()
        except Exception:
            pass
    try:
        vehicle.destroy()
    except Exception:
        pass
    # Restore settings
    try:
        world.apply_settings(original_settings)
    except Exception:
        pass

print("Captured frames:", frames)
print("Wrote files to:", OUT_DIR)