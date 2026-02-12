import os, time, json
from datetime import datetime

import carla
import numpy as np
from PIL import Image

OUT_ROOT = "data/rig"
RUN_ID = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
OUT_DIR = os.path.join(OUT_ROOT, RUN_ID)
os.makedirs(OUT_DIR, exist_ok=True)

def save_rgb(image: carla.Image, path: str):
    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    rgb = arr[:, :, :3][:, :, ::-1]  # BGRA -> RGB
    Image.fromarray(rgb).save(path)

def main():
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(30.0)
    world = client.get_world()

    # --- optional: make capture deterministic-ish
    settings = world.get_settings()
    original_settings = settings
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.1  # 10 FPS ticks
    world.apply_settings(settings)

    bp = world.get_blueprint_library()
    actors = []

    try:
        # Spawn one vehicle (static)
        vehicle_bp = bp.filter("vehicle.tesla.model3")[0]
        spawn = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn)
        actors.append(vehicle)
        vehicle.set_autopilot(False)

        # --- Define a "reasonable" camera rig (all attached to vehicle)
        # You can tweak these transforms to match what you want.
        # Units are meters; +x forward, +y right, +z up (vehicle local frame).
        rig = {
            "front": carla.Transform(carla.Location(x=2.0, z=1.6), carla.Rotation(pitch=0, yaw=0, roll=0)),
            "rear":  carla.Transform(carla.Location(x=-2.2, z=1.6), carla.Rotation(pitch=0, yaw=180, roll=0)),
            "left":  carla.Transform(carla.Location(x=0.2, y=-0.8, z=1.6), carla.Rotation(pitch=0, yaw=-90, roll=0)),
            "right": carla.Transform(carla.Location(x=0.2, y=0.8, z=1.6), carla.Rotation(pitch=0, yaw=90, roll=0)),
            "roof":  carla.Transform(carla.Location(x=0.2, z=2.3), carla.Rotation(pitch=-10, yaw=0, roll=0)),
        }

        # Camera blueprint (RGB)
        cam_bp = bp.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "1280")
        cam_bp.set_attribute("image_size_y", "720")
        cam_bp.set_attribute("fov", "90")

        # Spawn rig cameras
        rig_cams = {}
        for name, tf in rig.items():
            cam = world.spawn_actor(cam_bp, tf, attach_to=vehicle)
            actors.append(cam)
            rig_cams[name] = cam

        # Spawn a chase camera to "see the car + rig"
        chase_tf = carla.Transform(
            carla.Location(x=-8.0, z=4.0),
            carla.Rotation(pitch=-15, yaw=0, roll=0),
        )
        chase = world.spawn_actor(cam_bp, chase_tf, attach_to=vehicle)
        actors.append(chase)

        # Draw debug markers where each rig camera is in world space
        # (These will show up in the chase camera render.)
        for name, cam in rig_cams.items():
            wt = cam.get_transform()
            world.debug.draw_point(
                wt.location,
                size=0.15,
                color=carla.Color(255, 0, 255),
                life_time=10.0
            )
            world.debug.draw_string(
                wt.location + carla.Location(z=0.3),
                name,
                draw_shadow=False,
                color=carla.Color(255, 255, 255),
                life_time=10.0
            )

        saved = {}

        def make_callback(name):
            def _cb(image):
                if name in saved:
                    return
                path = os.path.join(OUT_DIR, f"{name}_{image.frame}.png")
                save_rgb(image, path)
                saved[name] = {"frame": image.frame, "path": path}
            return _cb

        # Listen for one frame per camera
        for name, cam in rig_cams.items():
            cam.listen(make_callback(name))
        chase.listen(make_callback("chase"))

        # Tick until we got all images
        needed = set(list(rig_cams.keys()) + ["chase"])
        t0 = time.time()
        while needed - set(saved.keys()):
            world.tick()
            if time.time() - t0 > 10:
                raise RuntimeError(f"Timeout waiting for: {needed - set(saved.keys())}")

        # Stop sensors
        for cam in list(rig_cams.values()) + [chase]:
            cam.stop()

        # Save rig metadata
        meta = {
            "run_id": RUN_ID,
            "map": world.get_map().name,
            "vehicle_spawn": {
                "location": {"x": spawn.location.x, "y": spawn.location.y, "z": spawn.location.z},
                "rotation": {"pitch": spawn.rotation.pitch, "yaw": spawn.rotation.yaw, "roll": spawn.rotation.roll},
            },
            "rig": {
                name: {
                    "location": {"x": tf.location.x, "y": tf.location.y, "z": tf.location.z},
                    "rotation": {"pitch": tf.rotation.pitch, "yaw": tf.rotation.yaw, "roll": tf.rotation.roll},
                }
                for name, tf in rig.items()
            },
            "outputs": saved,
        }
        with open(os.path.join(OUT_DIR, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)

        print("Saved outputs to:", OUT_DIR)
        for k, v in saved.items():
            print(f"  {k}: {v['path']}")

    finally:
        # Restore settings
        try:
            world.apply_settings(original_settings)
        except Exception:
            pass
        # Cleanup actors
        for a in actors[::-1]:
            try:
                a.destroy()
            except Exception:
                pass

if __name__ == "__main__":
    main()