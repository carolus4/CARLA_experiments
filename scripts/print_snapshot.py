"""
Print the current world snapshot (frame, timestamp, and all actor snapshots).
Run with CARLA connected: python scripts/print_snapshot.py
"""
import carla


def main():
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    snapshot = world.get_snapshot()

    # Frame and timestamp (Python API may use .frame / .timestamp or get_frame() / get_timestamp())
    frame = getattr(snapshot, "frame", None) or getattr(snapshot, "get_frame", lambda: None)()
    ts = getattr(snapshot, "timestamp", None)
    if ts is None and hasattr(snapshot, "get_timestamp"):
        ts = snapshot.get_timestamp()
    episode_id = getattr(snapshot, "id", None) or getattr(snapshot, "get_id", lambda: None)()

    print("=== WORLD SNAPSHOT ===")
    print(f"Frame: {frame}")
    print(f"Episode ID: {episode_id}")
    print(f"Timestamp: {ts}")
    print()

    # Iterate over actor snapshots (Python bindings often make WorldSnapshot iterable)
    try:
        count = 0
        for actor_snapshot in snapshot:
            count += 1
            aid = getattr(actor_snapshot, "id", "?")
            t = actor_snapshot.get_transform()
            v = actor_snapshot.get_velocity()
            loc = t.location
            rot = t.rotation
            print(
                f"  actor id={aid}  "
                f"location=({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})  "
                f"rotation(p,y,r)=({rot.pitch:.1f}, {rot.yaw:.1f}, {rot.roll:.1f})  "
                f"velocity=({v.x:.1f}, {v.y:.1f}, {v.z:.1f})"
            )
        print(f"\nTotal actors: {count}")
    except TypeError:
        # Snapshot might not be iterable; try alternative API
        print("(Snapshot iteration not available; check CARLA Python API for this version)")
        if hasattr(snapshot, "size"):
            print(f"Snapshot size: {snapshot.size()}")


if __name__ == "__main__":
    main()
