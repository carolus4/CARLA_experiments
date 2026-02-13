import carla


def inspect_world(world: carla.World) -> None:
    settings = world.get_settings()
    weather = world.get_weather()
    actors = world.get_actors()

    vehicles = actors.filter("vehicle.*")
    walkers = actors.filter("walker.*")
    sensors = actors.filter("sensor.*")
    traffic_lights = actors.filter("traffic.traffic_light")

    print("=== WORLD INSPECT ===")
    print(f"Map: {world.get_map().name}")
    print(f"Synchronous mode: {settings.synchronous_mode}")
    print(f"Fixed delta seconds: {settings.fixed_delta_seconds}")
    print()

    print("Weather:")
    print(f"  Cloudiness: {weather.cloudiness}")
    print(f"  Precipitation: {weather.precipitation}")
    print(f"  Sun altitude angle: {weather.sun_altitude_angle}")
    print()

    print("Actors:")
    print(f"  Vehicles: {len(vehicles)}")
    print(f"  Walkers: {len(walkers)}")
    print(f"  Sensors: {len(sensors)}")
    print(f"  Traffic lights: {len(traffic_lights)}")
    print()

    spawn_points = world.get_map().get_spawn_points()
    print(f"Spawn points available: {len(spawn_points)}")
    print()


if __name__ == "__main__":
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    inspect_world(world)