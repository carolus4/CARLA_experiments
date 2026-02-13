"""
Set CARLA weather to the default scene (cloudiness 20, precipitation 0, wetness 0, sun 45°).
Run with the simulator connected: python scripts/set_scene.py
"""
import carla

# Default scene weather (reset / baseline)
DEFAULT_CLOUDINESS = 20.0
DEFAULT_PRECIPITATION = 0.0
DEFAULT_WETNESS = 0.0
DEFAULT_SUN_ALTITUDE_ANGLE = 45.0


def main():
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    weather = world.get_weather()
    weather.cloudiness = DEFAULT_CLOUDINESS
    weather.precipitation = DEFAULT_PRECIPITATION
    weather.wetness = DEFAULT_WETNESS
    weather.sun_altitude_angle = DEFAULT_SUN_ALTITUDE_ANGLE
    world.set_weather(weather)

    print(
        f"Scene set: cloudiness={DEFAULT_CLOUDINESS}, precipitation={DEFAULT_PRECIPITATION}, "
        f"wetness={DEFAULT_WETNESS}, sun_altitude_angle={DEFAULT_SUN_ALTITUDE_ANGLE}"
    )


if __name__ == "__main__":
    main()
