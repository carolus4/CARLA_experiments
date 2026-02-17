"""
Single source of truth for A→B route planning: spawn 0 to spawn END_SPAWN_INDEX.

Used by run_pcla_vision.py, 2Dplot_route.py, and closed_loop_drive.py so map
choice, start/end waypoints (snapped to driving lane), and GlobalRoutePlanner
trace are consistent and easier to debug.
"""
import os
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.dirname(_script_dir)
_vendored = os.path.join(_repo_root, "third_party", "carla_pythonapi")
if os.path.isdir(_vendored) and _vendored not in sys.path:
    sys.path.insert(0, _vendored)

import carla

# Shared constants
PREFERRED_MAPS = ["Town10", "Town10HD", "Town10_Opt", "Town01", "Town02"]
END_SPAWN_INDEX = 10
DEFAULT_SAMPLING_RESOLUTION = 2.0


def short_map_name(map_path):
    """Return short map name, e.g. /Game/Carla/Maps/Town01 -> Town01."""
    return map_path.split("/")[-1] if "/" in map_path else map_path


def select_map_name(available_maps, preferred=None):
    """Return first preferred map present in available_maps, else first available, else 'Town01'.

    available_maps can be full paths; short_map_name is used for comparison.
    """
    preferred = preferred or PREFERRED_MAPS
    short_available = [short_map_name(m) for m in available_maps]
    for name in preferred:
        if name in short_available:
            return name
    return short_available[0] if short_available else "Town01"


def get_start_end_waypoints(carla_map, end_spawn_index=None):
    """Pick start = spawns[0], end = spawns[end_spawn_index] (or last if fewer).

    Snaps both to driving lane so the route planner uses the correct graph node.
    Returns (start_wp, end_wp, end_idx).
    """
    if end_spawn_index is None:
        end_spawn_index = END_SPAWN_INDEX
    spawns = carla_map.get_spawn_points()
    if not spawns:
        raise RuntimeError("No spawn points on this map.")
    end_idx = min(end_spawn_index, len(spawns) - 1)
    start_wp = carla_map.get_waypoint(
        spawns[0].location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    end_wp = carla_map.get_waypoint(
        spawns[end_idx].location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    return start_wp, end_wp, end_idx


def trace_route(carla_map, start_wp, end_wp, sampling_resolution=None):
    """Compute shortest route with GlobalRoutePlanner.

    Returns list of (Waypoint, RoadOption). Raises on import or planner failure.
    """
    if sampling_resolution is None:
        sampling_resolution = DEFAULT_SAMPLING_RESOLUTION
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    grp = GlobalRoutePlanner(carla_map, sampling_resolution)
    return grp.trace_route(
        start_wp.transform.location,
        end_wp.transform.location,
    )


def waypoints_from_trace(route_trace):
    """Return list of Waypoint from route_trace for PCLA route_maker."""
    return [wp for wp, _ in route_trace]


def route_trace_to_plot_coords(route_trace):
    """Return (xs, ys) with Y negated for matplotlib plot coordinates.

    Plot uses (x, -y) so CARLA's left-handed Y-right becomes Y-up. This flips
    the scene across the X axis: a right turn in world (e.g. yaw 0° → -90°)
    appears as a left turn on the 2D plot, and vice versa.
    """
    xs, ys = [], []
    for wp, _ in route_trace:
        loc = wp.transform.location
        xs.append(loc.x)
        ys.append(-loc.y)
    return xs, ys


if __name__ == "__main__":
    # Minimal CLI: connect, load preferred map, get start/end and trace, print waypoint count.
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(60.0)  # load_world (e.g. Town10HD) can take longer than 10s
    available = client.get_available_maps()
    map_name = select_map_name(available)
    print("Loading map:", map_name, "...")
    world = client.load_world(map_name)
    print("Loaded map:", map_name)
    carla_map = world.get_map()
    start_wp, end_wp, end_idx = get_start_end_waypoints(carla_map)
    print("Start:", start_wp.transform.location, "End index:", end_idx, "End:", end_wp.transform.location)
    route_trace = trace_route(carla_map, start_wp, end_wp)
    print("Route waypoints:", len(route_trace))
