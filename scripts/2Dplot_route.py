"""
2D route map: Google Maps-style schematic with start (A) and end (B).
Step-by-step: connect → densify roads → choose start/end → draw → save.
Optional: aerial top-down render as background with aligned overlay.
"""
import math
import time

import carla
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

HOST, PORT = "127.0.0.1", 2000
USE_AERIAL = True  # Set False to skip aerial capture and use schematic-only

DENSE_STEP = 2.0  # meters between waypoints when walking lane splines
ARROW_SUBSAMPLE = 16  # show a direction arrow every N-th dense waypoint pair (higher = fewer arrows)
MAX_SPAWN_DISTANCE = 800  # meters – pick the farthest spawn from start


# --- Step 1: Connect to CARLA, load preferred map, get carla_map -----------

def _short_map_name(path: str) -> str:
    """e.g. /Game/Carla/Maps/Town01 -> Town01"""
    return path.split("/")[-1] if "/" in path else path


def connect_and_load_map():
    client = carla.Client(HOST, PORT)
    client.set_timeout(10.0)
    available = [_short_map_name(m) for m in client.get_available_maps()]
    preferred = ["Town10", "Town10HD", "Town10_Opt", "Town01", "Town02"]
    map_name = next(
        (m for m in preferred if m in available),
        available[0] if available else "Town01",
    )
    world = client.load_world(map_name)
    carla_map = world.get_map()
    return client, world, carla_map


# --- Step 2: Build road geometry by walking lane splines -----------------

def _walk_lane(wp_start, wp_end, step=DENSE_STEP):
    """Walk from wp_start toward wp_end in `step`-metre increments.

    Returns list of (x, -y) tuples.  Y is negated so that the CARLA
    left-handed frame (Y-right) maps to matplotlib's Y-up convention.
    """
    target = wp_end.transform.location
    max_dist = wp_start.transform.location.distance(target)
    # Clamp walk length so we don't wander past the segment end
    max_steps = max(int(max_dist / step) + 1, 2)

    pts = []
    wp = wp_start
    for _ in range(max_steps):
        loc = wp.transform.location
        pts.append((loc.x, -loc.y))
        nexts = wp.next(step)
        if not nexts:
            break
        wp = nexts[0]
    # Always include the exact endpoint
    pts.append((target.x, -target.y))
    return pts


def build_road_geometry(carla_map):
    """Return (xs, ys) with None-separated polylines that follow lane splines."""
    topology = carla_map.get_topology()
    xs, ys = [], []
    for w0, w1 in topology:
        pts = _walk_lane(w0, w1)
        for x, y in pts:
            xs.append(x)
            ys.append(y)
        xs.append(None)
        ys.append(None)
    return xs, ys


def get_arrow_geometry(carla_map, subsample=ARROW_SUBSAMPLE):
    """Return (mid_xs, mid_ys, us, vs) for lane-direction arrows.

    Arrows are placed along the densified lane polylines, subsampled to
    keep the plot readable.
    """
    topology = carla_map.get_topology()
    mid_xs, mid_ys, us, vs = [], [], [], []
    counter = 0
    for w0, w1 in topology:
        pts = _walk_lane(w0, w1)
        for i in range(len(pts) - 1):
            counter += 1
            if counter % subsample != 0:
                continue
            x0, y0 = pts[i]
            x1, y1 = pts[i + 1]
            dx, dy = x1 - x0, y1 - y0
            length = math.hypot(dx, dy)
            if length < 0.01:
                continue
            mid_xs.append((x0 + x1) / 2)
            mid_ys.append((y0 + y1) / 2)
            us.append(dx / length)
            vs.append(dy / length)
    return mid_xs, mid_ys, us, vs


def get_map_bounds(carla_map, margin_meters=20):
    """Compute (x_min, x_max, y_min, y_max) from topology (already Y-negated)."""
    topology = carla_map.get_topology()
    xs, ys = [], []
    for w0, w1 in topology:
        for w in (w0, w1):
            xs.append(w.transform.location.x)
            ys.append(-w.transform.location.y)
    x_min, x_max = min(xs) - margin_meters, max(xs) + margin_meters
    y_min, y_max = min(ys) - margin_meters, max(ys) + margin_meters
    return x_min, x_max, y_min, y_max


# --- Aerial capture (optional) -------------------------------------------

def capture_aerial(world, x_min, x_max, y_min, y_max, image_size=1920):
    """Capture one top-down RGB frame aligned to the plot coordinate space.

    Returns (rgb_array, extent) or (None, None) on failure.
    extent = (x_lo, x_hi, y_lo, y_hi) in plot coords (Y-negated).
    """
    # Camera aims at the centre of the *world-space* bounds.
    # Note: y_min/y_max are already negated, so negate back for CARLA.
    cx = (x_min + x_max) / 2
    cy_world = -(y_min + y_max) / 2  # back to CARLA Y
    side = max(x_max - x_min, y_max - y_min)
    half_side = side / 2

    fov_deg = 90
    height = half_side / math.tan(math.radians(fov_deg / 2))

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.1
    world.apply_settings(settings)

    camera = None
    captured = {}

    def on_image(image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))
        captured["rgb"] = arr[:, :, :3][:, :, ::-1].copy()  # BGRA -> RGB
        camera.stop()

    try:
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(image_size))
        bp.set_attribute("image_size_y", str(image_size))
        bp.set_attribute("fov", str(fov_deg))
        # pitch=-90 looks straight down; yaw=0 means camera-right = world-Y
        transform = carla.Transform(
            carla.Location(x=cx, y=cy_world, z=height),
            carla.Rotation(pitch=-90, yaw=0, roll=0),
        )
        camera = world.spawn_actor(bp, transform)
        camera.listen(on_image)

        world.tick()
        timeout = time.time() + 5
        while "rgb" not in captured and time.time() < timeout:
            time.sleep(0.05)

        if "rgb" not in captured:
            return None, None

        # The camera looks down with pitch=-90, yaw=0.
        # Image row 0 = world -X direction, col 0 = world -Y direction.
        # We need: row 0 → plot Y-max (top), col 0 → plot X-min (left).
        # Plot Y = -world_Y, so col 0 (world -Y) = plot +Y → need flip rows.
        # Row 0 (world -X) should map to plot-top (Y-max), but X is horizontal
        # in our plot → we need a 90° rotation.
        rgb = captured["rgb"]
        # Rotate 90° counter-clockwise: maps (row→-X, col→-Y) to
        # (row→-Y = plot+Y top-to-bottom, col→-X → plot-X left-to-right after flip)
        rgb = np.rot90(rgb, k=1)
        # After rot90(k=1): row 0 = old last-col = world +Y = plot -Y.
        # We want row 0 = plot Y-max, so flip vertically.
        rgb = rgb[::-1]
        # col 0 = old row 0 = world -X = plot -X (left). ✓

        extent = (cx - half_side, cx + half_side,
                  -(cy_world + half_side), -(cy_world - half_side))
        return np.ascontiguousarray(rgb), extent

    except Exception:
        return None, None
    finally:
        if camera is not None and camera.is_alive:
            try:
                camera.destroy()
            except Exception:
                pass
        world.apply_settings(original_settings)


# --- Step 3: Choose start and end (spawns[0] and spawns[10]) ----------------

def choose_start_end(carla_map):
    """Pick start = spawns[0], end = spawns[10] (or last if fewer)."""
    spawns = carla_map.get_spawn_points()
    if len(spawns) < 2:
        wp = carla_map.get_waypoint(
            spawns[0].location, project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        return wp, wp

    start_tf = spawns[0]
    end_idx = min(10, len(spawns) - 1)
    end_tf = spawns[end_idx]

    start_wp = carla_map.get_waypoint(
        start_tf.location, project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    end_wp = carla_map.get_waypoint(
        end_tf.location, project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    return start_wp, end_wp


# --- Step 4: Draw the map ------------------------------------------------

def draw_map(ax, xs, ys, *, road_color="#2d2d2d", linewidth=1.0,
             title=None, arrow_data=None, use_aerial=False):
    ax.plot(xs, ys, color=road_color, linewidth=linewidth, zorder=1)

    if arrow_data:
        mid_xs, mid_ys, us, vs = arrow_data
        if mid_xs:
            # Arrows in yellow so they stand out on both roads and aerial
            arrow_color = "#F4C430" if use_aerial else road_color
            ax.quiver(
                mid_xs, mid_ys, us, vs,
                color=arrow_color,
                angles="xy",
                scale_units="xy",
                scale=1 / 12,
                width=0.004,
                headwidth=4,
                headlength=5,
                zorder=2,
            )

    ax.set_aspect("equal")
    ax.set_facecolor("none" if use_aerial else "#f5f5f5")
    if title:
        ax.set_title(title)
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)


# --- Step 5: Draw start/end markers --------------------------------------

def draw_start_end(ax, start_wp, end_wp, marker_size=120):
    sx = start_wp.transform.location.x
    sy = -start_wp.transform.location.y
    ex = end_wp.transform.location.x
    ey = -end_wp.transform.location.y

    ax.scatter(
        [sx], [sy], c="#4285F4", s=marker_size, marker="o",
        edgecolors="white", linewidths=1.5, zorder=4,
    )
    ax.annotate(
        "A", (sx, sy), fontsize=9, fontweight="bold", color="white",
        ha="center", va="center", zorder=5,
    )

    ax.scatter(
        [ex], [ey], c="#EA4335", s=marker_size, marker="o",
        edgecolors="white", linewidths=1.5, zorder=4,
    )
    ax.annotate(
        "B", (ex, ey), fontsize=9, fontweight="bold", color="white",
        ha="center", va="center", zorder=5,
    )


# --- Step 6: Save figure -------------------------------------------------

def save_figure(fig, carla_map, out_dir=None):
    if out_dir is None:
        out_dir = Path(__file__).resolve().parent.parent / "data" / "map_plots"
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    safe_name = carla_map.name.split("/")[-1]
    out_path = out_dir / f"route_plot_{safe_name}.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor="white")
    return out_path


# --- main -----------------------------------------------------------------

def main():
    # Step 1
    client, world, carla_map = connect_and_load_map()

    # Step 2
    xs, ys = build_road_geometry(carla_map)
    arrow_data = get_arrow_geometry(carla_map)
    x_min, x_max, y_min, y_max = get_map_bounds(carla_map)

    # Step 3
    start_wp, end_wp = choose_start_end(carla_map)

    # Aerial capture (optional)
    aerial_rgb, extent = None, None
    if USE_AERIAL:
        try:
            aerial_rgb, extent = capture_aerial(
                world, x_min, x_max, y_min, y_max, image_size=1920,
            )
        except Exception:
            aerial_rgb, extent = None, None

    # Step 4 & 5
    fig, ax = plt.subplots(1, 1, facecolor="white")
    map_title = f"CARLA {carla_map.name.split('/')[-1]} Lanes"

    if aerial_rgb is not None and extent is not None:
        ax.imshow(
            aerial_rgb,
            extent=[extent[0], extent[1], extent[2], extent[3]],
            origin="lower",
            aspect="equal",
            zorder=0,
        )
        ax.set_xlim(extent[0], extent[1])
        ax.set_ylim(extent[2], extent[3])
        draw_map(ax, xs, ys, title=map_title, arrow_data=arrow_data,
                 use_aerial=True)
    else:
        draw_map(ax, xs, ys, title=map_title, arrow_data=arrow_data,
                 use_aerial=False)

    draw_start_end(ax, start_wp, end_wp)

    # Step 6
    out_path = save_figure(fig, carla_map)
    print(f"Saved: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()