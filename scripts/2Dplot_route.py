"""
2D route map: Google Maps-style schematic with start (A) and end (B).
Step-by-step: connect → densify roads → choose start/end → draw → save.
Optional: aerial top-down render as background with aligned overlay.
Optional: GlobalRoutePlanner route A→B overlaid (when vendored agents available).
"""
import math
import os
import sys
import time

import carla
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _script_dir)
import route_plan

import carla

HOST, PORT = "127.0.0.1", 2000
USE_AERIAL = True  # Set False to skip aerial capture and use schematic-only
DRAW_START_END = True   # Set False for map-only (no A/B markers)
DRAW_GRP_ROUTE = True   # Set False for map-only (no green route line)

DENSE_STEP = 2.0  # meters between waypoints when walking lane splines
ARROW_SUBSAMPLE = 24  # show a direction arrow every N-th dense waypoint pair
# Arrow size (matplotlib quiver): smaller scale = longer arrows; width/head* = thickness and head size
ARROW_SCALE = 8         # quiver scale (e.g. 4 = bigger, 10 = smaller)
ARROW_SHAFT_WIDTH = 0.005
ARROW_HEADWIDTH = 3
ARROW_HEADLENGTH = 4


# --- Step 1: Connect to CARLA, load preferred map, get carla_map -----------

def connect_and_load_map():
    client = carla.Client(HOST, PORT)
    client.set_timeout(60.0)  # load_world (e.g. Town10HD) can take longer than 10s
    map_name = route_plan.select_map_name(client.get_available_maps())
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

    Camera orientation (pitch=-90, yaw=-90) is chosen so the raw image
    maps directly to our plot axes with no post-rotation needed:
      - increasing column  →  +X world  (plot X, left-to-right)
      - increasing row     →  +Y world  (= -plot_Y, top-to-bottom)
    Used with imshow(origin="upper") so row 0 is at the top of the figure.
    """
    # x_min..x_max and y_min..y_max are in *plot* coords (Y already negated).
    # Convert centre back to CARLA world-Y for the camera spawn.
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

        # pitch=-90 → look straight down (-Z).
        # yaw=-90  → camera-right = +X world, camera-up = -Y world = +plot_Y.
        # This means image columns = world X, image rows = world Y, which
        # matches our plot axes directly (no rotation/flip needed).
        transform = carla.Transform(
            carla.Location(x=cx, y=cy_world, z=height),
            carla.Rotation(pitch=-90, yaw=-90, roll=0),
        )
        camera = world.spawn_actor(bp, transform)
        camera.listen(on_image)

        world.tick()
        timeout = time.time() + 5
        while "rgb" not in captured and time.time() < timeout:
            time.sleep(0.05)

        if "rgb" not in captured:
            return None, None

        rgb = captured["rgb"]
        # No rotation needed — axes already aligned.

        # extent for imshow: [x_left, x_right, y_bottom, y_top]
        # Column 0 = world (cx - half_side) = plot x_min
        # Column max = world (cx + half_side) = plot x_max
        # Row 0 (top, origin="upper") = world (cy_world - half_side)
        #   → plot_y = -(cy_world - half_side) = -cy_world + half_side
        # Row max (bottom) = world (cy_world + half_side)
        #   → plot_y = -(cy_world + half_side) = -cy_world - half_side
        y_plot_top = -cy_world + half_side
        y_plot_bottom = -cy_world - half_side
        extent = (cx - half_side, cx + half_side,
                  y_plot_bottom, y_plot_top)

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


def draw_grp_route(ax, route_xs, route_ys, color="#34A853",
                   linewidth=2.5, zorder=3):
    """Draw the GlobalRoutePlanner route as a single polyline on the map."""
    if not route_xs or not route_ys:
        return
    ax.plot(route_xs, route_ys, color=color, linewidth=linewidth, zorder=zorder)


# --- Step 4: Draw the map ------------------------------------------------

def draw_map(ax, xs, ys, *, road_color="#2d2d2d", linewidth=1.0,
             title=None, arrow_data=None, use_aerial=False):
    if use_aerial:
        ax.plot(xs, ys, color="white", linewidth=linewidth + 1.0, zorder=0.5)
    ax.plot(xs, ys, color=road_color, linewidth=linewidth, zorder=1)

    if arrow_data:
        mid_xs, mid_ys, us, vs = arrow_data
        if mid_xs:
            arrow_color = "#F4C430" if use_aerial else road_color
            ax.quiver(
                mid_xs, mid_ys, us, vs,
                color=arrow_color,
                angles="xy",
                scale_units="inches",
                scale=ARROW_SCALE,
                width=ARROW_SHAFT_WIDTH,
                headwidth=ARROW_HEADWIDTH,
                headlength=ARROW_HEADLENGTH,
                zorder=2,
            )

    ax.set_aspect("equal")
    ax.set_facecolor("none" if use_aerial else "#f5f5f5")
    ax.set_axis_off()
    if title:
        ax.set_title(title)


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

def save_figure(fig, carla_map, out_dir=None, with_grp_route=False):
    if out_dir is None:
        out_dir = Path(__file__).resolve().parent.parent / "data" / "map_plots"
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    safe_name = carla_map.name.split("/")[-1]
    suffix = "_grp" if with_grp_route else ""
    out_path = out_dir / f"route_plot_{safe_name}{suffix}.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight", pad_inches=0,
                facecolor="white")
    return out_path


# --- main -----------------------------------------------------------------

def main():
    # Step 1
    client, world, carla_map = connect_and_load_map()

    # Step 2
    xs, ys = build_road_geometry(carla_map)
    arrow_data = get_arrow_geometry(carla_map)
    x_min, x_max, y_min, y_max = get_map_bounds(carla_map)

    # Step 3: start/end waypoints and optional GRP route (from route_plan)
    start_wp, end_wp, end_idx = route_plan.get_start_end_waypoints(carla_map)
    spawns = carla_map.get_spawn_points()
    print(f"  spawn[0]  raw: {spawns[0].location}")
    print(f"  spawn[{end_idx}] raw: {spawns[end_idx].location}")
    print(f"  start_wp:  {start_wp.transform.location}")
    print(f"  end_wp:    {end_wp.transform.location}")

    route_xs, route_ys = None, None
    if DRAW_GRP_ROUTE:
        try:
            route_trace = route_plan.trace_route(carla_map, start_wp, end_wp)
            if route_trace:
                route_xs, route_ys = route_plan.route_trace_to_plot_coords(route_trace)
                for idx, (wp, road_opt) in enumerate(route_trace):
                    if idx <= 31:
                        loc = wp.transform.location
                        print(f"    [{idx:3d}] ({loc.x:8.2f}, {loc.y:8.2f})  {road_opt}")
                    elif idx == 32:
                        print(f"    ... ({len(route_trace) - 32} more)")
                        break
                print(f"  GRP route has {len(route_trace)} waypoints")
        except Exception:
            route_xs, route_ys = None, None

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

    if aerial_rgb is not None and extent is not None:
        ax.imshow(
            aerial_rgb,
            extent=[extent[0], extent[1], extent[2], extent[3]],
            origin="upper",
            aspect="equal",
            zorder=0,
        )
        ax.set_xlim(extent[0], extent[1])
        ax.set_ylim(extent[2], extent[3])
        draw_map(ax, xs, ys, arrow_data=arrow_data, use_aerial=True)
    else:
        draw_map(ax, xs, ys, arrow_data=arrow_data, use_aerial=False)

    if DRAW_START_END:
        draw_start_end(ax, start_wp, end_wp)

    if DRAW_GRP_ROUTE and route_xs is not None and route_ys is not None:
        draw_grp_route(ax, route_xs, route_ys)

    # Step 6
    out_path = save_figure(
        fig, carla_map,
        with_grp_route=(DRAW_GRP_ROUTE and route_xs is not None),
    )
    print(f"Saved: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
