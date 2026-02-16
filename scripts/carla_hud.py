"""
Minimal CARLA-style HUD for throttle/brake/steer and telemetry.
Vendored from CARLA manual_control.py; no IMU/GNSS/collision sensors.
"""
import datetime
import math
import os

import pygame

# Optional: type hint only; carla may be used at runtime for VehicleControl
try:
    import carla
except ImportError:
    carla = None


def _get_mono_font():
    """Mono font for HUD text (same style as CARLA manual_control)."""
    font_name = "courier" if os.name == "nt" else "mono"
    fonts = [x for x in pygame.font.get_fonts() if font_name in x]
    default_font = "ubuntumono"
    mono = default_font if default_font in fonts else (fonts[0] if fonts else None)
    if mono:
        path = pygame.font.match_font(mono)
        if path:
            return pygame.font.Font(path, 12 if os.name == "nt" else 14)
    return pygame.font.Font(pygame.font.get_default_font(), 14)


class MinimalHUD:
    """
    Renders a left-side panel with speed, location, frame, sim time,
    and throttle/brake/steer bars (and related vehicle control state).
    """

    def __init__(self, width, height):
        self.dim = (width, height)
        self._font_mono = _get_mono_font()
        self._info_text = []

    def update(
        self,
        vehicle,
        control,
        frame,
        simulation_time,
        map_name=None,
        velocity=None,
        transform=None,
    ):
        """Build the list of lines and bar tuples from vehicle and control.

        If velocity or transform are provided (e.g. from world snapshot), they are
        used for speed and location; otherwise vehicle.get_velocity() and
        vehicle.get_transform() are used.
        """
        t = transform if transform is not None else vehicle.get_transform()
        v = velocity if velocity is not None else vehicle.get_velocity()
        speed_kmh = 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        map_str = (map_name or "unknown").split("/")[-1]

        self._info_text = [
            "Frame: %d" % frame,
            "Simulation time: %s"
            % datetime.timedelta(seconds=int(simulation_time)),
            "",
            "Vehicle: %s" % _actor_display_name(vehicle, 20),
            "Map: %s" % map_str,
            "",
            "Speed: % 15.0f km/h" % speed_kmh,
            "Location: (% 5.1f, % 5.1f)" % (t.location.x, t.location.y),
            "Height: % 18.0f m" % t.location.z,
            "",
        ]
        if control is not None and carla is not None and isinstance(
            control, carla.VehicleControl
        ):
            self._info_text += [
                ("Throttle:", control.throttle, 0.0, 1.0),
                ("Steer:", control.steer, -1.0, 1.0),
                ("Brake:", control.brake, 0.0, 1.0),
                ("Reverse:", control.reverse),
                ("Hand brake:", control.hand_brake),
                ("Manual:", control.manual_gear_shift),
                "Gear: %s" % {-1: "R", 0: "N"}.get(control.gear, control.gear),
            ]

    def render(self, display):
        """Draw the HUD onto a Pygame surface (left panel + bars/text)."""
        if not self._info_text:
            return
        # Semi-transparent left panel (220px wide, same as CARLA)
        info_surface = pygame.Surface((220, self.dim[1]))
        info_surface.set_alpha(100)
        display.blit(info_surface, (0, 0))

        v_offset = 4
        bar_h_offset = 100
        bar_width = 106
        white = (255, 255, 255)

        for item in self._info_text:
            if v_offset + 18 > self.dim[1]:
                break
            if isinstance(item, tuple):
                if isinstance(item[1], bool):
                    rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                    pygame.draw.rect(
                        display, white, rect, 0 if item[1] else 1
                    )
                else:
                    # (label, value, min_val, max_val) -> bar
                    rect_border = pygame.Rect(
                        (bar_h_offset, v_offset + 8), (bar_width, 6)
                    )
                    pygame.draw.rect(display, white, rect_border, 1)
                    lo, hi = item[2], item[3]
                    val = item[1]
                    denom = hi - lo
                    f = (val - lo) / denom if denom != 0 else 0
                    f = max(0, min(1, f))
                    if lo < 0:
                        # Steer: centered indicator
                        center_x = bar_h_offset + (f * (bar_width - 6))
                        rect = pygame.Rect(
                            (int(center_x), v_offset + 8), (6, 6)
                        )
                    else:
                        # Throttle/Brake: left-to-right fill
                        rect = pygame.Rect(
                            (bar_h_offset, v_offset + 8),
                            (int(f * bar_width), 6),
                        )
                    pygame.draw.rect(display, white, rect)
                item = item[0]
            if item:
                surface = self._font_mono.render(str(item), True, white)
                display.blit(surface, (8, v_offset))
            v_offset += 18


def _actor_display_name(actor, truncate=250):
    """Short display name for an actor (e.g. vehicle type)."""
    name = " ".join(
        actor.type_id.replace("_", ".").title().split(".")[1:]
    )
    if len(name) > truncate:
        name = name[: truncate - 1] + "\u2026"
    return name


def composite_hud_on_rgb(
    rgb,
    hud,
    vehicle,
    control,
    frame,
    simulation_time,
    map_name=None,
    velocity=None,
    transform=None,
):
    """
    Draw the HUD onto a numpy RGB image (height, width, 3) and return
    the composited numpy array (same shape). Uses Pygame offscreen.

    velocity and transform can be from world.get_snapshot() for correct
    per-frame values when the callback runs during the tick.
    """
    height, width = rgb.shape[0], rgb.shape[1]
    # Pygame surfarray uses (width, height, 3)
    rgb_t = rgb.transpose((1, 0, 2)).copy()
    surf = pygame.Surface((width, height))
    pygame.surfarray.blit_array(surf, rgb_t)
    hud.update(
        vehicle,
        control,
        frame,
        simulation_time,
        map_name=map_name,
        velocity=velocity,
        transform=transform,
    )
    hud.render(surf)
    out = pygame.surfarray.array3d(surf)
    # Back to (height, width, 3)
    return out.transpose((1, 0, 2)).copy()


def init_pygame_for_headless():
    """
    Call before pygame.init() when running without a display (e.g. headless).
    Sets SDL_VIDEODRIVER=dummy so no window is created.
    """
    if not os.environ.get("DISPLAY") and "SDL_VIDEODRIVER" not in os.environ:
        os.environ["SDL_VIDEODRIVER"] = "dummy"


def init_pygame_for_hud():
    """
    Call before pygame.init() when using Pygame only for HUD drawing.
    Disables SDL audio to avoid ALSA errors on headless/no-sound systems,
    and hides the Pygame community message. Safe to call even with a display.
    """
    if "SDL_AUDIODRIVER" not in os.environ:
        os.environ["SDL_AUDIODRIVER"] = "dummy"
    if "PYGAME_HIDE_SUPPORT_PROMPT" not in os.environ:
        os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
