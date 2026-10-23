"""Single-knob camera model: flight altitude -> every imaging number downstream.

Ported from drone-dev's `mission_params.py`, where the ZR30 optics and the
motion-blur speed cap were worked out and flight-validated in PX4 SITL.

    cross-track swath   W = 2*h*tan(HFOV/2)
    along-track length  L = 2*h*tan(VFOV/2)
    GSD (m/px)          = W / image_width
    line spacing        = W * (1 - side_overlap)        <- the scanner's lane pitch
    speed (fwd-overlap) = L * (1 - fwd_overlap) * fps   <- rarely the limit at 30 fps
    speed (motion-blur) = GSD / exposure_time           <- the real speed cap
    cruise              = min(caps) * derate

Altitude is the only knob. Nothing downstream picks an imaging number twice.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class Camera:
    """SIYI ZR30 at 1x zoom, 720p video — the defaults are drone-dev's validated numbers."""

    hfov_deg: float = 58.1     # horizontal field of view (from the SIYI spec)
    vfov_deg: float = 35.0     # vertical FOV (derived from HFOV + the 65.4 diagonal)
    img_w: int = 1280          # 720p width (4K downsampled 3x)
    img_h: int = 720           # 720p height
    fps: int = 30


ZR30 = Camera()


def camera_with_hfov(hfov_deg: float, base: Camera = ZR30) -> Camera:
    """Same sensor, a different horizontal FOV (e.g. a zoom step, or another camera).

    VFOV follows the image aspect ratio. The ZR30's own HFOV short-circuits so its
    spec-sheet VFOV of 35 deg is used verbatim rather than the 34.7 deg the aspect
    ratio implies — the measured pair is what was flight-validated.
    """
    if math.isclose(hfov_deg, base.hfov_deg, rel_tol=1e-9):
        return base
    half_width = math.tan(math.radians(hfov_deg) / 2)
    vfov_deg = 2 * math.degrees(math.atan(half_width * base.img_h / base.img_w))
    return replace(base, hfov_deg=hfov_deg, vfov_deg=vfov_deg)


@dataclass(frozen=True)
class MissionProfile:
    altitude_m: float
    swath_w_m: float            # cross-track ground width the camera sees
    along_track_m: float        # along-track footprint length
    gsd_cm_px: float            # ground sampling distance
    line_spacing_m: float       # <- the scanner's lane pitch
    v_fwd_overlap_ms: float     # speed cap from forward overlap + fps
    v_motion_blur_ms: float     # speed cap from motion blur (exposure)
    v_recommended_ms: float     # the binding one (min), de-rated
    target_px: float            # pixels across the target's smallest dimension

    def report(self) -> str:
        return "\n".join(
            [
                f"  altitude               : {self.altitude_m:.0f} m",
                f"  swath width (W)        : {self.swath_w_m:.1f} m",
                f"  along-track footprint  : {self.along_track_m:.1f} m",
                f"  GSD                    : {self.gsd_cm_px:.2f} cm/px",
                f"  LINE SPACING           : {self.line_spacing_m:.1f} m   <- lane pitch",
                f"  speed cap (fwd-overlap): {self.v_fwd_overlap_ms:.0f} m/s  (fps-bound; rarely limits)",
                f"  speed cap (motion-blur): {self.v_motion_blur_ms:.1f} m/s  (the real limit)",
                f"  RECOMMENDED SPEED      : {self.v_recommended_ms:.1f} m/s",
                f"  target pixels          : {self.target_px:.0f} px across smallest dimension",
            ]
        )


def compute(
    altitude_m: float,
    cam: Camera = ZR30,
    side_overlap: float = 0.25,
    fwd_overlap: float = 0.25,
    exposure_s: float = 1.0 / 1000,   # daylight; the blur cap scales with this
    target_size_m: float = 0.5,       # a person's ~0.5 m smallest dimension
    speed_derate: float = 0.5,        # fly at half the binding cap for margin
) -> MissionProfile:
    """Everything downstream of a chosen altitude. Angles -> radians internally."""
    if altitude_m <= 0:
        raise ValueError("altitude_m must be positive")

    h = altitude_m
    swath_w = 2 * h * math.tan(math.radians(cam.hfov_deg) / 2)
    along = 2 * h * math.tan(math.radians(cam.vfov_deg) / 2)
    gsd_m = swath_w / cam.img_w
    line_spacing = swath_w * (1 - side_overlap)

    v_fwd = along * (1 - fwd_overlap) * cam.fps    # forward-overlap bound
    v_blur = gsd_m / exposure_s                    # 1-pixel smear bound

    return MissionProfile(
        altitude_m=h,
        swath_w_m=swath_w,
        along_track_m=along,
        gsd_cm_px=gsd_m * 100,
        line_spacing_m=line_spacing,
        v_fwd_overlap_ms=v_fwd,
        v_motion_blur_ms=v_blur,
        v_recommended_ms=min(v_fwd, v_blur) * speed_derate,
        target_px=target_size_m / gsd_m,
    )


def ceiling_for_gsd(target_gsd_cm: float, cam: Camera = ZR30) -> float:
    """Max altitude (m) that still achieves a required GSD (cm/px).

    Inverts gsd = 2*h*tan(hfov/2)/img_w, so an operator cannot unknowingly plan a
    scan flown too high to resolve a person.
    """
    if target_gsd_cm <= 0:
        raise ValueError("target_gsd_cm must be positive")
    return (target_gsd_cm / 100) * cam.img_w / (2 * math.tan(math.radians(cam.hfov_deg) / 2))


def area_estimate(profile: MissionProfile, width_m: float, height_m: float) -> dict:
    """Rough scan time for a rectangular AO (ignores turns; add ~25% for those)."""
    n_lines = math.ceil(width_m / profile.line_spacing_m)
    total_path_m = n_lines * height_m
    return {
        "n_lines": n_lines,
        "total_path_km": total_path_m / 1000,
        "scan_time_min": total_path_m / profile.v_recommended_ms / 60,
    }
