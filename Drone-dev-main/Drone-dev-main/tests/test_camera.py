"""The single-knob camera model, and the config properties that read from it."""

import math
from dataclasses import replace
from pathlib import Path

import pytest

from flight_planner.camera import ZR30, camera_with_hfov, ceiling_for_gsd, compute
from flight_planner.config import load_config
from flight_planner.models import PlannerConfig

ROOT = Path(__file__).parents[1]


def test_zr30_profile_at_30m_matches_the_flight_validated_numbers():
    """drone-dev's validated figures for the ZR30 at 30 m, 720p, 25% overlap."""
    profile = compute(30.0)
    assert profile.swath_w_m == pytest.approx(33.3, abs=0.1)
    assert profile.gsd_cm_px == pytest.approx(2.60, abs=0.01)
    assert profile.line_spacing_m == pytest.approx(25.0, abs=0.1)
    assert profile.v_recommended_ms == pytest.approx(13.0, abs=0.1)


def test_motion_blur_is_the_binding_speed_cap():
    profile = compute(30.0)
    assert profile.v_motion_blur_ms < profile.v_fwd_overlap_ms
    assert profile.v_recommended_ms == pytest.approx(profile.v_motion_blur_ms * 0.5)


def test_every_number_scales_off_altitude_alone():
    low, high = compute(30.0), compute(60.0)
    assert high.swath_w_m == pytest.approx(low.swath_w_m * 2)
    assert high.gsd_cm_px == pytest.approx(low.gsd_cm_px * 2)
    assert high.line_spacing_m == pytest.approx(low.line_spacing_m * 2)
    assert high.v_recommended_ms == pytest.approx(low.v_recommended_ms * 2)
    assert high.target_px == pytest.approx(low.target_px / 2)


def test_ceiling_for_gsd_inverts_the_gsd_calculation():
    altitude = ceiling_for_gsd(3.0)
    assert compute(altitude).gsd_cm_px == pytest.approx(3.0)


def test_ceiling_rejects_a_nonsensical_gsd():
    with pytest.raises(ValueError):
        ceiling_for_gsd(0.0)


def test_compute_rejects_a_nonsensical_altitude():
    with pytest.raises(ValueError):
        compute(0.0)


def test_zr30_hfov_keeps_its_measured_vfov():
    assert camera_with_hfov(ZR30.hfov_deg) is ZR30


def test_a_different_hfov_rescales_vfov_by_the_image_aspect_ratio():
    camera = camera_with_hfov(90.0)
    expected = 2 * math.degrees(math.atan(math.tan(math.radians(45.0)) * ZR30.img_h / ZR30.img_w))
    assert camera.vfov_deg == pytest.approx(expected)
    assert camera.hfov_deg == 90.0


def test_config_spacing_and_speed_both_read_from_one_profile():
    config = load_config(ROOT / "config.example.json")
    assert config.lane_spacing_m == pytest.approx(config.profile.line_spacing_m)
    assert config.cruise_speed_m_s == pytest.approx(round(config.profile.v_recommended_ms, 1))
    assert config.lane_spacing_m == pytest.approx(25.0, abs=0.1)
    assert config.cruise_speed_m_s == pytest.approx(13.0, abs=0.1)


def test_altitude_moves_spacing_and_speed_together():
    low = PlannerConfig(scan_altitude_m=30.0)
    high = replace(low, scan_altitude_m=60.0)
    assert high.lane_spacing_m == pytest.approx(low.lane_spacing_m * 2)
    assert high.cruise_speed_m_s == pytest.approx(low.cruise_speed_m_s * 2, abs=0.1)


def test_lane_spacing_never_collapses_to_zero():
    assert PlannerConfig(scan_overlap_percent=99.9).lane_spacing_m >= 0.1
