"""parameters.yaml is a reference, so it must not drift from the code it documents.

Every value recorded there is checked against the live source here. If you change a
default, this test tells you the reference needs updating too.
"""

import inspect
from pathlib import Path

import pytest

yaml = pytest.importorskip("yaml")

from flight_planner import qgc_plan
from flight_planner.camera import Camera, compute
from flight_planner.models import PlannerConfig

ROOT = Path(__file__).parents[1]
DOC = yaml.safe_load((ROOT / "parameters.yaml").read_text())

METADATA_KEYS = {"status", "source", "note", "critical"}


def test_camera_sensor_block_matches_the_camera_dataclass():
    camera = Camera()
    for field in ("hfov_deg", "vfov_deg", "img_w", "img_h", "fps"):
        assert DOC["camera"][field]["value"] == getattr(camera, field), field


def test_imaging_block_matches_compute_defaults():
    defaults = inspect.signature(compute).parameters
    for field in ("side_overlap", "fwd_overlap", "exposure_s", "target_size_m", "speed_derate"):
        assert DOC["imaging"][field]["value"] == defaults[field].default, field


def test_mission_block_matches_planner_config_defaults():
    config = PlannerConfig()
    for field in ("scan_altitude_m", "camera_fov_degrees", "scan_overlap_percent",
                  "number_of_drones", "scan_type", "layer_spacing_m", "turn_radius_m",
                  "turn_style", "pre_yaw_lead_s", "ingress_style", "ingress_objective",
                  "transit_layer_spacing_m", "assignment", "ingress_min_gate_m"):
        assert DOC["mission"][field]["value"] == getattr(config, field), field


def test_derived_block_matches_the_computed_profile():
    profile = PlannerConfig().profile
    documented = DOC["derived"]["at_30m_defaults"]
    for field in ("swath_w_m", "along_track_m", "gsd_cm_px", "line_spacing_m",
                  "v_fwd_overlap_ms", "v_motion_blur_ms", "v_recommended_ms"):
        assert documented[field]["value"] == pytest.approx(getattr(profile, field), abs=0.05), field


def test_plan_format_block_matches_the_writer_constants():
    for field, actual in (
        ("NAV_TAKEOFF", qgc_plan.NAV_TAKEOFF),
        ("NAV_WAYPOINT", qgc_plan.NAV_WAYPOINT),
        ("NAV_RETURN_TO_LAUNCH", qgc_plan.NAV_RETURN_TO_LAUNCH),
        ("FRAME_MISSION", qgc_plan.FRAME_MISSION),
        ("FRAME_GLOBAL_RELATIVE_ALT", qgc_plan.FRAME_GLOBAL_RELATIVE_ALT),
    ):
        assert DOC["plan_format"][field]["value"] == actual, field


def _params_file() -> dict[str, float]:
    entries = {}
    for line in (ROOT / "sar_turn_params.params").read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            fields = line.split()
            entries[fields[2]] = float(fields[3])
    return entries


def test_px4_block_matches_the_params_file_exactly():
    live = _params_file()
    documented = {k: v for k, v in DOC["px4_parameters"].items() if k not in METADATA_KEYS}
    assert set(documented) == set(live)
    for name, value in live.items():
        assert float(documented[name]["value"]) == value, name


def test_yaw_mode_is_along_trajectory_not_the_legacy_numeral():
    """MPC_YAW_MODE=1 is 'towards home' -- nose pinned at the launch point. Since 11 Sep 2026
    the survey flies 3, 'along trajectory': the heading follows the path, so the vehicle yaws
    while it banks through a corner instead of stopping to yaw first (4)."""
    entry = DOC["px4_parameters"]["MPC_YAW_MODE"]
    assert entry["value"] == 3
    assert "along trajectory" in entry["enum"][3]
    assert _params_file()["MPC_YAW_MODE"] == 3.0


def test_every_documented_parameter_says_what_it_changes():
    """The point of the file is the annotation -- an entry without one is not useful."""
    missing = []

    def walk(node, path):
        if isinstance(node, dict):
            if "value" in node and "effect" not in node:
                missing.append(path)
            for key, child in node.items():
                if key not in METADATA_KEYS:
                    walk(child, f"{path}.{key}")

    for section, body in DOC.items():
        walk(body, section)
    assert not missing, f"parameters with no 'effect' annotation: {missing}"
