"""The abort ladder decides whether a flight continues, so pin every rung.

A fake clock is passed in everywhere, so these are exact, not timing-dependent.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import safety_rules as sr  # noqa: E402


def _wd(**kw):
    return sr.Watchdog(sr.Limits(**kw), now=0.0)


def _quiet(wd, t):
    """Keep telemetry alive so the gap rules do not fire during other tests."""
    wd.heartbeat(t)


# ---- the error-accumulation rate ------------------------------------------------------

def test_a_healthy_aircraft_stays_at_none():
    wd = _wd()
    for t in range(0, 60, 2):
        wd.heartbeat(float(t))
        wd.statustext(6, "Ready for takeoff", float(t))   # INFO scores nothing
    assert wd.evaluate(59.0).action == sr.NONE
    assert wd.evaluate(59.0).error_rate == 0.0


def test_one_warning_is_not_an_emergency():
    wd = _wd()
    _quiet(wd, 1.0)
    wd.statustext(4, "EKF high accel bias", 1.0)
    v = wd.evaluate(1.0)
    assert v.action == sr.NONE
    assert v.error_rate == pytest.approx(1.0)


def test_errors_accumulating_fast_force_a_landing():
    wd = _wd()
    _quiet(wd, 10.0)
    for i in range(8):                      # 8 x ERROR(2.0) = 16.0/min >= land_rate 14
        wd.statustext(3, f"EKF fault {i}", 1.0 + i * 0.5)
    v = wd.evaluate(10.0)
    assert v.action == sr.LAND
    assert "error rate" in v.reasons[0]


def test_the_same_errors_spread_out_only_warn():
    wd = _wd()
    _quiet(wd, 600.0)
    for i in range(8):                      # same 8 errors, but over 10 minutes
        wd.statustext(3, f"EKF fault {i}", i * 75.0)
    v = wd.evaluate(600.0)
    assert v.action == sr.NONE              # window has slid past all of them
    assert v.error_rate == 0.0


def test_severity_is_weighted_not_counted():
    wd = _wd()
    _quiet(wd, 2.0)
    wd.statustext(0, "EMERGENCY", 1.0)      # one EMERGENCY = 8.0/min
    assert wd.evaluate(2.0).action == sr.HOLD
    wd.statustext(0, "EMERGENCY again", 1.5)
    assert wd.evaluate(2.0).action == sr.LAND


def test_the_rate_decays_as_the_window_slides():
    wd = _wd()
    for i in range(8):
        wd.statustext(3, f"fault {i}", 1.0 + i * 0.5)
    assert wd.error_rate(10.0) == pytest.approx(16.0)
    assert wd.error_rate(70.0) == 0.0       # every event is older than the window


# ---- latching -------------------------------------------------------------------------

def test_the_ladder_never_climbs_back_down():
    wd = _wd()
    for i in range(8):
        wd.statustext(3, f"fault {i}", 1.0 + i * 0.5)
    assert wd.evaluate(10.0).action == sr.LAND
    _quiet(wd, 200.0)
    assert wd.evaluate(200.0).error_rate == 0.0
    assert wd.evaluate(200.0).action == sr.LAND     # still landing. Recovery is human.


def test_climbed_is_only_true_on_the_step_up():
    wd = _wd()
    _quiet(wd, 2.0)
    wd.statustext(0, "EMERGENCY", 1.0)
    assert wd.evaluate(2.0).climbed is True
    assert wd.evaluate(2.0).climbed is False


# ---- the other rules ------------------------------------------------------------------

def test_battery_holds_then_lands():
    wd = _wd()
    _quiet(wd, 1.0)
    wd.battery(24.0, 1.0)
    assert wd.evaluate(1.0).action == sr.HOLD
    wd.battery(14.0, 2.0)
    _quiet(wd, 2.0)
    assert wd.evaluate(2.0).action == sr.LAND


def test_an_unknown_battery_is_not_a_flat_battery():
    wd = _wd()
    _quiet(wd, 1.0)
    wd.battery(-1.0, 1.0)                   # PX4 sends -1 when it has no estimate
    assert wd.evaluate(1.0).action == sr.NONE


def test_extreme_tilt_lands():
    wd = _wd()
    _quiet(wd, 1.0)
    wd.attitude(roll_deg=-62.0, pitch_deg=3.0, now=1.0)
    v = wd.evaluate(1.0)
    assert v.action == sr.LAND
    assert "tilt 62" in v.reasons[0]


def test_losing_telemetry_escalates_with_time():
    wd = _wd()
    wd.heartbeat(10.0)
    assert wd.evaluate(14.0).action == sr.NONE     # a few dropped heartbeats are normal
    assert wd.evaluate(15.5).action == sr.HOLD
    assert wd.evaluate(22.5).action == sr.LAND


def test_losing_the_attitude_solution_lands_immediately():
    wd = _wd()
    _quiet(wd, 1.0)
    wd.estimator(attitude_ok=False, worst_innovation=None, now=1.0)
    assert wd.evaluate(1.0).action == sr.LAND


def test_one_innovation_spike_is_tolerated_but_a_trend_is_not():
    wd = _wd()
    _quiet(wd, 1.0)
    wd.estimator(attitude_ok=True, worst_innovation=1.4, now=1.0)
    assert wd.evaluate(2.0).action == sr.NONE        # 1 s of it
    wd.estimator(attitude_ok=True, worst_innovation=0.3, now=2.0)
    _quiet(wd, 9.0)
    assert wd.evaluate(9.0).action == sr.NONE        # recovered, timer reset

    wd2 = _wd()
    for t in (1.0, 2.0, 3.0, 4.0, 5.0):
        wd2.heartbeat(t)
        wd2.estimator(attitude_ok=True, worst_innovation=1.4, now=t)
    assert wd2.evaluate(5.0).action == sr.LAND       # 4 s sustained


def test_an_operator_command_joins_the_ladder():
    wd = _wd()
    _quiet(wd, 1.0)
    wd.manual(sr.LAND, "pilot called it")
    v = wd.evaluate(1.0)
    assert v.action == sr.LAND
    assert "operator: pilot called it" in v.reasons


def test_an_unknown_operator_command_is_refused():
    wd = _wd()
    with pytest.raises(ValueError):
        wd.manual("explode", "no")


def test_reasons_accumulate_without_duplicating():
    wd = _wd()
    _quiet(wd, 1.0)
    wd.battery(10.0, 1.0)
    wd.evaluate(1.0)
    wd.evaluate(1.0)
    assert sum("battery" in r for r in wd.evaluate(1.0).reasons) == 1


# ---- never disarm something that is still flying --------------------------------------

def test_disarm_in_the_air_is_downgraded_to_land():
    assert sr.on_ground_action(sr.DISARM, airborne=True) == sr.LAND
    assert sr.on_ground_action(sr.DISARM, airborne=None) == sr.LAND
    assert sr.on_ground_action(sr.DISARM, airborne=False) == sr.DISARM


def test_landing_something_already_on_the_ground_means_cut_the_motors():
    assert sr.on_ground_action(sr.LAND, airborne=False) == sr.DISARM
    assert sr.on_ground_action(sr.LAND, airborne=True) == sr.LAND
    assert sr.on_ground_action(sr.LAND, airborne=None) == sr.LAND


def test_the_lower_rungs_pass_through_untouched():
    for a in (sr.NONE, sr.WARN, sr.HOLD):
        for air in (True, False, None):
            assert sr.on_ground_action(a, air) == a
