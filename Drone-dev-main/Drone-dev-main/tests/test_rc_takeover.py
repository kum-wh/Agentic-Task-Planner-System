"""Whether the RC switch can take the aircraft is a yes/no question with specific causes.

The most important case is the PX4 default: COM_RC_OVERRIDE=1 looks enabled but leaves
OFFBOARD -- the mode every arming test uses -- immune to the sticks.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import drone_agent as agent  # noqa: E402


def _good(**over):
    p = {"COM_RC_IN_MODE": 0, "COM_RC_OVERRIDE": 3, "COM_RC_STICK_OV": 30.0,
         "RC_MAP_FLTMODE": 5, "RC_MAP_KILL_SW": 7, "COM_RCL_EXCEPT": 0,
         "COM_FLTMODE1": 2, "COM_FLTMODE2": -1, "COM_FLTMODE3": -1,
         "COM_FLTMODE4": -1, "COM_FLTMODE5": -1, "COM_FLTMODE6": 7}
    p.update(over)
    return p


def _findings(p, rc_seen=True):
    return agent.rc_takeover_findings(p, rc_seen)


def _notes(p):
    return agent.rc_takeover_notes(p)


def test_a_properly_configured_aircraft_has_nothing_to_report():
    assert _findings(_good()) == []


def test_stick_override_is_a_policy_choice_not_a_fault():
    # Decided 6 Oct 2026: COM_RC_OVERRIDE stays at PX4's default 1, so a knocked stick
    # cannot drop an aircraft out of OFFBOARD. The mode switch is the takeover path.
    assert _findings(_good(COM_RC_OVERRIDE=1)) == []
    n = _notes(_good(COM_RC_OVERRIDE=1))
    assert any("will NOT break OFFBOARD" in x for x in n)


def test_override_disabled_entirely_is_noted_twice():
    n = _notes(_good(COM_RC_OVERRIDE=0))
    assert any("OFFBOARD" in x for x in n)
    assert any("AUTO" in x for x in n)


def test_full_override_has_nothing_to_note():
    assert _notes(_good(COM_RC_OVERRIDE=3, RC_MAP_KILL_SW=8)) == []


def test_an_unreadable_override_is_noted_not_assumed_good():
    assert any("could not be read" in x for x in _notes(_good(COM_RC_OVERRIDE=None)))


def test_stick_input_disabled_is_caught():
    assert any("does not accept RC stick input" in x
               for x in _findings(_good(COM_RC_IN_MODE=4)))


def test_joystick_only_is_not_good_enough_for_a_real_transmitter():
    assert any("COM_RC_IN_MODE=1" in x for x in _findings(_good(COM_RC_IN_MODE=1)))


def test_every_rc_accepting_input_mode_is_allowed():
    # 3 is PX4's own default ("keep first"); RC wins when RC is seen first, and the QGC
    # mirror is observe-only so no joystick can get in ahead of it.
    for mode in (0, 2, 3):
        assert _findings(_good(COM_RC_IN_MODE=mode)) == [], f"mode {mode} should be accepted"


def test_no_mode_channel_is_caught():
    assert any("RC_MAP_FLTMODE=0" in x for x in _findings(_good(RC_MAP_FLTMODE=0)))


def test_a_mode_switch_that_never_hands_over_is_caught():
    # every slot is still the autopilot flying: Mission, Hold, Return, Offboard
    f = _findings(_good(COM_FLTMODE1=3, COM_FLTMODE6=5))
    assert any("hand-flown mode" in x for x in f)


def test_each_hand_flown_mode_counts():
    for mode in (0, 1, 2, 6, 8, 9):
        assert _findings(_good(COM_FLTMODE1=mode)) == [], f"mode {mode} should hand over"


def test_autopilot_modes_do_not_count_as_handing_over():
    for mode in (3, 4, 5, 7, 10, 11):
        assert any("hand-flown" in x for x in _findings(_good(COM_FLTMODE1=mode, COM_FLTMODE6=-1)))


def test_a_missing_kill_switch_is_noted_but_does_not_block():
    assert _findings(_good(RC_MAP_KILL_SW=0)) == []
    assert any("RC_MAP_KILL_SW=0" in x for x in _notes(_good(RC_MAP_KILL_SW=0)))


def test_a_transmitter_that_was_never_heard_is_the_headline():
    assert any("no RC_CHANNELS heard" in x for x in _findings(_good(), rc_seen=False))


def test_artex113_as_actually_configured_is_ready():
    """The real 113, read 6 Oct 2026: mode switch on ch7, slot 1 Altitude, kill on ch8."""
    real = {"COM_RC_IN_MODE": 3, "COM_RC_OVERRIDE": 1, "COM_RC_STICK_OV": 30.0,
            "RC_MAP_FLTMODE": 7, "RC_MAP_KILL_SW": 8, "COM_RCL_EXCEPT": 0,
            "COM_FLTMODE1": 1, "COM_FLTMODE2": -1, "COM_FLTMODE3": -1,
            "COM_FLTMODE4": 2, "COM_FLTMODE5": -1, "COM_FLTMODE6": 2}
    assert _findings(real) == []                     # nothing blocks the switch
    assert any("NOT break OFFBOARD" in x for x in _notes(real))
