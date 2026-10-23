"""The read-back comparison decides whether an upload counts as verified."""

import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import drone_agent as agent  # noqa: E402


def _items():
    return [
        dict(seq=0, frame=3, command=22, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
             x=14172000, y=1038636000, z=30.0),
        dict(seq=1, frame=3, command=16, p1=0.0, p2=4.83, p3=0.0, p4=0.0,
             x=14173000, y=1038637000, z=30.0),
        dict(seq=2, frame=2, command=20, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
             x=0, y=0, z=0.0),
    ]


def test_identical_mission_has_no_differences():
    assert agent.diff_items(_items(), copy.deepcopy(_items())) == []


def test_px4_frame_renumbering_is_not_a_difference():
    # PX4 stores the _INT frames: local-relative 3 reads back as 6, and 0 as 5.
    got = copy.deepcopy(_items())
    got[0]["frame"], got[1]["frame"] = 6, 6
    assert agent.diff_items(_items(), got) == []


def test_real_differences_are_named_by_item():
    got = copy.deepcopy(_items())
    got[1]["z"] += 2.0
    got[1]["x"] += 1
    got[2]["command"] = 17
    assert agent.diff_items(_items(), got) == [
        "item 1: position/z differ",
        "item 2: command differ",
    ]


def test_missing_items_are_reported():
    assert agent.diff_items(_items(), _items()[:2]) == ["2 items on board, 3 sent"]
