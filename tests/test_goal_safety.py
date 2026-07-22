import math

import pytest

from main_thread import MainThread
from pathPlaning.PathMain import MainPathPlaning


def test_virtual_goal_is_midpoint_of_old_offset_and_marker():
    planner = MainPathPlaning()

    assert planner.newGoalState([300.0, 200.0, 0.0]) == pytest.approx(
        [200.0, 200.0, 0.0]
    )
    assert planner.newGoalState([300.0, 200.0, math.pi / 2]) == pytest.approx(
        [300.0, 100.0, math.pi / 2]
    )


def test_pose_change_threshold_filters_marker_jitter():
    reference = [100.0, 100.0, 0.0]

    assert not MainThread._pose_differs([119.0, 100.0, 0.0], reference)
    assert MainThread._pose_differs([120.0, 100.0, 0.0], reference)
    assert not MainThread._pose_differs(
        [100.0, 100.0, math.radians(7.9)], reference
    )
    assert MainThread._pose_differs(
        [100.0, 100.0, math.radians(8.0)], reference
    )
