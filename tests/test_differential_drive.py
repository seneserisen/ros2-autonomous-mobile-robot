from __future__ import annotations

from math import isclose, pi

import pytest

from faultnav_robot.differential_drive import RobotState, integrate_twist, wrap_angle


def test_integrates_straight_motion() -> None:
    result = integrate_twist(RobotState(), 2.0, 0.0, 1.5)

    assert isclose(result.x_m, 3.0)
    assert isclose(result.y_m, 0.0, abs_tol=1e-12)
    assert isclose(result.yaw_rad, 0.0, abs_tol=1e-12)


def test_integrates_rotation_in_place() -> None:
    result = integrate_twist(RobotState(), 0.0, pi / 2.0, 1.0)

    assert isclose(result.x_m, 0.0, abs_tol=1e-12)
    assert isclose(result.y_m, 0.0, abs_tol=1e-12)
    assert isclose(result.yaw_rad, pi / 2.0)


def test_integrates_constant_radius_arc() -> None:
    result = integrate_twist(RobotState(), 1.0, 1.0, pi / 2.0)

    assert isclose(result.x_m, 1.0, rel_tol=1e-12)
    assert isclose(result.y_m, 1.0, rel_tol=1e-12)
    assert isclose(result.yaw_rad, pi / 2.0, rel_tol=1e-12)


def test_zero_interval_returns_same_state() -> None:
    state = RobotState(x_m=1.0, y_m=-2.0, yaw_rad=0.4)

    assert integrate_twist(state, 5.0, 2.0, 0.0) == state


def test_rejects_negative_interval() -> None:
    with pytest.raises(ValueError, match="non-negative"):
        integrate_twist(RobotState(), 1.0, 0.0, -0.1)


@pytest.mark.parametrize(
    ("input_angle", "expected"),
    [
        (0.0, 0.0),
        (pi, -pi),
        (-pi, -pi),
        (3.0 * pi, -pi),
        (-3.0 * pi, -pi),
    ],
)
def test_wrap_angle(input_angle: float, expected: float) -> None:
    assert isclose(wrap_angle(input_angle), expected, abs_tol=1e-12)
