from __future__ import annotations

from math import isclose, pi

import pytest

from faultnav_robot.scenarios import (
    CommandSegment,
    available_scenarios,
    circle_scenario,
    get_scenario,
    square_scenario,
)


def test_available_scenarios_are_uniquely_named() -> None:
    scenarios = available_scenarios()

    assert set(scenarios) == {"circle", "figure-eight", "square", "straight"}
    assert all(name == scenario.name for name, scenario in scenarios.items())


def test_circle_completes_one_full_rotation() -> None:
    scenario = circle_scenario()
    segment = scenario.segments[0]

    assert isclose(segment.angular_velocity_rad_s * segment.duration_s, 2.0 * pi)
    assert isclose(segment.linear_velocity_m_s / segment.angular_velocity_rad_s, 1.0)


def test_square_contains_four_sides_and_four_turns() -> None:
    scenario = square_scenario()

    assert len(scenario.segments) == 8
    assert isclose(scenario.duration_s, 16.0)
    assert sum(segment.linear_velocity_m_s > 0.0 for segment in scenario.segments) == 4


def test_command_segment_rejects_invalid_duration() -> None:
    with pytest.raises(ValueError, match="positive"):
        CommandSegment("invalid", 0.0, 1.0, 0.0)


def test_get_scenario_reports_supported_choices() -> None:
    with pytest.raises(ValueError, match="figure-eight"):
        get_scenario("unknown")
