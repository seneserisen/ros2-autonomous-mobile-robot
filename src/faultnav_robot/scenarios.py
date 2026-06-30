"""Deterministic motion scenarios used by FaultNav experiments."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi


@dataclass(frozen=True)
class CommandSegment:
    """Constant body-frame velocity command applied for a fixed duration."""

    label: str
    duration_s: float
    linear_velocity_m_s: float
    angular_velocity_rad_s: float

    def __post_init__(self) -> None:
        values = (
            self.duration_s,
            self.linear_velocity_m_s,
            self.angular_velocity_rad_s,
        )
        if not self.label.strip():
            raise ValueError("segment label must not be empty")
        if not all(isfinite(value) for value in values):
            raise ValueError("segment values must be finite")
        if self.duration_s <= 0.0:
            raise ValueError("segment duration_s must be positive")


@dataclass(frozen=True)
class MotionScenario:
    """Named sequence of constant-twist motion commands."""

    name: str
    description: str
    segments: tuple[CommandSegment, ...]

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("scenario name must not be empty")
        if not self.description.strip():
            raise ValueError("scenario description must not be empty")
        if not self.segments:
            raise ValueError("scenario must contain at least one segment")

    @property
    def duration_s(self) -> float:
        """Return the total commanded duration."""

        return sum(segment.duration_s for segment in self.segments)


def straight_scenario() -> MotionScenario:
    """Drive two metres along the positive x-axis."""

    return MotionScenario(
        name="straight",
        description="Two-metre straight-line reference motion.",
        segments=(
            CommandSegment(
                label="forward",
                duration_s=4.0,
                linear_velocity_m_s=0.5,
                angular_velocity_rad_s=0.0,
            ),
        ),
    )


def circle_scenario() -> MotionScenario:
    """Complete one circle with one-metre radius."""

    angular_velocity_rad_s = 0.5
    return MotionScenario(
        name="circle",
        description="One counter-clockwise circle with one-metre radius.",
        segments=(
            CommandSegment(
                label="counter_clockwise_circle",
                duration_s=2.0 * pi / angular_velocity_rad_s,
                linear_velocity_m_s=0.5,
                angular_velocity_rad_s=angular_velocity_rad_s,
            ),
        ),
    )


def square_scenario() -> MotionScenario:
    """Drive a one-metre square with in-place quarter turns."""

    segments: list[CommandSegment] = []
    for side_number in range(1, 5):
        segments.append(
            CommandSegment(
                label=f"side_{side_number}",
                duration_s=2.0,
                linear_velocity_m_s=0.5,
                angular_velocity_rad_s=0.0,
            )
        )
        segments.append(
            CommandSegment(
                label=f"turn_{side_number}",
                duration_s=2.0,
                linear_velocity_m_s=0.0,
                angular_velocity_rad_s=pi / 4.0,
            )
        )
    return MotionScenario(
        name="square",
        description="One-metre square with four ninety-degree in-place turns.",
        segments=tuple(segments),
    )


def figure_eight_scenario() -> MotionScenario:
    """Complete two tangent circles with opposite yaw rates."""

    angular_velocity_rad_s = 0.5
    loop_duration_s = 2.0 * pi / angular_velocity_rad_s
    return MotionScenario(
        name="figure-eight",
        description="Two tangent one-metre-radius loops with opposite turn direction.",
        segments=(
            CommandSegment(
                label="left_loop",
                duration_s=loop_duration_s,
                linear_velocity_m_s=0.5,
                angular_velocity_rad_s=angular_velocity_rad_s,
            ),
            CommandSegment(
                label="right_loop",
                duration_s=loop_duration_s,
                linear_velocity_m_s=0.5,
                angular_velocity_rad_s=-angular_velocity_rad_s,
            ),
        ),
    )


def available_scenarios() -> dict[str, MotionScenario]:
    """Return all built-in deterministic scenarios indexed by CLI name."""

    scenarios = (
        straight_scenario(),
        circle_scenario(),
        square_scenario(),
        figure_eight_scenario(),
    )
    return {scenario.name: scenario for scenario in scenarios}


def get_scenario(name: str) -> MotionScenario:
    """Return a built-in scenario or raise a descriptive error."""

    scenarios = available_scenarios()
    try:
        return scenarios[name]
    except KeyError as error:
        choices = ", ".join(sorted(scenarios))
        raise ValueError(f"unsupported scenario '{name}'; choose one of: {choices}") from error
