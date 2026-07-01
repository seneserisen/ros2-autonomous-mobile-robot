"""Deterministic wheel-encoder and IMU simulation for FaultNav."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite, pi, sqrt

import numpy as np

from faultnav_robot.differential_drive import RobotState, integrate_twist, wrap_angle
from faultnav_robot.experiments import ExperimentSample


@dataclass(frozen=True)
class TimeWindow:
    """Half-open time interval used to activate a simulated fault."""

    start_s: float
    end_s: float

    def __post_init__(self) -> None:
        if not isfinite(self.start_s) or not isfinite(self.end_s):
            raise ValueError("fault-window bounds must be finite")
        if self.start_s < 0.0:
            raise ValueError("fault-window start_s must be non-negative")
        if self.end_s <= self.start_s:
            raise ValueError("fault-window end_s must be greater than start_s")

    def contains(self, time_s: float) -> bool:
        """Return whether ``time_s`` is inside the half-open interval."""

        return self.start_s <= time_s < self.end_s


@dataclass(frozen=True)
class RobotGeometry:
    """Differential-drive geometry and encoder resolution."""

    wheel_radius_m: float = 0.08
    wheel_separation_m: float = 0.34
    encoder_counts_per_revolution: int = 2048

    def __post_init__(self) -> None:
        if not isfinite(self.wheel_radius_m) or self.wheel_radius_m <= 0.0:
            raise ValueError("wheel_radius_m must be finite and positive")
        if not isfinite(self.wheel_separation_m) or self.wheel_separation_m <= 0.0:
            raise ValueError("wheel_separation_m must be finite and positive")
        if self.encoder_counts_per_revolution <= 0:
            raise ValueError("encoder_counts_per_revolution must be positive")


@dataclass(frozen=True)
class SensorNoiseConfig:
    """Zero-mean Gaussian measurement-noise configuration."""

    wheel_velocity_std_rad_s: float = 0.0
    gyro_std_rad_s: float = 0.0
    acceleration_std_m_s2: float = 0.0
    seed: int = 7

    def __post_init__(self) -> None:
        values = (
            self.wheel_velocity_std_rad_s,
            self.gyro_std_rad_s,
            self.acceleration_std_m_s2,
        )
        if not all(isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("noise standard deviations must be finite and non-negative")


@dataclass(frozen=True)
class SensorFaultConfig:
    """Deterministic calibration and time-window fault configuration."""

    left_encoder_scale_error: float = 0.0
    right_encoder_scale_error: float = 0.0
    left_wheel_slip_fraction: float = 0.0
    right_wheel_slip_fraction: float = 0.0
    wheel_slip_window: TimeWindow | None = None
    gyro_bias_rad_s: float = 0.0
    accelerometer_bias_m_s2: float = 0.0
    imu_dropout_window: TimeWindow | None = None
    gyro_outlier_window: TimeWindow | None = None
    gyro_outlier_rad_s: float = 0.0

    def __post_init__(self) -> None:
        values = (
            self.left_encoder_scale_error,
            self.right_encoder_scale_error,
            self.left_wheel_slip_fraction,
            self.right_wheel_slip_fraction,
            self.gyro_bias_rad_s,
            self.accelerometer_bias_m_s2,
            self.gyro_outlier_rad_s,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("fault magnitudes must be finite")
        if self.left_wheel_slip_fraction <= -1.0 or self.right_wheel_slip_fraction <= -1.0:
            raise ValueError("wheel slip fractions must be greater than -1")


@dataclass(frozen=True)
class SensorSimulationConfig:
    """Complete deterministic sensor-simulation configuration."""

    geometry: RobotGeometry = RobotGeometry()
    noise: SensorNoiseConfig = SensorNoiseConfig()
    faults: SensorFaultConfig = SensorFaultConfig()


@dataclass(frozen=True)
class SensorSample:
    """Ground truth, simulated measurements, and encoder-derived odometry."""

    time_s: float
    dt_s: float
    segment: str
    truth_x_m: float
    truth_y_m: float
    truth_yaw_rad: float
    truth_linear_velocity_m_s: float
    truth_angular_velocity_rad_s: float
    truth_longitudinal_acceleration_m_s2: float
    ideal_left_wheel_rad_s: float
    ideal_right_wheel_rad_s: float
    measured_left_wheel_rad_s: float
    measured_right_wheel_rad_s: float
    left_encoder_count: int
    right_encoder_count: int
    imu_yaw_rate_rad_s: float | None
    imu_longitudinal_acceleration_m_s2: float | None
    wheel_odom_x_m: float
    wheel_odom_y_m: float
    wheel_odom_yaw_rad: float
    wheel_slip_active: bool
    imu_dropout_active: bool
    gyro_outlier_active: bool


@dataclass(frozen=True)
class SensorMetrics:
    """Error and fault-duration metrics for one sensor experiment."""

    sample_count: int
    duration_s: float
    wheel_position_rmse_m: float
    wheel_heading_rmse_rad: float
    final_wheel_position_error_m: float
    final_wheel_heading_error_rad: float
    max_wheel_position_error_m: float
    max_abs_gyro_error_rad_s: float
    wheel_slip_duration_s: float
    imu_dropout_duration_s: float
    gyro_outlier_duration_s: float


def body_twist_to_wheel_rates(
    linear_velocity_m_s: float,
    angular_velocity_rad_s: float,
    geometry: RobotGeometry,
) -> tuple[float, float]:
    """Convert a planar body twist to left and right wheel angular rates."""

    half_track_velocity = 0.5 * geometry.wheel_separation_m * angular_velocity_rad_s
    left_rad_s = (linear_velocity_m_s - half_track_velocity) / geometry.wheel_radius_m
    right_rad_s = (linear_velocity_m_s + half_track_velocity) / geometry.wheel_radius_m
    return left_rad_s, right_rad_s


def _window_active(window: TimeWindow | None, time_s: float) -> bool:
    return window is not None and window.contains(time_s)


def simulate_sensors(
    truth_samples: tuple[ExperimentSample, ...],
    config: SensorSimulationConfig | None = None,
) -> tuple[SensorSample, ...]:
    """Generate deterministic encoder, IMU, and wheel-odometry samples.

    Wheel odometry is reconstructed from quantised cumulative encoder counts. Ground truth remains
    independent from the corrupted measurement path.
    """

    if not truth_samples:
        raise ValueError("truth_samples must not be empty")

    resolved_config = config if config is not None else SensorSimulationConfig()
    rng = np.random.default_rng(resolved_config.noise.seed)
    geometry = resolved_config.geometry
    counts_per_rad = geometry.encoder_counts_per_revolution / (2.0 * pi)

    left_count_float = 0.0
    right_count_float = 0.0
    previous_left_count = 0
    previous_right_count = 0
    previous_truth_velocity = truth_samples[0].linear_velocity_m_s
    wheel_odom = RobotState(
        x_m=truth_samples[0].x_m,
        y_m=truth_samples[0].y_m,
        yaw_rad=truth_samples[0].yaw_rad,
    )

    output = [
        SensorSample(
            time_s=truth_samples[0].time_s,
            dt_s=0.0,
            segment=truth_samples[0].segment,
            truth_x_m=truth_samples[0].x_m,
            truth_y_m=truth_samples[0].y_m,
            truth_yaw_rad=truth_samples[0].yaw_rad,
            truth_linear_velocity_m_s=truth_samples[0].linear_velocity_m_s,
            truth_angular_velocity_rad_s=truth_samples[0].angular_velocity_rad_s,
            truth_longitudinal_acceleration_m_s2=0.0,
            ideal_left_wheel_rad_s=0.0,
            ideal_right_wheel_rad_s=0.0,
            measured_left_wheel_rad_s=0.0,
            measured_right_wheel_rad_s=0.0,
            left_encoder_count=0,
            right_encoder_count=0,
            imu_yaw_rate_rad_s=0.0,
            imu_longitudinal_acceleration_m_s2=0.0,
            wheel_odom_x_m=wheel_odom.x_m,
            wheel_odom_y_m=wheel_odom.y_m,
            wheel_odom_yaw_rad=wheel_odom.yaw_rad,
            wheel_slip_active=False,
            imu_dropout_active=False,
            gyro_outlier_active=False,
        )
    ]

    for previous, truth in zip(truth_samples[:-1], truth_samples[1:], strict=True):
        dt_s = truth.time_s - previous.time_s
        if dt_s <= 0.0:
            raise ValueError("truth sample times must be strictly increasing")

        midpoint_s = previous.time_s + 0.5 * dt_s
        slip_active = _window_active(resolved_config.faults.wheel_slip_window, midpoint_s)
        dropout_active = _window_active(resolved_config.faults.imu_dropout_window, midpoint_s)
        outlier_active = _window_active(resolved_config.faults.gyro_outlier_window, midpoint_s)

        ideal_left_rad_s, ideal_right_rad_s = body_twist_to_wheel_rates(
            truth.linear_velocity_m_s,
            truth.angular_velocity_rad_s,
            geometry,
        )

        left_factor = 1.0 + resolved_config.faults.left_encoder_scale_error
        right_factor = 1.0 + resolved_config.faults.right_encoder_scale_error
        if slip_active:
            left_factor *= 1.0 + resolved_config.faults.left_wheel_slip_fraction
            right_factor *= 1.0 + resolved_config.faults.right_wheel_slip_fraction

        measured_left_rad_s = (
            ideal_left_rad_s * left_factor
            + rng.normal(0.0, resolved_config.noise.wheel_velocity_std_rad_s)
        )
        measured_right_rad_s = (
            ideal_right_rad_s * right_factor
            + rng.normal(0.0, resolved_config.noise.wheel_velocity_std_rad_s)
        )

        left_count_float += measured_left_rad_s * dt_s * counts_per_rad
        right_count_float += measured_right_rad_s * dt_s * counts_per_rad
        left_count = int(round(left_count_float))
        right_count = int(round(right_count_float))
        delta_left_count = left_count - previous_left_count
        delta_right_count = right_count - previous_right_count
        previous_left_count = left_count
        previous_right_count = right_count

        delta_left_rad = delta_left_count / counts_per_rad
        delta_right_rad = delta_right_count / counts_per_rad
        left_distance_m = geometry.wheel_radius_m * delta_left_rad
        right_distance_m = geometry.wheel_radius_m * delta_right_rad
        delta_distance_m = 0.5 * (left_distance_m + right_distance_m)
        delta_yaw_rad = (right_distance_m - left_distance_m) / geometry.wheel_separation_m
        wheel_odom = integrate_twist(
            wheel_odom,
            delta_distance_m / dt_s,
            delta_yaw_rad / dt_s,
            dt_s,
        )

        truth_acceleration_m_s2 = (
            truth.linear_velocity_m_s - previous_truth_velocity
        ) / dt_s
        previous_truth_velocity = truth.linear_velocity_m_s

        if dropout_active:
            imu_yaw_rate_rad_s = None
            imu_acceleration_m_s2 = None
        else:
            imu_yaw_rate_rad_s = (
                truth.angular_velocity_rad_s
                + resolved_config.faults.gyro_bias_rad_s
                + rng.normal(0.0, resolved_config.noise.gyro_std_rad_s)
            )
            if outlier_active:
                imu_yaw_rate_rad_s += resolved_config.faults.gyro_outlier_rad_s
            imu_acceleration_m_s2 = (
                truth_acceleration_m_s2
                + resolved_config.faults.accelerometer_bias_m_s2
                + rng.normal(0.0, resolved_config.noise.acceleration_std_m_s2)
            )

        output.append(
            SensorSample(
                time_s=truth.time_s,
                dt_s=dt_s,
                segment=truth.segment,
                truth_x_m=truth.x_m,
                truth_y_m=truth.y_m,
                truth_yaw_rad=truth.yaw_rad,
                truth_linear_velocity_m_s=truth.linear_velocity_m_s,
                truth_angular_velocity_rad_s=truth.angular_velocity_rad_s,
                truth_longitudinal_acceleration_m_s2=truth_acceleration_m_s2,
                ideal_left_wheel_rad_s=ideal_left_rad_s,
                ideal_right_wheel_rad_s=ideal_right_rad_s,
                measured_left_wheel_rad_s=measured_left_rad_s,
                measured_right_wheel_rad_s=measured_right_rad_s,
                left_encoder_count=left_count,
                right_encoder_count=right_count,
                imu_yaw_rate_rad_s=imu_yaw_rate_rad_s,
                imu_longitudinal_acceleration_m_s2=imu_acceleration_m_s2,
                wheel_odom_x_m=wheel_odom.x_m,
                wheel_odom_y_m=wheel_odom.y_m,
                wheel_odom_yaw_rad=wheel_odom.yaw_rad,
                wheel_slip_active=slip_active,
                imu_dropout_active=dropout_active,
                gyro_outlier_active=outlier_active,
            )
        )

    return tuple(output)


def calculate_sensor_metrics(samples: tuple[SensorSample, ...]) -> SensorMetrics:
    """Calculate odometry degradation and fault-duration metrics."""

    if not samples:
        raise ValueError("samples must not be empty")

    position_errors = [
        hypot(sample.wheel_odom_x_m - sample.truth_x_m, sample.wheel_odom_y_m - sample.truth_y_m)
        for sample in samples
    ]
    heading_errors = [
        wrap_angle(sample.wheel_odom_yaw_rad - sample.truth_yaw_rad) for sample in samples
    ]
    gyro_errors = [
        abs(sample.imu_yaw_rate_rad_s - sample.truth_angular_velocity_rad_s)
        for sample in samples
        if sample.imu_yaw_rate_rad_s is not None
    ]

    final = samples[-1]
    return SensorMetrics(
        sample_count=len(samples),
        duration_s=final.time_s,
        wheel_position_rmse_m=sqrt(
            sum(error * error for error in position_errors) / len(position_errors)
        ),
        wheel_heading_rmse_rad=sqrt(
            sum(error * error for error in heading_errors) / len(heading_errors)
        ),
        final_wheel_position_error_m=position_errors[-1],
        final_wheel_heading_error_rad=heading_errors[-1],
        max_wheel_position_error_m=max(position_errors),
        max_abs_gyro_error_rad_s=max(gyro_errors, default=0.0),
        wheel_slip_duration_s=sum(
            sample.dt_s for sample in samples if sample.wheel_slip_active
        ),
        imu_dropout_duration_s=sum(
            sample.dt_s for sample in samples if sample.imu_dropout_active
        ),
        gyro_outlier_duration_s=sum(
            sample.dt_s for sample in samples if sample.gyro_outlier_active
        ),
    )


def sensor_profile(name: str, seed: int = 7) -> SensorSimulationConfig:
    """Return a documented sensor and fault profile."""

    noise = SensorNoiseConfig(
        wheel_velocity_std_rad_s=0.01,
        gyro_std_rad_s=0.003,
        acceleration_std_m_s2=0.03,
        seed=seed,
    )
    if name == "nominal":
        return SensorSimulationConfig(noise=noise)
    if name == "wheel-slip":
        return SensorSimulationConfig(
            noise=noise,
            faults=SensorFaultConfig(
                left_wheel_slip_fraction=0.25,
                right_wheel_slip_fraction=-0.05,
                wheel_slip_window=TimeWindow(6.0, 12.0),
            ),
        )
    if name == "gyro-bias":
        return SensorSimulationConfig(
            noise=noise,
            faults=SensorFaultConfig(gyro_bias_rad_s=0.05),
        )
    if name == "combined-faults":
        return SensorSimulationConfig(
            noise=noise,
            faults=SensorFaultConfig(
                left_encoder_scale_error=0.01,
                right_encoder_scale_error=-0.005,
                left_wheel_slip_fraction=0.25,
                right_wheel_slip_fraction=-0.05,
                wheel_slip_window=TimeWindow(6.0, 12.0),
                gyro_bias_rad_s=0.04,
                accelerometer_bias_m_s2=0.02,
                imu_dropout_window=TimeWindow(16.0, 18.0),
                gyro_outlier_window=TimeWindow(20.0, 20.2),
                gyro_outlier_rad_s=0.8,
            ),
        )
    choices = "combined-faults, gyro-bias, nominal, wheel-slip"
    raise ValueError(f"unsupported sensor profile '{name}'; choose one of: {choices}")
