"""Extended Kalman filtering and innovation-based fault monitoring."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, isfinite, pi, sin, sqrt

import numpy as np

from faultnav_robot.differential_drive import wrap_angle
from faultnav_robot.sensors import RobotGeometry, SensorSample

STATE_SIZE = 6
POSITION_X = 0
POSITION_Y = 1
YAW = 2
LINEAR_VELOCITY = 3
YAW_RATE = 4
GYRO_BIAS = 5


@dataclass(frozen=True)
class EkfNoiseConfig:
    """Process and measurement standard deviations."""

    position_process_std_m: float = 0.01
    yaw_process_std_rad: float = 0.01
    velocity_process_std_m_s: float = 0.02
    yaw_rate_process_std_rad_s: float = 0.08
    gyro_bias_process_std_rad_s: float = 0.002
    wheel_velocity_std_m_s: float = 0.006
    wheel_yaw_rate_std_rad_s: float = 0.015
    gyro_std_rad_s: float = 0.01

    def __post_init__(self) -> None:
        values = (
            self.position_process_std_m,
            self.yaw_process_std_rad,
            self.velocity_process_std_m_s,
            self.yaw_rate_process_std_rad_s,
            self.gyro_bias_process_std_rad_s,
            self.wheel_velocity_std_m_s,
            self.wheel_yaw_rate_std_rad_s,
            self.gyro_std_rad_s,
        )
        if not all(isfinite(value) and value > 0.0 for value in values):
            raise ValueError("EKF standard deviations must be finite and positive")


@dataclass(frozen=True)
class InnovationGateConfig:
    """Scalar NIS thresholds and angular-sensor agreement threshold."""

    wheel_velocity_nis: float | None = None
    wheel_yaw_rate_nis: float | None = None
    gyro_nis: float | None = None
    angular_agreement_rad_s: float = 0.15

    def __post_init__(self) -> None:
        thresholds = (
            self.wheel_velocity_nis,
            self.wheel_yaw_rate_nis,
            self.gyro_nis,
        )
        if not all(
            value is None or (isfinite(value) and value > 0.0)
            for value in thresholds
        ):
            raise ValueError("NIS thresholds must be positive when enabled")
        if not isfinite(self.angular_agreement_rad_s) or self.angular_agreement_rad_s <= 0.0:
            raise ValueError("angular_agreement_rad_s must be finite and positive")


@dataclass(frozen=True)
class EkfConfig:
    """Complete EKF configuration."""

    noise: EkfNoiseConfig = EkfNoiseConfig()
    gates: InnovationGateConfig = InnovationGateConfig()
    initial_position_std_m: float = 0.05
    initial_yaw_std_rad: float = 0.05
    initial_velocity_std_m_s: float = 0.2
    initial_yaw_rate_std_rad_s: float = 0.2
    initial_gyro_bias_std_rad_s: float = 0.1

    def __post_init__(self) -> None:
        values = (
            self.initial_position_std_m,
            self.initial_yaw_std_rad,
            self.initial_velocity_std_m_s,
            self.initial_yaw_rate_std_rad_s,
            self.initial_gyro_bias_std_rad_s,
        )
        if not all(isfinite(value) and value > 0.0 for value in values):
            raise ValueError("initial standard deviations must be finite and positive")


@dataclass(frozen=True)
class InnovationResult:
    """One scalar EKF update result."""

    innovation: float
    variance: float
    nis: float
    accepted: bool


@dataclass(frozen=True)
class EstimatorSample:
    """Timestamped EKF state, uncertainty, measurements, and gate decisions."""

    time_s: float
    truth_x_m: float
    truth_y_m: float
    truth_yaw_rad: float
    estimated_x_m: float
    estimated_y_m: float
    estimated_yaw_rad: float
    estimated_linear_velocity_m_s: float
    estimated_yaw_rate_rad_s: float
    estimated_gyro_bias_rad_s: float
    covariance_trace: float
    covariance_x_m2: float
    covariance_y_m2: float
    covariance_yaw_rad2: float
    wheel_linear_velocity_m_s: float
    wheel_yaw_rate_rad_s: float
    imu_yaw_rate_rad_s: float | None
    imu_acceleration_m_s2: float | None
    wheel_velocity_nis: float
    wheel_yaw_rate_nis: float
    gyro_nis: float | None
    wheel_velocity_accepted: bool
    wheel_yaw_rate_accepted: bool
    gyro_accepted: bool
    wheel_slip_active: bool
    imu_dropout_active: bool
    gyro_outlier_active: bool


@dataclass(frozen=True)
class EstimatorMetrics:
    """Accuracy, uncertainty, gate, and recovery metrics for one EKF run."""

    sample_count: int
    duration_s: float
    position_rmse_m: float
    heading_rmse_rad: float
    final_position_error_m: float
    final_heading_error_rad: float
    max_covariance_trace: float
    wheel_velocity_rejections: int
    wheel_yaw_rate_rejections: int
    gyro_rejections: int
    nominal_false_rejections: int
    transient_fault_rejections: int
    transient_fault_measurements: int
    transient_fault_rejection_rate: float
    position_recovery_time_s: float | None
    heading_recovery_time_s: float | None
    final_estimated_gyro_bias_rad_s: float


class PlanarEkf:
    """Six-state EKF for planar pose, body velocity, yaw rate, and gyro bias."""

    def __init__(
        self,
        initial_x_m: float,
        initial_y_m: float,
        initial_yaw_rad: float,
        config: EkfConfig | None = None,
    ) -> None:
        self.config = config if config is not None else EkfConfig()
        self.state = np.array(
            [initial_x_m, initial_y_m, initial_yaw_rad, 0.0, 0.0, 0.0],
            dtype=float,
        )
        self.covariance = np.diag(
            [
                self.config.initial_position_std_m**2,
                self.config.initial_position_std_m**2,
                self.config.initial_yaw_std_rad**2,
                self.config.initial_velocity_std_m_s**2,
                self.config.initial_yaw_rate_std_rad_s**2,
                self.config.initial_gyro_bias_std_rad_s**2,
            ]
        )

    def predict(self, dt_s: float, longitudinal_acceleration_m_s2: float = 0.0) -> None:
        """Propagate state and covariance with a midpoint-heading motion model."""

        if not isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be finite and positive")
        if not isfinite(longitudinal_acceleration_m_s2):
            raise ValueError("longitudinal acceleration must be finite")

        x_m, y_m, yaw_rad, velocity_m_s, yaw_rate_rad_s, gyro_bias_rad_s = self.state
        distance_m = velocity_m_s * dt_s + 0.5 * longitudinal_acceleration_m_s2 * dt_s**2
        midpoint_yaw_rad = yaw_rad + 0.5 * yaw_rate_rad_s * dt_s
        self.state = np.array(
            [
                x_m + distance_m * cos(midpoint_yaw_rad),
                y_m + distance_m * sin(midpoint_yaw_rad),
                wrap_angle(yaw_rad + yaw_rate_rad_s * dt_s),
                velocity_m_s + longitudinal_acceleration_m_s2 * dt_s,
                yaw_rate_rad_s,
                gyro_bias_rad_s,
            ],
            dtype=float,
        )

        jacobian = np.eye(STATE_SIZE)
        jacobian[POSITION_X, YAW] = -distance_m * sin(midpoint_yaw_rad)
        jacobian[POSITION_X, LINEAR_VELOCITY] = dt_s * cos(midpoint_yaw_rad)
        jacobian[POSITION_X, YAW_RATE] = -0.5 * distance_m * dt_s * sin(midpoint_yaw_rad)
        jacobian[POSITION_Y, YAW] = distance_m * cos(midpoint_yaw_rad)
        jacobian[POSITION_Y, LINEAR_VELOCITY] = dt_s * sin(midpoint_yaw_rad)
        jacobian[POSITION_Y, YAW_RATE] = 0.5 * distance_m * dt_s * cos(midpoint_yaw_rad)
        jacobian[YAW, YAW_RATE] = dt_s

        noise = self.config.noise
        process_covariance = np.diag(
            [
                noise.position_process_std_m**2 * dt_s,
                noise.position_process_std_m**2 * dt_s,
                noise.yaw_process_std_rad**2 * dt_s,
                noise.velocity_process_std_m_s**2 * dt_s,
                noise.yaw_rate_process_std_rad_s**2 * dt_s,
                noise.gyro_bias_process_std_rad_s**2 * dt_s,
            ]
        )
        self.covariance = jacobian @ self.covariance @ jacobian.T + process_covariance
        self._stabilize_covariance()

    def innovation_linear(
        self,
        measurement: float,
        observation: np.ndarray,
        variance: float,
    ) -> tuple[float, float, float]:
        """Return innovation, innovation variance, and scalar NIS."""

        if not isfinite(measurement):
            raise ValueError("measurement must be finite")
        if not isfinite(variance) or variance <= 0.0:
            raise ValueError("measurement variance must be finite and positive")
        observation = np.asarray(observation, dtype=float).reshape(1, STATE_SIZE)
        if not np.all(np.isfinite(observation)):
            raise ValueError("observation must contain finite values")
        innovation = measurement - (observation @ self.state).item()
        innovation_variance = (
            observation @ self.covariance @ observation.T + variance
        ).item()
        return innovation, innovation_variance, innovation**2 / innovation_variance

    def innovation_statistics(
        self,
        measurement: float,
        state_index: int,
        variance: float,
    ) -> tuple[float, float, float]:
        """Return innovation statistics for one state component."""

        observation = np.zeros(STATE_SIZE)
        observation[state_index] = 1.0
        return self.innovation_linear(measurement, observation, variance)

    def update_linear(
        self,
        measurement: float,
        observation: np.ndarray,
        variance: float,
        nis_threshold: float | None,
        *,
        force_accept: bool = False,
    ) -> InnovationResult:
        """Apply one scalar linear update using Joseph covariance form."""

        observation = np.asarray(observation, dtype=float).reshape(1, STATE_SIZE)
        innovation, innovation_variance, nis = self.innovation_linear(
            measurement,
            observation,
            variance,
        )
        accepted = force_accept or nis_threshold is None or nis <= nis_threshold
        if not accepted:
            return InnovationResult(innovation, innovation_variance, nis, False)

        gain = self.covariance @ observation.T / innovation_variance
        self.state += gain[:, 0] * innovation
        self.state[YAW] = wrap_angle(float(self.state[YAW]))
        residual_matrix = np.eye(STATE_SIZE) - gain @ observation
        self.covariance = (
            residual_matrix @ self.covariance @ residual_matrix.T
            + (gain @ gain.T) * variance
        )
        self._stabilize_covariance()
        if not np.all(np.isfinite(self.state)):
            raise FloatingPointError("EKF state contains non-finite values")
        return InnovationResult(innovation, innovation_variance, nis, True)

    def update_scalar(
        self,
        measurement: float,
        state_index: int,
        variance: float,
        nis_threshold: float | None,
        *,
        force_accept: bool = False,
    ) -> InnovationResult:
        """Apply a scalar update to one state component."""

        observation = np.zeros(STATE_SIZE)
        observation[state_index] = 1.0
        return self.update_linear(
            measurement,
            observation,
            variance,
            nis_threshold,
            force_accept=force_accept,
        )

    def _stabilize_covariance(self) -> None:
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        if not np.all(np.isfinite(self.covariance)):
            raise FloatingPointError("EKF covariance contains non-finite values")
        minimum_eigenvalue = float(np.min(np.linalg.eigvalsh(self.covariance)))
        if minimum_eigenvalue < -1e-9:
            raise FloatingPointError("EKF covariance is not positive semidefinite")
        if minimum_eigenvalue < 0.0:
            self.covariance += np.eye(STATE_SIZE) * (-minimum_eigenvalue + 1e-12)


def encoder_twist(
    previous: SensorSample,
    current: SensorSample,
    geometry: RobotGeometry,
) -> tuple[float, float]:
    """Derive body linear velocity and yaw rate from encoder count differences."""

    if current.dt_s <= 0.0:
        raise ValueError("current sample must have a positive dt_s")
    count_to_rad = 2.0 * pi / geometry.encoder_counts_per_revolution
    left_velocity_m_s = (
        geometry.wheel_radius_m
        * (current.left_encoder_count - previous.left_encoder_count)
        * count_to_rad
        / current.dt_s
    )
    right_velocity_m_s = (
        geometry.wheel_radius_m
        * (current.right_encoder_count - previous.right_encoder_count)
        * count_to_rad
        / current.dt_s
    )
    return (
        0.5 * (left_velocity_m_s + right_velocity_m_s),
        (right_velocity_m_s - left_velocity_m_s) / geometry.wheel_separation_m,
    )


def _angular_updates(
    ekf: PlanarEkf,
    wheel_yaw_rate: float,
    gyro_measurement: float | None,
    config: EkfConfig,
) -> tuple[InnovationResult, InnovationResult | None]:
    """Fuse or arbitrate wheel and gyroscope yaw-rate measurements."""

    noise = config.noise
    gates = config.gates
    wheel_variance = noise.wheel_yaw_rate_std_rad_s**2
    gyro_variance = noise.gyro_std_rad_s**2
    wheel_innovation, wheel_innovation_variance, wheel_nis = ekf.innovation_statistics(
        wheel_yaw_rate,
        YAW_RATE,
        wheel_variance,
    )
    if gyro_measurement is None:
        return (
            ekf.update_scalar(
                wheel_yaw_rate,
                YAW_RATE,
                wheel_variance,
                gates.wheel_yaw_rate_nis,
            ),
            None,
        )

    gyro_observation = np.zeros(STATE_SIZE)
    gyro_observation[YAW_RATE] = 1.0
    gyro_observation[GYRO_BIAS] = 1.0
    gyro_innovation, gyro_innovation_variance, gyro_nis = ekf.innovation_linear(
        gyro_measurement,
        gyro_observation,
        gyro_variance,
    )
    gating_enabled = any(
        threshold is not None
        for threshold in (
            gates.wheel_velocity_nis,
            gates.wheel_yaw_rate_nis,
            gates.gyro_nis,
        )
    )
    if not gating_enabled:
        return (
            ekf.update_scalar(wheel_yaw_rate, YAW_RATE, wheel_variance, None),
            ekf.update_linear(gyro_measurement, gyro_observation, gyro_variance, None),
        )

    sensors_agree = abs(
        wheel_yaw_rate - (gyro_measurement - float(ekf.state[GYRO_BIAS]))
    ) <= gates.angular_agreement_rad_s
    wheel_passes = gates.wheel_yaw_rate_nis is None or wheel_nis <= gates.wheel_yaw_rate_nis
    gyro_passes = gates.gyro_nis is None or gyro_nis <= gates.gyro_nis

    if sensors_agree:
        force_accept = not wheel_passes and not gyro_passes
        return (
            ekf.update_scalar(
                wheel_yaw_rate,
                YAW_RATE,
                wheel_variance,
                gates.wheel_yaw_rate_nis,
                force_accept=force_accept,
            ),
            ekf.update_linear(
                gyro_measurement,
                gyro_observation,
                gyro_variance,
                gates.gyro_nis,
                force_accept=force_accept,
            ),
        )
    if wheel_nis <= gyro_nis:
        return (
            ekf.update_scalar(
                wheel_yaw_rate,
                YAW_RATE,
                wheel_variance,
                gates.wheel_yaw_rate_nis,
                force_accept=True,
            ),
            InnovationResult(gyro_innovation, gyro_innovation_variance, gyro_nis, False),
        )
    return (
        InnovationResult(wheel_innovation, wheel_innovation_variance, wheel_nis, False),
        ekf.update_linear(
            gyro_measurement,
            gyro_observation,
            gyro_variance,
            gates.gyro_nis,
            force_accept=True,
        ),
    )


def _estimator_sample(
    ekf: PlanarEkf,
    sensor: SensorSample,
    wheel_velocity: float,
    wheel_yaw_rate: float,
    wheel_velocity_result: InnovationResult,
    wheel_yaw_result: InnovationResult,
    gyro_result: InnovationResult | None,
) -> EstimatorSample:
    """Create an immutable estimator output sample."""

    return EstimatorSample(
        time_s=sensor.time_s,
        truth_x_m=sensor.truth_x_m,
        truth_y_m=sensor.truth_y_m,
        truth_yaw_rad=sensor.truth_yaw_rad,
        estimated_x_m=float(ekf.state[POSITION_X]),
        estimated_y_m=float(ekf.state[POSITION_Y]),
        estimated_yaw_rad=float(ekf.state[YAW]),
        estimated_linear_velocity_m_s=float(ekf.state[LINEAR_VELOCITY]),
        estimated_yaw_rate_rad_s=float(ekf.state[YAW_RATE]),
        estimated_gyro_bias_rad_s=float(ekf.state[GYRO_BIAS]),
        covariance_trace=float(np.trace(ekf.covariance)),
        covariance_x_m2=float(ekf.covariance[POSITION_X, POSITION_X]),
        covariance_y_m2=float(ekf.covariance[POSITION_Y, POSITION_Y]),
        covariance_yaw_rad2=float(ekf.covariance[YAW, YAW]),
        wheel_linear_velocity_m_s=wheel_velocity,
        wheel_yaw_rate_rad_s=wheel_yaw_rate,
        imu_yaw_rate_rad_s=sensor.imu_yaw_rate_rad_s,
        imu_acceleration_m_s2=sensor.imu_longitudinal_acceleration_m_s2,
        wheel_velocity_nis=wheel_velocity_result.nis,
        wheel_yaw_rate_nis=wheel_yaw_result.nis,
        gyro_nis=None if gyro_result is None else gyro_result.nis,
        wheel_velocity_accepted=wheel_velocity_result.accepted,
        wheel_yaw_rate_accepted=wheel_yaw_result.accepted,
        gyro_accepted=False if gyro_result is None else gyro_result.accepted,
        wheel_slip_active=sensor.wheel_slip_active,
        imu_dropout_active=sensor.imu_dropout_active,
        gyro_outlier_active=sensor.gyro_outlier_active,
    )


def run_ekf(
    sensor_samples: tuple[SensorSample, ...],
    geometry: RobotGeometry,
    config: EkfConfig | None = None,
) -> tuple[EstimatorSample, ...]:
    """Run the EKF over interval-average encoder and IMU measurements."""

    if len(sensor_samples) < 2:
        raise ValueError("at least two sensor samples are required")
    resolved_config = config if config is not None else EkfConfig()
    initial = sensor_samples[0]
    ekf = PlanarEkf(initial.truth_x_m, initial.truth_y_m, initial.truth_yaw_rad, resolved_config)
    accepted = InnovationResult(0.0, 1.0, 0.0, True)
    output = [_estimator_sample(ekf, initial, 0.0, 0.0, accepted, accepted, accepted)]
    wheel_velocity_variance = resolved_config.noise.wheel_velocity_std_m_s**2

    for previous, current in zip(sensor_samples[:-1], sensor_samples[1:], strict=True):
        wheel_velocity, wheel_yaw_rate = encoder_twist(previous, current, geometry)
        velocity_innovation, velocity_variance, velocity_nis = ekf.innovation_statistics(
            wheel_velocity,
            LINEAR_VELOCITY,
            wheel_velocity_variance,
        )
        wheel_yaw_result, gyro_result = _angular_updates(
            ekf,
            wheel_yaw_rate,
            current.imu_yaw_rate_rad_s,
            resolved_config,
        )
        wheel_is_suspect = (
            gyro_result is not None
            and gyro_result.accepted
            and not wheel_yaw_result.accepted
        )
        if wheel_is_suspect:
            wheel_velocity_result = InnovationResult(
                velocity_innovation,
                velocity_variance,
                velocity_nis,
                False,
            )
        else:
            wheel_velocity_result = ekf.update_scalar(
                wheel_velocity,
                LINEAR_VELOCITY,
                wheel_velocity_variance,
                resolved_config.gates.wheel_velocity_nis,
            )

        # Measurements describe the interval ending at ``current``. Fuse the
        # interval-average rates before propagating pose across that interval.
        ekf.predict(current.dt_s)
        output.append(
            _estimator_sample(
                ekf,
                current,
                wheel_velocity,
                wheel_yaw_rate,
                wheel_velocity_result,
                wheel_yaw_result,
                gyro_result,
            )
        )
    return tuple(output)


def calculate_estimator_metrics(
    samples: tuple[EstimatorSample, ...],
    *,
    recovery_position_threshold_m: float = 0.25,
    recovery_heading_threshold_rad: float = 0.1,
) -> EstimatorMetrics:
    """Calculate accuracy, rejection, uncertainty, and recovery metrics."""

    if not samples:
        raise ValueError("samples must not be empty")
    position_errors = [
        hypot(sample.estimated_x_m - sample.truth_x_m, sample.estimated_y_m - sample.truth_y_m)
        for sample in samples
    ]
    heading_errors = [
        wrap_angle(sample.estimated_yaw_rad - sample.truth_yaw_rad)
        for sample in samples
    ]
    wheel_velocity_rejections = sum(not sample.wheel_velocity_accepted for sample in samples[1:])
    wheel_yaw_rejections = sum(not sample.wheel_yaw_rate_accepted for sample in samples[1:])
    gyro_rejections = sum(
        sample.imu_yaw_rate_rad_s is not None and not sample.gyro_accepted
        for sample in samples[1:]
    )

    false_rejections = 0
    fault_rejections = 0
    fault_measurements = 0
    last_fault_time_s = 0.0
    for sample in samples[1:]:
        fault_active = (
            sample.wheel_slip_active
            or sample.imu_dropout_active
            or sample.gyro_outlier_active
        )
        rejections = (
            int(not sample.wheel_velocity_accepted)
            + int(not sample.wheel_yaw_rate_accepted)
            + int(sample.imu_yaw_rate_rad_s is not None and not sample.gyro_accepted)
        )
        if fault_active:
            fault_rejections += rejections
            fault_measurements += 2 + int(sample.imu_yaw_rate_rad_s is not None)
            last_fault_time_s = sample.time_s
        else:
            false_rejections += rejections

    position_recovery_time_s: float | None = None
    heading_recovery_time_s: float | None = None
    for sample, position_error, heading_error in zip(
        samples,
        position_errors,
        heading_errors,
        strict=True,
    ):
        if sample.time_s < last_fault_time_s:
            continue
        elapsed_s = sample.time_s - last_fault_time_s
        if position_recovery_time_s is None and position_error <= recovery_position_threshold_m:
            position_recovery_time_s = elapsed_s
        if heading_recovery_time_s is None and abs(heading_error) <= recovery_heading_threshold_rad:
            heading_recovery_time_s = elapsed_s
        if position_recovery_time_s is not None and heading_recovery_time_s is not None:
            break

    return EstimatorMetrics(
        sample_count=len(samples),
        duration_s=samples[-1].time_s,
        position_rmse_m=sqrt(sum(error**2 for error in position_errors) / len(position_errors)),
        heading_rmse_rad=sqrt(sum(error**2 for error in heading_errors) / len(heading_errors)),
        final_position_error_m=position_errors[-1],
        final_heading_error_rad=heading_errors[-1],
        max_covariance_trace=max(sample.covariance_trace for sample in samples),
        wheel_velocity_rejections=wheel_velocity_rejections,
        wheel_yaw_rate_rejections=wheel_yaw_rejections,
        gyro_rejections=gyro_rejections,
        nominal_false_rejections=false_rejections,
        transient_fault_rejections=fault_rejections,
        transient_fault_measurements=fault_measurements,
        transient_fault_rejection_rate=(
            fault_rejections / fault_measurements if fault_measurements else 0.0
        ),
        position_recovery_time_s=position_recovery_time_s,
        heading_recovery_time_s=heading_recovery_time_s,
        final_estimated_gyro_bias_rad_s=samples[-1].estimated_gyro_bias_rad_s,
    )


def standard_ekf_config() -> EkfConfig:
    """Return an EKF configuration without innovation gating."""

    return EkfConfig()


def fault_aware_ekf_config() -> EkfConfig:
    """Return a three-sigma-style scalar-NIS gate configuration."""

    return EkfConfig(
        gates=InnovationGateConfig(
            wheel_velocity_nis=9.0,
            wheel_yaw_rate_nis=9.0,
            gyro_nis=9.0,
            angular_agreement_rad_s=0.15,
        )
    )
