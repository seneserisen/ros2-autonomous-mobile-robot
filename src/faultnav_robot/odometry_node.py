"""ROS 2 node that integrates commanded velocity into planar odometry."""

from __future__ import annotations

from math import cos, sin

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from faultnav_robot.differential_drive import RobotState, integrate_twist


class CommandOdometryNode(Node):
    """Publish deterministic odometry from ``cmd_vel`` commands.

    This is an intentionally simple software-in-the-loop plant model. It is useful for learning
    ROS 2 topics, parameters, coordinate frames, timing, and testable robot kinematics before a
    physics simulator or hardware driver is introduced.
    """

    def __init__(self) -> None:
        super().__init__("command_odometry")

        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self._command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        if update_rate_hz <= 0.0:
            raise ValueError("update_rate_hz must be positive")
        if self._command_timeout_s < 0.0:
            raise ValueError("command_timeout_s must be non-negative")

        self._state = RobotState()
        self._linear_velocity_m_s = 0.0
        self._angular_velocity_rad_s = 0.0
        self._last_command_ns: int | None = None
        self._last_update_ns = self.get_clock().now().nanoseconds

        self._odometry_publisher = self.create_publisher(Odometry, "odom", 10)
        self._command_subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self._command_callback,
            10,
        )
        self._transform_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(1.0 / update_rate_hz, self._update)

        self.get_logger().info(
            f"command odometry started at {update_rate_hz:.1f} Hz "
            f"with timeout {self._command_timeout_s:.2f} s"
        )

    def _command_callback(self, message: Twist) -> None:
        self._linear_velocity_m_s = float(message.linear.x)
        self._angular_velocity_rad_s = float(message.angular.z)
        self._last_command_ns = self.get_clock().now().nanoseconds

    def _update(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        dt_s = (now_ns - self._last_update_ns) / 1_000_000_000.0
        self._last_update_ns = now_ns
        if dt_s <= 0.0:
            return

        linear_velocity_m_s = self._linear_velocity_m_s
        angular_velocity_rad_s = self._angular_velocity_rad_s
        if self._command_is_stale(now_ns):
            linear_velocity_m_s = 0.0
            angular_velocity_rad_s = 0.0

        self._state = integrate_twist(
            self._state,
            linear_velocity_m_s,
            angular_velocity_rad_s,
            dt_s,
        )

        half_yaw = 0.5 * self._state.yaw_rad
        quaternion_z = sin(half_yaw)
        quaternion_w = cos(half_yaw)
        stamp = now.to_msg()

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self._odom_frame
        odometry.child_frame_id = self._base_frame
        odometry.pose.pose.position.x = self._state.x_m
        odometry.pose.pose.position.y = self._state.y_m
        odometry.pose.pose.orientation.z = quaternion_z
        odometry.pose.pose.orientation.w = quaternion_w
        odometry.twist.twist.linear.x = linear_velocity_m_s
        odometry.twist.twist.angular.z = angular_velocity_rad_s
        odometry.pose.covariance[0] = 0.02
        odometry.pose.covariance[7] = 0.02
        odometry.pose.covariance[35] = 0.04
        self._odometry_publisher.publish(odometry)

        if self._publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._odom_frame
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = self._state.x_m
            transform.transform.translation.y = self._state.y_m
            transform.transform.rotation.z = quaternion_z
            transform.transform.rotation.w = quaternion_w
            self._transform_broadcaster.sendTransform(transform)

    def _command_is_stale(self, now_ns: int) -> bool:
        if self._last_command_ns is None:
            return True
        command_age_s = (now_ns - self._last_command_ns) / 1_000_000_000.0
        return command_age_s > self._command_timeout_s


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CommandOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
