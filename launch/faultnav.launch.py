from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("faultnav_robot"))
    parameters = package_share / "config" / "faultnav.yaml"

    return LaunchDescription(
        [
            Node(
                package="faultnav_robot",
                executable="command_odometry",
                name="command_odometry",
                output="screen",
                parameters=[str(parameters)],
            )
        ]
    )
