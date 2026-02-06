import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    description_pkg = get_package_share_directory("sr_description")

    xacro_file = os.path.join(
        description_pkg,
        "urdf",
        "robot.urdf.xacro"
    )

    robot_description = Command(["xacro ", xacro_file])

    # Robot state publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # Start Gazebo Fortress
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", "empty.sdf"],
        output="screen",
    )

    # Spawn robot
    spawn = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-name", "swarm_robot",
            "-topic", "robot_description"
        ],
        output="screen",
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn,
    ])
