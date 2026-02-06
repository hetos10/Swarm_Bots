from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    # Get paths
    pkg_desc = get_package_share_directory("sr_description")
    pkg_gazebo = get_package_share_directory("sr_gazebo")

    xacro_file = os.path.join(pkg_desc, "urdf", "runner_bot.urdf.xacro")
    world_file = os.path.join(pkg_gazebo, "worlds", "empty.sdf")

    # Process xacro
    robot_desc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_desc.toxml()}

    return LaunchDescription([

        # Start Gazebo with world
        ExecuteProcess(
            cmd=["gz", "sim", world_file],
            output="screen"
        ),

        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen"
        ),

        # Spawn robot
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic", "robot_description",
                "-name", "lifter"
            ],
            output="screen"
        ),
    ])
