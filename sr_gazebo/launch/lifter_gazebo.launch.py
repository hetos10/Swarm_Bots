import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():

    # Get paths
    pkg_desc = get_package_share_directory("sr_description")
    pkg_gazebo = get_package_share_directory("sr_gazebo")

    xacro_file = os.path.join(pkg_desc, "urdf", "lifter_bot.urdf.xacro")
    world_file = os.path.join(pkg_gazebo, "worlds", "map.sdf")

    # Process xacro
    robot_desc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_desc.toxml()}

  
    return LaunchDescription([

        # Start Gazebo with world
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_file, " -v 4 -r'"])
                }.items()
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
                "-name", "lifter",
            ],
            output="screen"
        ),
    ])
