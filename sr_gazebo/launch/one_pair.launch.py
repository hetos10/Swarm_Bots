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

    # Package paths

    pkg_gazebo = get_package_share_directory("sr_gazebo")
    description_pkg = get_package_share_directory('sr_description')
    world_file = os.path.join(pkg_gazebo, "worlds", "map.sdf")

    lifter_xacro = os.path.join(
        description_pkg,
        'urdf',
        'lifter_bot.urdf.xacro'
    )

    runner_xacro = os.path.join(
        description_pkg,
        'urdf',
        'runner_bot.urdf.xacro'
    )

    lifter_desc = xacro.process_file(lifter_xacro)
    lifter_description = {"robot_description": lifter_desc.toxml()}

    runner_desc = xacro.process_file(runner_xacro)
    runner_description = {"robot_description": runner_desc.toxml()}

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_file, " -v 4 -r'"])
                }.items()
             )
    # Spawn lifter
       # Robot state publishers
    lifter_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='lifter',
        parameters=[lifter_description],
        output='screen'
    )

    runner_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='runner',
        parameters=[runner_description],
        output='screen'
    )

    # Spawn lifter
    spawn_lifter = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_lifter',
        output='screen',
        arguments=[
            '-name', 'lifter1',
            '-topic', '/lifter/robot_description',
            '-x', '-4',
            '-y', '4',
            '-z', '0.1'
            
        ]
    )

    # Spawn runner
    spawn_runner = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_runner',
        output='screen',
        arguments=[
            '-name', 'runner1',
            '-topic', '/runner/robot_description',
            '-x', '-4',
            '-y', '-4',
            '-z', '0.1'
        
        ]
    )


    return LaunchDescription([
        gazebo,
        lifter_rsp,
        runner_rsp,
        spawn_lifter,
        spawn_runner
    ])
