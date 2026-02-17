import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    sr_gazebo_pkg= os.path.join(get_package_share_directory('sr_gazebo'))
    sr_description_pkg= os.path.join(get_package_share_directory('sr_description'))

    xacro_file = os.path.join(sr_description_pkg,'urdf','runner_bot.urdf.xacro')
    rviz_config_file = os.path.join(sr_description_pkg,'rviz','runner.rviz')
    world_path = os.path.join(sr_gazebo_pkg, 'worlds', 'empty.sdf')
    description_params_file= {
        "robot_description": ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str
        )
    }
    controller_params_file = os.path.join(get_package_share_directory('sr_control'),'config','runner_config.yaml')

    gazebo = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                    launch_arguments={
                        "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                    }.items()
                )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "runner1",
            "-allow_renaming",
            "true",
        ],
    )
    
    
    control_node = TimerAction(
        period=5.0,  # wait 2 seconds for robot_description to be published
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controller_params_file],
                output="both",
            )
        ]
    )
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[description_params_file],
        
    )

    joint_state_broadcaster_spawner= TimerAction(
        period=4.0,
        actions=[
            Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
            )
        ]
    )
    
    mecanum_controller_spawner = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['mecanum_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    piston_controller_spawner = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['piston_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes = [
        gazebo,
        node_robot_state_publisher, 
        gz_spawn_entity, 
        control_node,
        joint_state_broadcaster_spawner,
        mecanum_controller_spawner,
        piston_controller_spawner,
        rviz_node 
    ]

    return LaunchDescription(nodes)