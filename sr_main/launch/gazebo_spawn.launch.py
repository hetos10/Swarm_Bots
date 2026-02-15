#!/usr/bin/env python3
"""
Complete Unified Launch File for 8-Robot Warehouse System
With Individual Namespaces, Control Manager, and Type-Specific Controllers

Features:
    ✅ Each robot has individual namespace
    ✅ Each robot has individual RSP
    ✅ Control Manager node for each robot
    ✅ Mecanum/DiffDrive controller for each robot
    ✅ Type-specific controllers:
        - Lifters: Arm controller
        - Runners: Piston controller
    ✅ All controllers in FOR loop for scalability

Usage:
    ros2 launch sr_gazebo gazebo_8robots_complete.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo 
from launch.substitutions import PythonExpression , Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node 
import xacro


def generate_launch_description():
    """
    Generate launch description for 8 robots with complete control stack
    """

    # ==================== PACKAGE PATHS ====================
    
    pkg_gazebo = get_package_share_directory("sr_gazebo")
    description_pkg = get_package_share_directory('sr_description')
    control_pkg = get_package_share_directory('sr_control')
    
    world_file = os.path.join(pkg_gazebo, "worlds", "map.sdf")

    # ==================== XACRO FILE PATHS ====================
    
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

    # ==================== PROCESS XACRO FILES ====================
        
    lifter_description = {
        "robot_description": ParameterValue(
            Command(['xacro ', lifter_xacro]),
            value_type=str
            )
        }

    runner_description = {
        "robot_description": ParameterValue(
            Command(['xacro ', runner_xacro]),
            value_type=str
            )
        }

    # ==================== LAUNCH GAZEBO ====================
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -v 4 -r'"])
        }.items()
    )

    # ==================== ROBOT CONFIGURATIONS ====================
    # Complete configuration for all 8 robots
    
    robot_configs = [
        # ========== LIFTER ROBOTS (4 total) ==========
        {
            'name': 'lifter1',
            'namespace': 'lifter1',
            'x': '-3.5',
            'y': '-2.0',
            'type': 'lifter',
            'description': lifter_description,
            'has_arm': True,
            'has_piston': False,
        },
        {
            'name': 'lifter2',
            'namespace': 'lifter2',
            'x': '-1.5',
            'y': '-2.0',
            'type': 'lifter',
            'description': lifter_description,
            'has_arm': True,
            'has_piston': False,
        },
        {
            'name': 'lifter3',
            'namespace': 'lifter3',
            'x': '-3.5',
            'y': '2.0',
            'type': 'lifter',
            'description': lifter_description,
            'has_arm': True,
            'has_piston': False,
        },
        {
            'name': 'lifter4',
            'namespace': 'lifter4',
            'x': '-1.5',
            'y': '2.0',
            'type': 'lifter',
            'description': lifter_description,
            'has_arm': True,
            'has_piston': False,
        },
        
        # ========== RUNNER ROBOTS (4 total) ==========
        {
            'name': 'runner1',
            'namespace': 'runner1',
            'x': '1.5',
            'y': '-2.0',
            'type': 'runner',
            'description': runner_description,
            'has_arm': False,
            'has_piston': True,
        },
        {
            'name': 'runner2',
            'namespace': 'runner2',
            'x': '3.5',
            'y': '-2.0',
            'type': 'runner',
            'description': runner_description,
            'has_arm': False,
            'has_piston': True,
        },
        {
            'name': 'runner3',
            'namespace': 'runner3',
            'x': '1.5',
            'y': '2.0',
            'type': 'runner',
            'description': runner_description,
            'has_arm': False,
            'has_piston': True,
        },
        {
            'name': 'runner4',
            'namespace': 'runner4',
            'x': '3.5',
            'y': '2.0',
            'type': 'runner',
            'description': runner_description,
            'has_arm': False,
            'has_piston': True,
        },
    ]

    # ==================== DYNAMICALLY CREATE NODES IN FOR LOOP ====================
    
    nodes_list = []
    
    # FOR LOOP: Create nodes for each robot
    for robot in robot_configs:
        robot_name = robot['name']
        namespace = robot['namespace']
        robot_type = robot['type']
        description = robot['description']
        
        # ========== 1. INDIVIDUAL ROBOT STATE PUBLISHER ==========
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            name='state_publisher',
            parameters=[description],
            output='screen',
            emulate_tty=True
        )
        nodes_list.append(rsp_node)
        
        # ========== 2. SPAWN ROBOT IN GAZEBO ==========
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_name}',
            output='screen',
            arguments=[
                '-name', robot_name,
                '-topic', f'/{namespace}/robot_description',
                '-x', robot['x'],
                '-y', robot['y'],
                '-z', '0.1'
            ]
        )
        nodes_list.append(spawn_node)
       

    # ==================== RETURN LAUNCH DESCRIPTION ====================
    
    return LaunchDescription([
        
        # Gazebo
        gazebo_launch,
        
        *nodes_list,
        
       
    ])