#!/usr/bin/env python3
"""
Unified Controller Launch File for ALL 8 Robots
Launches Lifter + Runner controllers for all 4 lifters and 4 runners in ONE command

Usage:
    ros2 launch sr_control all_robots_controllers.launch.py

Features:
    ✅ Launches Lifter Controller for all 4 lifters in one command
    ✅ Launches Arm Controller for all 4 lifters in one command
    ✅ Launches Runner Controller for all 4 runners in one command
    ✅ Launches Piston Controller for all 4 runners in one command
    ✅ Individual namespace for each robot
    ✅ FOR LOOP for scalability
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo

from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for ALL 8 robot controllers
    """

    pkg_gazebo = get_package_share_directory("sr_gazebo")
    description_pkg = get_package_share_directory('sr_description')
    control_pkg = get_package_share_directory('sr_control')
    
    world_file = os.path.join(pkg_gazebo, "worlds", "map.sdf")
    # ==================== ROBOT CONFIGURATIONS ====================
    
    robot_configs = [
        # ========== LIFTER ROBOTS (4 total) ==========
        {
            'name': 'lifter1',
            'namespace': 'lifter1',
            'type': 'lifter',
            'has_arm': True,
            'has_piston': False,
        },
        {
            'name': 'lifter2',
            'namespace': 'lifter2',
            'type': 'lifter',
            'has_arm': True,
            'has_piston': False,
        },
        {
            'name': 'lifter3',
            'namespace': 'lifter3',
            'type': 'lifter',
            'has_arm': True,
            'has_piston': False,
        },
        {
            'name': 'lifter4',
            'namespace': 'lifter4',
            'type': 'lifter',
            'has_arm': True,
            'has_piston': False,
        },
        
        # ========== RUNNER ROBOTS (4 total) ==========
        {
            'name': 'runner1',
            'namespace': 'runner1',
            'type': 'runner',
            'has_arm': False,
            'has_piston': True,
        },
        {
            'name': 'runner2',
            'namespace': 'runner2',
            'type': 'runner',
            'has_arm': False,
            'has_piston': True,
        },
        {
            'name': 'runner3',
            'namespace': 'runner3',
            'type': 'runner',
            'has_arm': False,
            'has_piston': True,
        },
        {
            'name': 'runner4',
            'namespace': 'runner4',
            'type': 'runner',
            'has_arm': False,
            'has_piston': True,
        },
    ]

    # ==================== DYNAMICALLY CREATE CONTROLLER NODES IN FOR LOOP ====================
    
    nodes_list = []
    
    # FOR LOOP: Create controller nodes for each robot
    for robot in robot_configs:
        robot_name = robot['name']
        namespace = robot['namespace']
        robot_type = robot['type']

        # ========== 1. CONTROL MANAGER NODE ==========
        # Loads controller configuration for this robot
        controller_config = os.path.join(
            control_pkg,
            'config',
            f'{robot_type}_config.yaml'
        )
        
        control_manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            name='controller_manager',
            parameters=[controller_config],
            output='screen',
            emulate_tty=True
        )
        nodes_list.append(control_manager)
        
        # ========== 2. JOINT STATE BROADCASTER ==========
        # Publishes joint states for each robot
        joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            name='joint_state_broadcaster_spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', f'/{namespace}/controller_manager'
            ],
            output='screen',
            emulate_tty=True
        )
        nodes_list.append(joint_state_broadcaster)

        # ========== 3. MECANUM CONTROLLER ==========
        # Different controller based on robot type
        mecanum_controller = Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                name='mecanum_controller_spawner',
                arguments=[
                    'mecanum_controller',
                    '--controller-manager', f'/{namespace}/controller_manager'
                ],
                output='screen',
                emulate_tty=True
            )
        
        nodes_list.append(mecanum_controller)
        
        # ========== 4. TYPE-SPECIFIC CONTROLLER ==========
        # ARM for Lifters, PISTON for Runners
        
        if robot['has_arm']:  # Lifter with ARM
            arm_controller = Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                name='arm_controller_spawner',
                arguments=[
                    'arm_controller',
                    '--controller-manager', f'/{namespace}/controller_manager'
                ],
                output='screen',
                emulate_tty=True
            )
            nodes_list.append(arm_controller)
        
        if robot['has_piston']:  # Runner with PISTON
            piston_controller = Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                name='piston_controller_spawner',
                arguments=[
                    'piston_controller',
                    '--controller-manager', f'/{namespace}/controller_manager'
                ],
                output='screen',
                emulate_tty=True
            )
            nodes_list.append(piston_controller)
        
        
    # ==================== RETURN LAUNCH DESCRIPTION ====================
    
    return LaunchDescription([
        # Info messages
        # All controller nodes created in FOR loop
        *nodes_list,
        
    ])