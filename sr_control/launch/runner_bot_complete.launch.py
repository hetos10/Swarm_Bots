#!/usr/bin/env python3
"""
Launch file for complete runner bot control
Launches all three controllers: mecanum, arm, and piston
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description"""
    
    # Get package share directory
    warehouse_control_dir = get_package_share_directory('warehouse_control')
    config_dir = os.path.join(warehouse_control_dir, 'config')
    
    # Declare launch arguments
    mecanum_config = os.path.join(config_dir, 'mecanum_config.yaml')
    arm_config = os.path.join(config_dir, 'arm_config.yaml')
    piston_config = os.path.join(config_dir, 'piston_config.yaml')
    
    return LaunchDescription([
        # Mecanum wheel controller
        Node(
            package='warehouse_control',
            executable='mecanum_controller',
            name='mecanum_controller',
            output='screen',
            parameters=[mecanum_config]
        ),
        
        # Arm controller
        Node(
            package='warehouse_control',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            parameters=[arm_config]
        ),
        
        # Piston controller
        Node(
            package='warehouse_control',
            executable='piston_controller',
            name='piston_controller',
            output='screen',
            parameters=[piston_config]
        ),
    ])
