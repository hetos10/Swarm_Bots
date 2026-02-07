#!/usr/bin/env python3
"""
Launch file for arm controller only
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description"""
    
    warehouse_control_dir = get_package_share_directory('warehouse_control')
    config_dir = os.path.join(warehouse_control_dir, 'config')
    arm_config = os.path.join(config_dir, 'arm_config.yaml')
    
    return LaunchDescription([
        Node(
            package='warehouse_control',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            parameters=[arm_config]
        ),
    ])
