#!/usr/bin/env python3
"""
Mecanum Wheel Holonomic Motion Controller
Converts Twist commands to individual wheel velocities for omnidirectional movement
"""

import rclpy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

from .base_controller import BaseController
from .utils import MecanumKinematics, clamp


class MecanumController(BaseController):
    """
    Holonomic drive controller for mecanum wheels
    
    Subscribes to: /cmd_vel (Twist)
    Publishes to: /wheel_commands (JointTrajectory)
    """
    
    def __init__(self):
        """Initialize mecanum controller"""
        super().__init__('mecanum_controller')
        
        # Robot parameters
        self.wheel_radius = 0.05
        self.chassis_length = 0.40
        self.chassis_width = 0.40
        self.max_wheel_velocity = 10.0
        
        # Load from parameters
        self.declare_parameter('wheel_radius', self.wheel_radius)
        self.declare_parameter('chassis_length', self.chassis_length)
        self.declare_parameter('chassis_width', self.chassis_width)
        self.declare_parameter('max_wheel_velocity', self.max_wheel_velocity)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.chassis_length = self.get_parameter('chassis_length').value
        self.chassis_width = self.get_parameter('chassis_width').value
        self.max_wheel_velocity = self.get_parameter('max_wheel_velocity').value
        
        # Initialize kinematics
        self.kinematics = MecanumKinematics(
            self.wheel_radius,
            self.chassis_length,
            self.chassis_width
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            self.qos_profile
        )
        
        # Publishers
        self.wheel_pub = self.create_publisher(
            JointTrajectory,
            '/wheel_commands',
            self.qos_profile
        )
        
        self.log_info(
            f'Mecanum Controller Ready\n'
            f'  Wheel Radius: {self.wheel_radius}m\n'
            f'  Chassis: {self.chassis_length}m x {self.chassis_width}m\n'
            f'  Max Wheel Velocity: {self.max_wheel_velocity} rad/s'
        )
    
    def cmd_vel_callback(self, msg):
        """
        Process velocity commands and convert to wheel velocities
        
        Args:
            msg: Twist message (linear.x, linear.y, angular.z)
        """
        vx = msg.linear.x      # Forward velocity
        vy = msg.linear.y      # Lateral velocity
        vz = msg.angular.z     # Angular velocity
        
        # Convert to wheel velocities
        wheel_vels = self.kinematics.twist_to_wheels(vx, vy, vz)
        
        # Clamp to maximum
        wheel_vels = [clamp(v, -self.max_wheel_velocity, self.max_wheel_velocity)
                      for v in wheel_vels]
        
        # Create and publish trajectory
        trajectory = self.create_wheel_trajectory(wheel_vels)
        self.wheel_pub.publish(trajectory)
        
        # Debug output
        self.log_debug(
            f'Twist: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} | '
            f'Wheels: FL={wheel_vels[0]:.2f}, FR={wheel_vels[1]:.2f}, '
            f'BL={wheel_vels[2]:.2f}, BR={wheel_vels[3]:.2f}'
        )
    
    def create_wheel_trajectory(self, velocities):
        """
        Create wheel trajectory message
        
        Args:
            velocities: [v_fl, v_fr, v_bl, v_br]
        
        Returns:
            JointTrajectory message
        """
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = [
            'wheel_fl_joint',
            'wheel_fr_joint',
            'wheel_bl_joint',
            'wheel_br_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.velocities = velocities
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 second
        
        trajectory.points.append(point)
        
        return trajectory


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    controller = MecanumController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.log_info('Shutting down mecanum controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
