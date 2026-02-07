#!/usr/bin/env python3
"""
Arm Joint Trajectory Controller
Controls manipulator arm with forward/inverse kinematics
"""

import rclpy
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration
import math

from .base_controller import BaseController
from .utils import ArmKinematics, clamp


class ArmController(BaseController):
    """
    Robotic arm controller with IK/FK support
    
    Subscribes to: /arm_target (Point)
    Publishes to: /arm_trajectory (JointTrajectory)
    """
    
    def __init__(self):
        """Initialize arm controller"""
        super().__init__('arm_controller')
        
        # Arm parameters
        self.upper_arm_length = 0.15
        self.forearm_length = 0.15
        self.trajectory_duration = 2.0
        
        # Load from parameters
        self.declare_parameter('upper_arm_length', self.upper_arm_length)
        self.declare_parameter('forearm_length', self.forearm_length)
        self.declare_parameter('trajectory_duration', self.trajectory_duration)
        
        self.upper_arm_length = self.get_parameter('upper_arm_length').value
        self.forearm_length = self.get_parameter('forearm_length').value
        self.trajectory_duration = self.get_parameter('trajectory_duration').value
        
        # Initialize kinematics
        self.kinematics = ArmKinematics(self.upper_arm_length, self.forearm_length)
        
        # Joint names
        self.joint_names = ['shoulder_joint', 'elbow_joint', 'wrist_joint']
        
        # Current state
        self.current_angles = [0.0, 0.0, 0.0]
        
        # Subscribers
        self.arm_target_sub = self.create_subscription(
            Point,
            '/arm_target',
            self.arm_target_callback,
            self.qos_profile
        )
        
        # Publishers
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_trajectory',
            self.qos_profile
        )
        
        self.log_info(
            f'Arm Controller Ready\n'
            f'  Upper Arm: {self.upper_arm_length}m\n'
            f'  Forearm: {self.forearm_length}m\n'
            f'  Max Reach: {self.kinematics.max_reach:.3f}m'
        )
    
    def arm_target_callback(self, msg):
        """
        Process target position and calculate IK
        
        Args:
            msg: Point message with target x, z
        """
        x = msg.x
        z = msg.z
        
        # Calculate IK
        theta1, theta2 = self.kinematics.inverse_kinematics(x, z)
        
        if theta1 is None:
            self.log_warn(f'Target unreachable: ({x:.3f}, {z:.3f})')
            return
        
        # Add wrist angle (set to zero for now)
        theta3 = 0.0
        
        # Clamp angles
        theta1 = clamp(theta1, -math.pi, math.pi)
        theta2 = clamp(theta2, -math.pi, math.pi)
        theta3 = clamp(theta3, -math.pi, math.pi)
        
        # Send trajectory
        self.send_trajectory([theta1, theta2, theta3])
        
        self.log_info(f'IK: ({x:.3f}, {z:.3f}) -> '
                     f'({theta1:.3f}, {theta2:.3f}, {theta3:.3f})')
    
    def send_trajectory(self, angles, duration=None):
        """
        Send trajectory command to arm
        
        Args:
            angles: [theta1, theta2, theta3]
            duration: Time to reach target (seconds)
        """
        if duration is None:
            duration = self.trajectory_duration
        
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = angles
        point.velocities = [0.0] * len(angles)
        point.time_from_start = Duration(seconds=duration).to_msg()
        
        trajectory.points.append(point)
        
        self.arm_pub.publish(trajectory)
        self.current_angles = angles


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    controller = ArmController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.log_info('Shutting down arm controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
