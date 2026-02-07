#!/usr/bin/env python3
"""
Piston Linear Actuator Controller
Controls piston extension/retraction for box pushing
"""

import rclpy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration

from .base_controller import BaseController
from .utils import clamp


class PistonController(BaseController):
    """
    Piston linear actuator controller
    
    Subscribes to: /piston_target (Float64)
    Publishes to: /piston_trajectory (JointTrajectory)
    """
    
    def __init__(self):
        """Initialize piston controller"""
        super().__init__('piston_controller')
        
        # Piston parameters
        self.min_position = -0.15   # Fully retracted (inside wall)
        self.max_position = 0.25    # Fully extended (to front)
        self.max_velocity = 0.5     # m/s
        self.trajectory_duration = 2.0
        
        # Load from parameters
        self.declare_parameter('min_position', self.min_position)
        self.declare_parameter('max_position', self.max_position)
        self.declare_parameter('max_velocity', self.max_velocity)
        self.declare_parameter('trajectory_duration', self.trajectory_duration)
        
        self.min_position = self.get_parameter('min_position').value
        self.max_position = self.get_parameter('max_position').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.trajectory_duration = self.get_parameter('trajectory_duration').value
        
        # Current state
        self.current_position = self.min_position
        
        # Subscribers
        self.piston_target_sub = self.create_subscription(
            Float64,
            '/piston_target',
            self.piston_target_callback,
            self.qos_profile
        )
        
        # Publishers
        self.piston_pub = self.create_publisher(
            JointTrajectory,
            '/piston_trajectory',
            self.qos_profile
        )
        
        self.log_info(
            f'Piston Controller Ready\n'
            f'  Range: [{self.min_position:.3f}, {self.max_position:.3f}]m\n'
            f'  Max Velocity: {self.max_velocity}m/s'
        )
    
    def piston_target_callback(self, msg):
        """
        Process target piston position
        
        Args:
            msg: Float64 message with normalized position (0.0 to 1.0)
        """
        # Normalize input (0.0 to 1.0)
        normalized = clamp(msg.data, 0.0, 1.0)
        
        # Convert to actual position
        target = self.min_position + normalized * (self.max_position - self.min_position)
        
        self.extend_piston(target)
    
    def extend_piston(self, target_position, duration=None):
        """
        Extend/retract piston to target position
        
        Args:
            target_position: Target position in meters
            duration: Time to reach target (seconds)
        """
        # Clamp position
        target_position = clamp(target_position, self.min_position, self.max_position)
        
        # Calculate duration based on velocity
        if duration is None:
            distance = abs(target_position - self.current_position)
            calc_duration = distance / self.max_velocity
            duration = max(self.trajectory_duration, calc_duration)
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = ['piston_rod_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [target_position]
        point.velocities = [0.0]
        point.time_from_start = Duration(seconds=duration).to_msg()
        
        trajectory.points.append(point)
        
        # Publish
        self.piston_pub.publish(trajectory)
        self.current_position = target_position
        
        # Calculate percentage
        percentage = (
            (target_position - self.min_position) /
            (self.max_position - self.min_position) * 100
        )
        
        self.log_info(
            f'Piston: pos={target_position:.3f}m ({percentage:.0f}%), '
            f'duration={duration:.2f}s'
        )
    
    def retract(self):
        """Fully retract piston"""
        self.extend_piston(self.min_position, 1.0)
    
    def extend(self):
        """Fully extend piston"""
        self.extend_piston(self.max_position, 2.0)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    controller = PistonController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.log_info('Shutting down piston controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
