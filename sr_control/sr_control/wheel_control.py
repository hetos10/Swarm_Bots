#!/usr/bin/env python3
"""
Warehouse Robot Mecanum Wheel Controller
Simple and Clean: Odometry → Proportional Controller → cmd_vel
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
from tf_transformations import euler_from_quaternion

class MecanumWheelController(Node):
    def __init__(self):
        super().__init__('wheel_control')
        
        # Robot configuration
        self.robot_name = "lifter1"
        
        # Control parameters
        self.kp_linear = 0.5  # Proportional gain for linear velocity
        self.kp_angular = 0.5  # Proportional gain for angular velocity
        
        # Velocity limits
        self.max_linear_velocity = 1.0  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        
        # Fixed target pose (box location)
        self.target_x = 2.0
        self.target_y = 2.0
        self.target_theta = 0.0
        
        # Current robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info(f'✓ Subscribed to /{self.robot_name}/odom')
        
        # Publish cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            10
        )
        self.get_logger().info(f'✓ Publishing to /{self.robot_name}/cmd_vel')
        
        # Control loop timer (10 Hz)
        self.control_loop_timer = self.create_timer(0.1, self.control_loop_callback)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Mecanum Wheel Controller Initialized!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Robot: {self.robot_name}')
        self.get_logger().info(f'Target: ({self.target_x}, {self.target_y})')
        self.get_logger().info(f'Kp_linear: {self.kp_linear}, Kp_angular: {self.kp_angular}')
        self.get_logger().info('='*60)
    
    def odom_callback(self, msg: Odometry):
        """
        Update robot state from odometry
        Input: nav_msgs/Odometry from Gazebo
        """
        # Get position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Get orientation (convert quaternion to yaw)
        q = msg.pose.pose.orientation
        _, _, self.current_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.odom_received = True
    
    def calculate_error(self):
        """
        Calculate position and orientation error to target
        
        Returns:
            error_x: position error in x
            error_y: position error in y
            error_theta: orientation error (yaw)
            distance: distance to target
        """
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        error_theta = self.target_theta - self.current_theta
        
        # Normalize angle to [-pi, pi]
        while error_theta > math.pi:
            error_theta -= 2 * math.pi
        while error_theta < -math.pi:
            error_theta += 2 * math.pi
        
        distance = math.sqrt(error_x**2 + error_y**2)
        
        return error_x, error_y, error_theta, distance
    
    def proportional_controller(self):
        """
        Proportional controller to generate desired velocity
        
        Input: Current position and target position
        Output: Desired velocity (v_x, v_y, w_z)
        
        Control law: v = Kp × error
        """
        error_x, error_y, error_theta, distance = self.calculate_error()
        
        # Stop if close to target
        tolerance = 0.1  # 10 cm
        if distance < tolerance:
            return 0.0, 0.0, 0.0
        
        # Calculate desired velocities using proportional control
        v_x = self.kp_linear * error_x
        v_y = self.kp_linear * error_y
        w_z = self.kp_angular * error_theta
        
        # Clamp velocities to limits
        v_x = max(-self.max_linear_velocity, min(self.max_linear_velocity, v_x))
        v_y = max(-self.max_linear_velocity, min(self.max_linear_velocity, v_y))
        w_z = max(-self.max_angular_velocity, min(self.max_angular_velocity, w_z))
        
        return v_x, v_y, w_z
    
    def publish_cmd_vel(self, v_x, v_y, w_z):
        """
        Publish velocity command to Gazebo
        
        Input: v_x (m/s), v_y (m/s), w_z (rad/s)
        Output: geometry_msgs/Twist to cmd_vel topic
        
        The Mecanum Drive plugin in Gazebo will:
        1. Receive this Twist message
        2. Internally calculate wheel velocities using Mecanum kinematics
        3. Apply velocities to wheel joints
        4. Simulate physics and movement
        """
        msg = Twist()
        msg.linear.x = v_x
        msg.linear.y = v_y
        msg.angular.z = w_z
        self.cmd_vel_pub.publish(msg)
    
    def control_loop_callback(self):
        """
        Main control loop (runs at 10 Hz = 0.1 second intervals)
        
        Algorithm:
            1. Get current position from odometry
            2. Calculate error to target
            3. Proportional controller → desired velocity
            4. Publish velocity to cmd_vel
        """
        
        if not self.odom_received:
            self.get_logger().warn('Waiting for odometry data...')
            return
        
        # Step 1: Calculate error to target
        error_x, error_y, error_theta, distance = self.calculate_error()
        
        # Step 2: Proportional controller
        v_x, v_y, w_z = self.proportional_controller()
        
        # Step 3: Publish cmd_vel
        self.publish_cmd_vel(v_x, v_y, w_z)
        
        # Log status
        self.get_logger().info(
            f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | '
            f'Dist: {distance:.2f}m | '
            f'Vel: (vx={v_x:.2f}, vy={v_y:.2f}, wz={w_z:.2f})'
        )
        
        # If reached target
        if distance < 0.1:
            self.get_logger().info('='*60)
            self.get_logger().info('🎉 TARGET REACHED! 🎉')
            self.get_logger().info('='*60)
            # Stop robot
            self.publish_cmd_vel(0.0, 0.0, 0.0)
    
    def set_target(self, x, y, theta=0.0):
        """Set new target pose"""
        self.target_x = x
        self.target_y = y
        self.target_theta = theta
        self.get_logger().info(f'Target set to: ({x}, {y}, {theta})')

def main(args=None):
    rclpy.init(args=args)
    
    controller = MecanumWheelController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Control stopped by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()