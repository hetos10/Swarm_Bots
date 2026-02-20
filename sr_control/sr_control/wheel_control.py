#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
import math

class MecanumWheelController(Node):
    def __init__(self):
        super().__init__('wheel_control')
        
        # ============================================
        # ROBOT CONFIGURATION
        # ============================================
        self.robot_name = "lifter1"
        self.wheel_radius = 0.05  
        # L + W in the kinematic formula
        self.l_plus_w = (0.44 / 2.0) + (0.36 / 2.0) 
        
        # ============================================
        # CONTROL PARAMETERS
        # ============================================
        self.kp_linear = 0.8  
        self.kp_angular = 1.0 
        self.target_x = 2.0
        self.target_y = 2.0
        self.target_theta = 0.0
        
        # ============================================
        # ROBOT STATE
        # ============================================
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False
        
        # ============================================
        # PUBLISHERS (Updated to match your Bridge)
        # ============================================
        self.wheel_joints = ['fl', 'fr', 'bl', 'br']
        self.wheel_pubs = {
            side: self.create_publisher(Float64, f'/{self.robot_name}/cmd_wheel_{side}', 10)
            for side in self.wheel_joints
        }
        
        # ============================================
        # SUBSCRIBERS
        # ============================================
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )

        # Control loop at 20Hz for smoother motion
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f'🚀 Mecanum Controller for {self.robot_name} started!')

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Manual Quaternion to Euler (Yaw) to avoid tf_transformations dependency issues
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.odom_received = True

    def control_loop(self):
        if not self.odom_received:
            return

        # 1. Calculate Errors
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        
        distance = math.sqrt(error_x**2 + error_y**2)
        
        # 2. Global to Local Coordinate Transformation
        # Mecanum needs velocities relative to the ROBOT frame, not the WORLD frame
        cos_theta = math.cos(self.current_theta)
        sin_theta = math.sin(self.current_theta)
        
        v_world_x = self.kp_linear * error_x
        v_world_y = self.kp_linear * error_y
        
        v_x = v_world_x * cos_theta + v_world_y * sin_theta
        v_y = -v_world_x * sin_theta + v_world_y * cos_theta
        w_z = 0.0 # Keeping orientation stable for now

        # 3. Inverse Kinematics
        # Using standard Mecanum equations
        w_fl = (v_x - v_y - self.l_plus_w * w_z) / self.wheel_radius
        w_fr = (v_x + v_y + self.l_plus_w * w_z) / self.wheel_radius
        w_bl = (v_x + v_y - self.l_plus_w * w_z) / self.wheel_radius
        w_br = (v_x - v_y + self.l_plus_w * w_z) / self.wheel_radius

        # 4. Publish or Stop
        if distance < 0.05:
            self.get_logger().info('✅ Target Reached!')
            self.stop_robot()
            self.timer.cancel() # Stop the loop
        else:
            self.publish_speeds([w_fl, w_fr, w_bl, w_br])

    def publish_speeds(self, speeds):
        for i, side in enumerate(self.wheel_joints):
            msg = Float64()
            msg.data = float(speeds[i])
            self.wheel_pubs[side].publish(msg)

    def stop_robot(self):
        for side in self.wheel_joints:
            self.wheel_pubs[side].publish(Float64(data=0.0))

def main(args=None):
    rclpy.init(args=args)
    node = MecanumWheelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()