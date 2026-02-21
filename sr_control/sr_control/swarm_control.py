#!/usr/bin/env python3

'''
Mecanum Wheel Controller with Ground Truth Pose Estimation
'''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
from linkattacher_msgs.srv import AttachLink, DetachLink
import time
import math
from tf_transformations import euler_from_quaternion
import json

# Import custom message
from sr_interfaces.msg import BotCmdArray , BotCmd


# -------- PID Class --------
class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        self.derivative = (error - self.prev_error) / dt
        self.output = self.kp * error + self.ki * self.integral + self.kd * self.derivative
        self.output = max(-self.max_out, min(self.output, self.max_out))
        self.prev_error = error
        return self.output
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class MecanumPIDController(Node):
    def __init__(self):
        super().__init__('mecanum_pid_controller')

        # -------- INITIAL POSE (from launch file spawn position) --------
        self.initial_x = -4.5
        self.initial_y = 4.0
        self.initial_theta = 0.0
        
        # Robot Parameters
        self.bot_id = 0
        
        # -------- ODOMETRY OFFSET --------
        # Gazebo odometry starts at (0,0), but robot is actually at (initial_x, initial_y)
        # We'll use this offset to convert odom to map frame
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        
        # Ground truth pose (odom + offset)
        self.current_x = self.initial_x
        self.current_y = self.initial_y
        self.current_theta = self.initial_theta
        
        self.last_time = self.get_clock().now()
        self.first_odom_received = False
        
        # -------- CRATE POSITION (FIXED IN MAP FRAME) --------
        self.crate_x = 4.8
        self.crate_y = 4.8
        self.crate_approach_distance = 0.5
        
        # -------- TARGET ZONE (FIXED IN MAP FRAME) --------
        self.target_zone = {
            'x_min': 3.0,
            'x_max': 6.0,
            'y_min': 3.0,
            'y_max': 6.0
        }
        self.target_center_x = (self.target_zone['x_min'] + self.target_zone['x_max']) / 2
        self.target_center_y = (self.target_zone['y_min'] + self.target_zone['y_max']) / 2
        
        # Goals in MAP frame
        self.goals = [
            (self.crate_x, self.crate_y - self.crate_approach_distance, 0),  # Approach crate
        ]
        self.goal_idx = 0
        
        self.max_vel = 1.5
        
        # Docking position (HOME - in MAP frame)
        self.dock_position = {
            'x': self.initial_x,
            'y': self.initial_y,
            'theta': self.initial_theta
        }
        
        # Arm control
        self.pickup_state = 'moving'
        self.arm_base_angle = 1.57
        self.arm_elbow_angle = 1.57
        self.state_timer = None
        
        # Service futures
        self.attach_future = None
        self.detach_future = None

        # Mecanum parameters
        self.wheel_radius = 0.1
        self.wheel_separation_x = 0.4
        self.wheel_separation_y = 0.4

        # PID Parameters
        self.pid_params = {
            'x': {'kp': 0.5, 'ki': 0.001, 'kd': 0.1, 'max_out': self.max_vel},
            'y': {'kp': 0.5, 'ki': 0.001, 'kd': 0.1, 'max_out': self.max_vel},
            'theta': {'kp': 0.5, 'ki': 0.0001, 'kd': 0.1, 'max_out': self.max_vel / 2}
        }

        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        # ROS 2 Publishers & Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/lifter1/odom',
            self.odom_callback,
            10
        )
        
        self.bot_cmd_pub = self.create_publisher(
            BotCmd,
            '/lifter1/bot_cmd',
            10
        )
        
        # Service clients
        self.attach_cli = self.create_client(AttachLink, '/attach_link')
        self.detach_cli = self.create_client(DetachLink, '/detach_link')

        # Control loop
        self.timer = self.create_timer(0.03, self.control_cb)

        self.get_logger().info('='*60)
        self.get_logger().info('Mecanum PID Controller Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Initial Pose (MAP): ({self.initial_x}, {self.initial_y}, {self.initial_theta})')
        self.get_logger().info(f'Crate Position (MAP): ({self.crate_x}, {self.crate_y})')
        self.get_logger().info(f'Target Zone Center (MAP): ({self.target_center_x}, {self.target_center_y})')
        self.get_logger().info(f'Dock Position (MAP): ({self.dock_position["x"]}, {self.dock_position["y"]})')
        self.get_logger().info('='*60)

    # -------- ODOMETRY CALLBACK --------
    def odom_callback(self, msg: Odometry):
        """
        Odometry gives pose relative to start (0,0)
        We convert to MAP frame by adding initial offset
        """
        # Get odometry pose (relative to odom frame origin)
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, self.odom_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Convert to MAP frame by adding initial pose
        self.current_x = self.odom_x + self.initial_x
        self.current_y = self.odom_y + self.initial_y
        self.current_theta = self.odom_theta + self.initial_theta
        
        if not self.first_odom_received:
            self.first_odom_received = True
            self.get_logger().info(f'First odometry received!')
            self.get_logger().info(f'Odometry (ODOM): ({self.odom_x:.2f}, {self.odom_y:.2f})')
            self.get_logger().info(f'Ground Truth (MAP): ({self.current_x:.2f}, {self.current_y:.2f})')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # -------- MECANUM KINEMATICS --------
    def mecanum_kinematics(self, vx, vy, w):
        L_x = self.wheel_separation_x / 2.0
        L_y = self.wheel_separation_y / 2.0
        R = self.wheel_radius
        
        v_FL = (vx - vy - (L_x + L_y) * w) / R
        v_FR = (vx + vy + (L_x + L_y) * w) / R
        v_BL = (vx + vy - (L_x + L_y) * w) / R
        v_BR = (vx - vy + (L_x + L_y) * w) / R
        
        return v_FL, v_FR, v_BL, v_BR

    # -------- SERVICE CALLS --------
    def call_attach_service_async(self, model1, link1, model2, link2):
        if not self.attach_cli.wait_for_service(timeout_sec=5):
            self.get_logger().warn('Attach service not available')
            return None

        req = AttachLink.Request()
        data_dict = {
            "model1_name": model1,
            "link1_name": link1, 
            "model2_name": model2,
            "link2_name": link2
        }
        req.data = json.dumps(data_dict)

        self.attach_future = self.attach_cli.call_async(req)
        self.get_logger().info(f'Attach service called')
        return self.attach_future

    def call_detach_service_async(self, model1, link1, model2, link2):
        if not self.detach_cli.wait_for_service(timeout_sec=5):
            self.get_logger().warn('Detach service not available')
            return None

        req = DetachLink.Request()
        data_dict = {
            "model1_name": model1,
            "link1_name": link1,
            "model2_name": model2, 
            "link2_name": link2
        }
        req.data = json.dumps(data_dict)

        self.detach_future = self.detach_cli.call_async(req)
        self.get_logger().info(f'Detach service called')
        return self.detach_future

    # -------- CONTROL LOOP --------
    def control_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        if not self.first_odom_received:
            return

        if self.goal_idx >= len(self.goals) and self.pickup_state == 'final_done':
            self.publish_bot_cmd([0.0, 0.0, 0.0])
            return

        x, y, theta = self.current_x, self.current_y, self.current_theta

        # STATE MACHINE
        if self.pickup_state == 'moving':
            tar_x, tar_y, tar_theta = self.goals[self.goal_idx]
            error_x = tar_x - x
            error_y = tar_y - y
            dist_to_goal = math.sqrt(error_x**2 + error_y**2)

            self.get_logger().info(f'Moving to goal: Pos=({x:.2f},{y:.2f}) Target=({tar_x:.2f},{tar_y:.2f}) Dist={dist_to_goal:.2f}m')

            if dist_to_goal < 0.1:
                self.get_logger().info(f"Goal reached!")
                self.publish_bot_cmd([0.0, 0.0, 0.0])
                self.arm_base_angle = 1.67
                self.arm_elbow_angle = 1.57
                self.pickup_state = 'lowering'
                self.state_timer = now
                self.get_logger().info("State: LOWERING")
            else:
                vx = self.pid_x.compute(error_x, dt)
                vy = self.pid_y.compute(error_y, dt)
                w = 0.0
                self.publish_bot_cmd([vx, vy, w])
        
        elif self.pickup_state == 'lowering':
            self.publish_bot_cmd([0.0, 0.0, 0.0])
            elapsed = (now - self.state_timer).nanoseconds / 1e9
            if elapsed > 1.5:
                if self.attach_cli.wait_for_service(timeout_sec=3.0):
                    future = self.call_attach_service_async("lifter1", "arm_link_2", "crate", "box_link")
                    if future is not None:
                        self.attach_future = future
                        self.pickup_state = 'attaching'
                        self.state_timer = now
                        self.get_logger().info("State: ATTACHING")
                else:
                    self.state_timer = now
        
        elif self.pickup_state == 'attaching':
            self.publish_bot_cmd([0.0, 0.0, 0.0])
            if self.attach_future is not None and self.attach_future.done():
                try:
                    result = self.attach_future.result()
                    self.get_logger().info("Crate attached! Lifting arm...")
                    self.arm_base_angle = 1.57
                    self.arm_elbow_angle = 1.57
                    self.pickup_state = 'lifting'
                    self.state_timer = now
                    self.attach_future = None
                    self.get_logger().info("State: LIFTING")
                except Exception as e:
                    self.get_logger().error(f"Attachment failed: {e}")
                    self.pickup_state = 'lowering'  
                    self.attach_future = None

        elif self.pickup_state == 'lifting':
            self.publish_bot_cmd([0.0, 0.0, 0.0])
            elapsed = (now - self.state_timer).nanoseconds / 1e9
            if elapsed > 0.5:
                self.get_logger().info("Moving to target zone...")
                self.arm_base_angle = 0.0
                self.arm_elbow_angle = 1.57
                self.pickup_state = 'moving_to_target'
                self.pid_x.reset()
                self.pid_y.reset()
                self.get_logger().info(f"State: MOVING_TO_TARGET")

        elif self.pickup_state == 'moving_to_target':
            tar_x = self.target_center_x
            tar_y = self.target_center_y
            error_x = tar_x - x
            error_y = tar_y - y
            dist_to_target = math.sqrt(error_x**2 + error_y**2)

            self.get_logger().info(f'Moving to target: Pos=({x:.2f},{y:.2f}) Target=({tar_x:.2f},{tar_y:.2f}) Dist={dist_to_target:.2f}m')

            if dist_to_target < 0.1:
                self.get_logger().info(f"Reached target zone!")
                self.publish_bot_cmd([0.0, 0.0, 0.0])
                self.arm_base_angle = 1.67
                self.arm_elbow_angle = 1.57
                self.pickup_state = 'lowering_at_target'
                self.state_timer = now
                self.get_logger().info("State: LOWERING_AT_TARGET")
            else:
                vx = self.pid_x.compute(error_x, dt)
                vy = self.pid_y.compute(error_y, dt)
                w = 0.0
                self.publish_bot_cmd([vx, vy, w])

        elif self.pickup_state == 'lowering_at_target':
            self.publish_bot_cmd([0.0, 0.0, 0.0])
            elapsed = (now - self.state_timer).nanoseconds / 1e9
            if elapsed > 1.5:
                if self.detach_cli.wait_for_service(timeout_sec=3.0):
                    future = self.call_detach_service_async("lifter1", "arm_link_2", "crate", "box_link")
                    if future is not None:
                        self.detach_future = future
                        self.pickup_state = 'detaching'
                        self.state_timer = now
                        self.get_logger().info("State: DETACHING")
                else:
                    self.state_timer = now

        elif self.pickup_state == 'detaching':
            self.publish_bot_cmd([0.0, 0.0, 0.0])
            if self.detach_future is not None and self.detach_future.done():
                try:
                    result = self.detach_future.result()
                    self.get_logger().info("Crate detached! Lifting arm...")
                    self.arm_base_angle = 1.57
                    self.arm_elbow_angle = 1.57
                    self.pickup_state = 'lifting_after_detach'
                    self.state_timer = now
                    self.detach_future = None
                    self.get_logger().info("State: LIFTING_AFTER_DETACH")
                except Exception as e:
                    self.get_logger().error(f"Detachment failed: {e}")
                    self.pickup_state = 'lowering_at_target'
                    self.detach_future = None

        elif self.pickup_state == 'lifting_after_detach':
            self.publish_bot_cmd([0.0, 0.0, 0.0])
            elapsed = (now - self.state_timer).nanoseconds / 1e9
            if elapsed > 1.0:
                self.get_logger().info("Returning to dock...")
                self.pickup_state = 'returning_to_dock'
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.get_logger().info(f"State: RETURNING_TO_DOCK")

        elif self.pickup_state == 'returning_to_dock':
            tar_x = self.dock_position['x']
            tar_y = self.dock_position['y']
            error_x = tar_x - x
            error_y = tar_y - y
            dist_to_dock = math.sqrt(error_x**2 + error_y**2)

            self.get_logger().info(f'Returning to dock: Pos=({x:.2f},{y:.2f}) Dock=({tar_x:.2f},{tar_y:.2f}) Dist={dist_to_dock:.2f}m')

            if dist_to_dock < 0.1:
                self.get_logger().info(f"Reached dock!")
                self.publish_bot_cmd([0.0, 0.0, 0.0])
                self.pid_theta.reset()
                self.pickup_state = 'aligning_theta'
                self.get_logger().info("State: ALIGNING_THETA")
            else:
                vx = self.pid_x.compute(error_x, dt)
                vy = self.pid_y.compute(error_y, dt)
                w = 0.0
                self.publish_bot_cmd([vx, vy, w])

        elif self.pickup_state == 'aligning_theta':
            tar_theta = self.dock_position['theta']
            error_theta = self.normalize_angle(theta - tar_theta)
            
            if abs(error_theta) < 0.035:
                self.get_logger().info(f"Theta aligned!")
                self.publish_bot_cmd([0.0, 0.0, 0.0])
                self.pickup_state = 'final_done'
                self.get_logger().info("State: FINAL_DONE - All tasks completed!")
            else:
                vx = 0.0
                vy = 0.0
                w = self.pid_theta.compute(error_theta, dt)
                self.publish_bot_cmd([vx, vy, w])

    # -------- PUBLISHER --------
    def publish_bot_cmd(self, vel):
        vx, vy, w = vel[0], vel[1], vel[2]
        
        v_FL, v_FR, v_BL, v_BR = self.mecanum_kinematics(vx, vy, w)
        
        cmd = BotCmd()
        cmd.id = self.bot_id
        cmd.m1 = float(v_FL) * 30.0
        cmd.m2 = float(v_FR) * 30.0
        cmd.m3 = float(v_BL) * 30.0
        cmd.m4 = float(v_BR) * 30.0
        cmd.base = float(self.arm_base_angle)
        cmd.elbow = float(self.arm_elbow_angle)
        
        self.bot_cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    controller = MecanumPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
