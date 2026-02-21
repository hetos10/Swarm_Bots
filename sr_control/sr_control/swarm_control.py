#!/usr/bin/env python3

'''
Mecanum Holonomic Controller
Debugging Version with Enhanced Logging
'''

import numpy as np
if not hasattr(np, 'float'):
    np.float = float  

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
from tf_transformations import euler_from_quaternion
from linkattacher_msgs.srv import AttachLink, DetachLink
import json

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

class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        
        self.last_time = self.get_clock().now()
        self.first_odom_received = False
        
        # --- TARGETS ---
        self.crate_x = 4.7
        self.crate_y = 4.9
        self.exchange_zone_x = 0.0
        self.exchange_zone_y = 0.0
        self.dock_position = {'x': -4.5 ,'y': 4.0, 'theta': 0.0}
        
        self.goals = [(self.crate_x, self.crate_y, 0.0)]
        self.goal_idx = 0
        
        # --- ROBOT PARAMS ---
        self.max_vel = 2.0 
        self.wheel_radius = 0.1
        self.wheel_separation_x = 0.4
        self.wheel_separation_y = 0.4

        self.pid_params = {
            'x': {'kp': 2.0, 'ki': 0.0, 'kd': 0.1, 'max_out': self.max_vel},
            'y': {'kp': 2.0, 'ki': 0.0, 'kd': 0.1, 'max_out': self.max_vel},
            'theta': {'kp': 1.0, 'ki': 0.0, 'kd': 0.0, 'max_out': self.max_vel / 2}
        }

        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        # --- ROS INTERFACE ---
        self.odom_sub = self.create_subscription(Odometry, '/lifter1/odom', self.odom_callback, 10)
        self.wheel_fl_pub = self.create_publisher(Float64, '/model/lifter1/joint/wheel_fl_joint/cmd_vel', 10)
        self.wheel_fr_pub = self.create_publisher(Float64, '/model/lifter1/joint/wheel_fr_joint/cmd_vel', 10)
        self.wheel_bl_pub = self.create_publisher(Float64, '/model/lifter1/joint/wheel_bl_joint/cmd_vel', 10)
        self.wheel_br_pub = self.create_publisher(Float64, '/model/lifter1/joint/wheel_br_joint/cmd_vel', 10)
        self.arm_base_pub = self.create_publisher(Float64, '/model/lifter1/joint/arm_joint_1/cmd_vel', 10)
        self.arm_elbow_pub = self.create_publisher(Float64, '/model/lifter1/joint/arm_joint_2/cmd_vel', 10)
        
        self.attach_cli = self.create_client(AttachLink, '/attach_link')
        self.detach_cli = self.create_client(DetachLink, '/detach_link')

        self.pickup_state = 'moving'
        self.arm_base_angle = 1.57
        self.arm_elbow_angle = 1.57
        self.state_timer = None
        self.attach_future = None
        self.detach_future = None

        self.timer = self.create_timer(0.03, self.control_cb)
        self.get_logger().info('*** Controller Initialized: Robot Spawning at (-4.5, 4.0) ***')

    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.odom_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.current_x = self.odom_x
        self.current_y = self.odom_y 
        self.current_theta = self.odom_theta 
        
        if not self.first_odom_received:
            self.first_odom_received = True
            self.get_logger().info('--- Odometry Synchronized with Global Map ---')
        
    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def mecanum_kinematics(self, vx, vy, w):
        """
        ALTERNATIVE Mecanum IK
        For wheels arranged as:
        FL  FR
        BL  BR
        """
        L_x = self.wheel_separation_x / 2.0  # 0.2m
        L_y = self.wheel_separation_y / 2.0  # 0.2m
        R = self.wheel_radius  # 0.1m
        
        # TRY: Different IK formulation
        v_FL = (vx + vy - (L_x + L_y) * w) / R
        v_FR = (vx - vy + (L_x + L_y) * w) / R
        v_BL = (vx - vy - (L_x + L_y) * w) / R
        v_BR = (vx + vy + (L_x + L_y) * w) / R
        
        return v_FL, v_FR, v_BL, v_BR

    def call_attach_service_async(self, model1, link1, model2, link2):
        self.get_logger().info(f'Calling Attach Service: {model1} -> {model2}')
        if not self.attach_cli.wait_for_service(timeout_sec=1): 
            self.get_logger().warn('Attach Service not available!')
            return None
        req = AttachLink.Request()
        req.data = json.dumps({"model1_name": model1, "link1_name": link1, "model2_name": model2, "link2_name": link2})
        return self.attach_cli.call_async(req)

    def call_detach_service_async(self, model1, link1, model2, link2):
        self.get_logger().info(f'Calling Detach Service: {model1} -> {model2}')
        if not self.detach_cli.wait_for_service(timeout_sec=1): 
            self.get_logger().warn('Detach Service not available!')
            return None
        req = DetachLink.Request()
        req.data = json.dumps({"model1_name": model1, "link1_name": link1, "model2_name": model2, "link2_name": link2})
        return self.detach_cli.call_async(req)

    def publish_wheel_velocities(self, vx, vy, w):
        v_FL, v_FR, v_BL, v_BR = self.mecanum_kinematics(vx, vy, w)
        
        # DEBUG: Print wheel velocities
        if vx != 0 or vy != 0 or w != 0:
            self.get_logger().info(f'[INPUT] vx={vx:.3f} vy={vy:.3f} w={w:.3f}')
            self.get_logger().info(f'[WHEELS] FL={v_FL:.3f} FR={v_FR:.3f} BL={v_BL:.3f} BR={v_BR:.3f}')
        
        # Original order
        self.wheel_fl_pub.publish(Float64(data=-v_FL))
        self.wheel_fr_pub.publish(Float64(data=v_FR))  # NEGATE
        self.wheel_bl_pub.publish(Float64(data=v_BL))  # NEGATE
        self.wheel_br_pub.publish(Float64(data=-v_BR))

    def publish_arm_cmd(self):
        self.arm_base_pub.publish(Float64(data=self.arm_base_angle))
        self.arm_elbow_pub.publish(Float64(data=self.arm_elbow_angle))

    def control_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return
        self.last_time = now
        if not self.first_odom_received: return

        x, y = self.current_x, self.current_y

        # ========== STATE MACHINE ==========
        if self.pickup_state == 'moving':
            tar_x, tar_y, _ = self.goals[self.goal_idx]
            error_x, error_y = tar_x - x, tar_y - y
            dist = math.sqrt(error_x**2 + error_y**2)

            self.get_logger().info(f'[STATE: MOVING] Dist: {dist:.2f}m | Pos: ({x:.2f},{y:.2f}) | Target: ({tar_x:.2f},{tar_y:.2f})', throttle_duration_sec=1.0)

            if dist < 0.15:
                self.get_logger().info('>>> Crate Position Reached! Stopping and Lowering Arm...')
                self.publish_wheel_velocities(0.0, 0.0, 0.0)
                self.arm_base_angle, self.arm_elbow_angle = 1.67, 1.57
                self.publish_arm_cmd()
                self.pickup_state, self.state_timer = 'lowering', now
            else:
                vx = self.pid_x.compute(error_x, dt)
                vy = self.pid_y.compute(error_y, dt)
                self.publish_wheel_velocities(vx, vy, 0.0)
        
        elif self.pickup_state == 'lowering':
            elapsed = (now - self.state_timer).nanoseconds / 1e9
            if elapsed > 1.5:
                self.get_logger().info('--- Arm Lowered. Initiating Attach Request ---')
                self.attach_future = self.call_attach_service_async("lifter1", "arm_link_2", "crate", "box_link")
                self.pickup_state, self.state_timer = 'attaching', now
        
        elif self.pickup_state == 'attaching':
            if self.attach_future and self.attach_future.done():
                self.get_logger().info('>>> Crate Successfully Attached. Lifting...')
                self.arm_base_angle, self.arm_elbow_angle = 0.0, 1.57 
                self.publish_arm_cmd()
                self.pickup_state, self.state_timer = 'lifting', now

        elif self.pickup_state == 'lifting':
            if (now - self.state_timer).nanoseconds / 1e9 > 1.0:
                self.get_logger().info('>>> Lift Complete. Heading to Exchange Zone (0,0)')
                self.pickup_state = 'moving_to_target'
                self.pid_x.reset()
                self.pid_y.reset()

        elif self.pickup_state == 'moving_to_target':
            error_x, error_y = self.exchange_zone_x - x, self.exchange_zone_y - y
            dist = math.sqrt(error_x**2 + error_y**2)
            self.get_logger().info(f'[STATE: CARRYING] Dist to Zone: {dist:.2f}m | Pos: ({x:.2f},{y:.2f})', throttle_duration_sec=1.0)

            if dist < 0.15:
                self.get_logger().info('>>> Exchange Zone Reached! Lowering Crate...')
                self.publish_wheel_velocities(0.0, 0.0, 0.0)
                self.arm_base_angle, self.arm_elbow_angle = 1.67, 1.57
                self.publish_arm_cmd()
                self.pickup_state, self.state_timer = 'lowering_at_target', now
            else:
                vx = self.pid_x.compute(error_x, dt)
                vy = self.pid_y.compute(error_y, dt)
                self.publish_wheel_velocities(vx, vy, 0.0)

        elif self.pickup_state == 'lowering_at_target':
            if (now - self.state_timer).nanoseconds / 1e9 > 1.5:
                self.get_logger().info('--- Crate Positioned. Detaching ---')
                self.detach_future = self.call_detach_service_async("lifter1", "arm_link_2", "crate", "box_link")
                self.pickup_state, self.state_timer = 'detaching', now

        elif self.pickup_state == 'detaching':
            if self.detach_future and self.detach_future.done():
                self.get_logger().info('>>> Crate Detached. Returning to Initial Dock...')
                self.arm_base_angle, self.arm_elbow_angle = 1.57, 1.57
                self.publish_arm_cmd()
                self.pickup_state, self.state_timer = 'returning_to_dock', now

        elif self.pickup_state == 'returning_to_dock':
            error_x, error_y = self.dock_position['x'] - x, self.dock_position['y'] - y
            dist = math.sqrt(error_x**2 + error_y**2)
            self.get_logger().info(f'[STATE: RETURNING] Dist to Dock: {dist:.2f}m', throttle_duration_sec=1.0)

            if dist < 0.15:
                self.publish_wheel_velocities(0.0, 0.0, 0.0)
                self.pickup_state = 'final_done'
                self.get_logger().info('MISSION COMPLETE: Robot back at (-4.5, 4.0)')
            else:
                vx = self.pid_x.compute(error_x, dt)
                vy = self.pid_y.compute(error_y, dt)
                self.publish_wheel_velocities(vx, vy, 0.0)

def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()