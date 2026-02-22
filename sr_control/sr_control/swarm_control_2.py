#!/usr/bin/env python3

'''
Multi-Robot Differential Drive Controller - PRODUCTION READY
- LIFTER: Picks crate from crate zone, delivers to exchange
- RUNNER: Moves to exchange, receives crate, delivers to drop zone
- Synchronized handoff with proper arm control
- All timing delays and waiting states implemented
'''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
from scipy.spatial.transform import Rotation as R_scipy
from linkattacher_msgs.srv import AttachLink, DetachLink
import json


def euler_from_quaternion(q):
    """Convert quaternion to Euler angles"""
    r = R_scipy.from_quat([q[0], q[1], q[2], q[3]])
    euler = r.as_euler('xyz')
    return euler[0], euler[1], euler[2]


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


class MultiRobotController(Node):
    def __init__(self):
        super().__init__('swarm_control_2')

        # ========== LIFTER ODOMETRY ==========
        self.lifter_x = 0.0
        self.lifter_y = 0.0
        self.lifter_theta = 0.0
        self.lifter_first_odom = False
        
        # ========== RUNNER ODOMETRY ==========
        self.runner_x = 0.0
        self.runner_y = 0.0
        self.runner_theta = 0.0
        self.runner_first_odom = False
        
        self.last_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.log_interval = 1.0
        
        # ========== CRATE POSITION ==========
        self.crate_x = 4.9
        self.crate_y = 4.7
        
        # ========== EXCHANGE ZONE ==========
        self.exchange_x = 0.0
        self.exchange_y = 0.0
        
        # ========== DROP ZONE ==========
        self.drop_x = 4.7
        self.drop_y = -4.9
        
        # ========== HOME POSITIONS ==========
        self.lifter_home = {'x': -4.5, 'y': 4.0, 'theta': 0.0}
        self.runner_home = {'x': -4.5, 'y': -4.0, 'theta': 0.0}
        
        self.max_vel = 2.0  # Slower smooth movement
        
        # ========== LIFTER STATE ==========
        self.lifter_state = 'moving_to_crate'
        self.lifter_arm_base = 0.0
        self.lifter_arm_elbow = 0.0
        self.lifter_timer = None
        self.lifter_attach_future = None
        self.lifter_detach_future = None
        
        # ========== RUNNER STATE ==========
        self.runner_state = 'moving_to_exchange'
        self.runner_piston = 0.0
        self.runner_timer = None
        self.runner_attach_future = None
        self.runner_detach_future = None

        # ========== WHEEL PARAMETERS ==========
        self.wheel_radius = 0.1
        self.wheel_separation_y = 0.4

        # ========== PID CONTROLLERS - TUNED FOR SMOOTH MOVEMENT ==========
        pid_params_x = {'kp': 0.4, 'ki': 0.0, 'kd': 0.2, 'max_out': self.max_vel}
        pid_params_theta = {'kp': 0.4, 'ki': 0.0, 'kd': 0.2, 'max_out': 1.5}
        
        self.lifter_pid_x = PID(**pid_params_x)
        self.lifter_pid_theta = PID(**pid_params_theta)
        self.runner_pid_x = PID(**pid_params_x)
        self.runner_pid_theta = PID(**pid_params_theta)

        # ========== ODOMETRY SUBSCRIBERS ==========
        self.lifter_odom_sub = self.create_subscription(Odometry, '/lifter1/odom', self.lifter_odom_callback, 10)
        self.runner_odom_sub = self.create_subscription(Odometry, '/runner1/odom', self.runner_odom_callback, 10)
        
        # ========== LIFTER WHEEL PUBLISHERS ==========
        self.lifter_fl_pub = self.create_publisher(Float64, '/model/lifter1/joint/wheel_fl_joint/cmd_vel', 10)
        self.lifter_fr_pub = self.create_publisher(Float64, '/model/lifter1/joint/wheel_fr_joint/cmd_vel', 10)
        self.lifter_bl_pub = self.create_publisher(Float64, '/model/lifter1/joint/wheel_bl_joint/cmd_vel', 10)
        self.lifter_br_pub = self.create_publisher(Float64, '/model/lifter1/joint/wheel_br_joint/cmd_vel', 10)
        self.lifter_arm_base_pub = self.create_publisher(Float64, '/model/lifter1/joint/arm_joint_1/cmd_vel', 10)
        self.lifter_arm_elbow_pub = self.create_publisher(Float64, '/model/lifter1/joint/arm_joint_2/cmd_vel', 10)
        
        # ========== RUNNER WHEEL PUBLISHERS ==========
        self.runner_fl_pub = self.create_publisher(Float64, '/model/runner1/joint/wheel_fl_joint/cmd_vel', 10)
        self.runner_fr_pub = self.create_publisher(Float64, '/model/runner1/joint/wheel_fr_joint/cmd_vel', 10)
        self.runner_bl_pub = self.create_publisher(Float64, '/model/runner1/joint/wheel_bl_joint/cmd_vel', 10)
        self.runner_br_pub = self.create_publisher(Float64, '/model/runner1/joint/wheel_br_joint/cmd_vel', 10)
        self.runner_piston_pub = self.create_publisher(Float64, '/model/runner1/joint/piston_rod_joint/cmd_vel', 10)
        
        # ========== SERVICE CLIENTS ==========
        self.attach_cli = self.create_client(AttachLink, '/attach_link')
        self.detach_cli = self.create_client(DetachLink, '/detach_link')

        # CONTROL LOOP
        self.timer = self.create_timer(0.03, self.control_cb)

        self.get_logger().info('='*70)
        self.get_logger().info('Multi-Robot Controller - PRODUCTION READY')
        self.get_logger().info('LIFTER: Crate Zone → Exchange')
        self.get_logger().info('RUNNER: Home → Exchange → Drop Zone → Home')
        self.get_logger().info('='*70)

    def lifter_odom_callback(self, msg: Odometry):
        self.lifter_x = msg.pose.pose.position.x
        self.lifter_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.lifter_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        if not self.lifter_first_odom:
            self.lifter_first_odom = True
            self.get_logger().info(f'✓ Lifter synchronized at ({self.lifter_x:.2f}, {self.lifter_y:.2f})')

    def runner_odom_callback(self, msg: Odometry):
        self.runner_x = msg.pose.pose.position.x
        self.runner_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.runner_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        if not self.runner_first_odom:
            self.runner_first_odom = True
            self.get_logger().info(f'✓ Runner synchronized at ({self.runner_x:.2f}, {self.runner_y:.2f})')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def differential_drive_kinematics(self, vx, w):
        half_separation = self.wheel_separation_y / 2.0
        R = self.wheel_radius
        v_left = vx - w * half_separation
        v_right = vx + w * half_separation
        v_FL = v_left / R
        v_FR = v_right / R
        v_BL = v_left / R
        v_BR = v_right / R
        return v_FL, v_FR, v_BL, v_BR

    def publish_lifter_wheels(self, vx, w):
        v_FL, v_FR, v_BL, v_BR = self.differential_drive_kinematics(vx, w)
        self.lifter_fl_pub.publish(Float64(data=v_FL))
        self.lifter_fr_pub.publish(Float64(data=v_FR))
        self.lifter_bl_pub.publish(Float64(data=v_BL))
        self.lifter_br_pub.publish(Float64(data=v_BR))

    def publish_runner_wheels(self, vx, w):
        v_FL, v_FR, v_BL, v_BR = self.differential_drive_kinematics(vx, w)
        self.runner_fl_pub.publish(Float64(data=v_FL))
        self.runner_fr_pub.publish(Float64(data=v_FR))
        self.runner_bl_pub.publish(Float64(data=v_BL))
        self.runner_br_pub.publish(Float64(data=v_BR))

    def publish_lifter_arm(self):
        self.lifter_arm_base_pub.publish(Float64(data=self.lifter_arm_base))
        self.lifter_arm_elbow_pub.publish(Float64(data=self.lifter_arm_elbow))

    def publish_runner_piston(self):
        self.runner_piston_pub.publish(Float64(data=self.runner_piston))

    def move_to_target(self, current_x, current_y, current_theta, target_x, target_y, pid_x, pid_theta, dt):
        error_x = target_x - current_x
        error_y = target_y - current_y
        dist = math.sqrt(error_x**2 + error_y**2)
        
        if dist > 0.01:
            desired_angle = math.atan2(error_y, error_x)
            angle_error = self.normalize_angle(desired_angle - current_theta)
        else:
            angle_error = 0.0

        if abs(angle_error) > 0.15:
            w = pid_theta.compute(angle_error, dt)
            vx = 0.0
        else:
            vx = pid_x.compute(dist, dt)
            w = 0.0

        return vx, w, dist, angle_error

    def call_attach_service(self, model1, link1, model2, link2):
        if not self.attach_cli.wait_for_service(timeout_sec=3):
            return None
        req = AttachLink.Request()
        req.data = json.dumps({"model1_name": model1, "link1_name": link1, "model2_name": model2, "link2_name": link2})
        return self.attach_cli.call_async(req)

    def call_detach_service(self, model1, link1, model2, link2):
        if not self.detach_cli.wait_for_service(timeout_sec=3):
            return None
        req = DetachLink.Request()
        req.data = json.dumps({"model1_name": model1, "link1_name": link1, "model2_name": model2, "link2_name": link2})
        return self.detach_cli.call_async(req)

    def control_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        time_since_log = (now - self.last_log_time).nanoseconds / 1e9
        should_log = time_since_log >= self.log_interval
        if should_log:
            self.last_log_time = now

        if not (self.lifter_first_odom and self.runner_first_odom):
            return

        # ========== KEEP ARMS RAISED during movement ==========
        if self.lifter_state in ['moving_to_crate', 'moving_to_exchange', 'returning_home']:
            self.lifter_arm_base = 0.0
            self.lifter_arm_elbow = 0.0
            self.publish_lifter_arm()

        if self.runner_state in ['moving_to_exchange', 'moving_to_drop', 'returning_home']:
            self.runner_piston = 0.0
            self.publish_runner_piston()

        # ========== LIFTER STATE MACHINE ==========
        if self.lifter_state == 'moving_to_crate':
            vx, w, dist, _ = self.move_to_target(self.lifter_x, self.lifter_y, self.lifter_theta, 
                                                   self.crate_x - 0.3, self.crate_y, 
                                                   self.lifter_pid_x, self.lifter_pid_theta, dt)
            
            if should_log:
                self.get_logger().info(f'[LIFTER] Moving to crate: Dist={dist:.2f}m')
            
            if dist < 0.25:
                if should_log:
                    self.get_logger().info('✓ LIFTER: Crate reached! Waiting...')
                self.publish_lifter_wheels(0.0, 0.0)
                self.lifter_state = 'waiting_before_lower'
                self.lifter_timer = now
            else:
                self.publish_lifter_wheels(vx, w)

        elif self.lifter_state == 'waiting_before_lower':
            self.publish_lifter_wheels(0.0, 0.0)
            elapsed = (now - self.lifter_timer).nanoseconds / 1e9
            if elapsed > 2.0:
                if should_log:
                    self.get_logger().info('[LIFTER] Lowering arm...')
                self.lifter_arm_base = 1.57
                self.lifter_arm_elbow = 1.57
                self.publish_lifter_arm()
                self.lifter_state = 'lowering_for_crate'
                self.lifter_timer = now

        elif self.lifter_state == 'lowering_for_crate':
            self.publish_lifter_wheels(0.0, 0.0)
            self.lifter_arm_base = 1.57
            self.lifter_arm_elbow = 1.57
            self.publish_lifter_arm()
            elapsed = (now - self.lifter_timer).nanoseconds / 1e9
            if elapsed > 2.0:
                if should_log:
                    self.get_logger().info('[LIFTER] Attaching crate...')
                future = self.call_attach_service("lifter1", "gripper_link", "crate_red_1", "box_link")
                if future:
                    self.lifter_attach_future = future
                    self.lifter_state = 'waiting_for_attach'
                    self.lifter_timer = now

        elif self.lifter_state == 'waiting_for_attach':
            self.publish_lifter_wheels(0.0, 0.0)
            self.lifter_arm_base = 1.57
            self.lifter_arm_elbow = 1.57
            self.publish_lifter_arm()
            
            if self.lifter_attach_future and self.lifter_attach_future.done():
                try:
                    self.lifter_attach_future.result()
                    if should_log:
                        self.get_logger().info('✓ LIFTER: Crate attached! Lifting...')
                    self.lifter_state = 'lifting_after_attach'
                    self.lifter_timer = now
                    self.lifter_attach_future = None
                except Exception as e:
                    if should_log:
                        self.get_logger().error(f'Attach failed: {e}')
                    self.lifter_state = 'lowering_for_crate'

        elif self.lifter_state == 'lifting_after_attach':
            self.publish_lifter_wheels(0.0, 0.0)
            self.lifter_arm_base = 0.0
            self.lifter_arm_elbow = 0.0
            self.publish_lifter_arm()
            elapsed = (now - self.lifter_timer).nanoseconds / 1e9
            if elapsed > 2.0:
                if should_log:
                    self.get_logger().info('✓ LIFTER: Moving to exchange...')
                self.lifter_state = 'moving_to_exchange'
                self.lifter_pid_x.reset()
                self.lifter_pid_theta.reset()

        elif self.lifter_state == 'moving_to_exchange':
            vx, w, dist, _ = self.move_to_target(self.lifter_x, self.lifter_y, self.lifter_theta,
                                                   self.exchange_x, self.exchange_y,
                                                   self.lifter_pid_x, self.lifter_pid_theta, dt)
            
            if should_log:
                self.get_logger().info(f'[LIFTER] Moving to exchange: Dist={dist:.2f}m')
            
            if dist < 0.25:
                if should_log:
                    self.get_logger().info('✓ LIFTER: At exchange! Waiting for runner...')
                self.publish_lifter_wheels(0.0, 0.0)
                self.lifter_state = 'waiting_for_runner_pickup'
                self.lifter_timer = now
            else:
                self.publish_lifter_wheels(vx, w)

        elif self.lifter_state == 'waiting_for_runner_pickup':
            self.publish_lifter_wheels(0.0, 0.0)
            elapsed = (now - self.lifter_timer).nanoseconds / 1e9
            if elapsed > 2.5:
                if should_log:
                    self.get_logger().info('[LIFTER] Detaching crate...')
                self.lifter_arm_base = 1.57
                self.lifter_arm_elbow = 1.57
                self.publish_lifter_arm()
                future = self.call_detach_service("lifter1", "gripper_link", "crate_red_1", "box_link")
                if future:
                    self.lifter_detach_future = future
                    self.lifter_state = 'waiting_for_detach'
                    self.lifter_timer = now

        elif self.lifter_state == 'waiting_for_detach':
            self.publish_lifter_wheels(0.0, 0.0)
            self.lifter_arm_base = 1.57
            self.lifter_arm_elbow = 1.57
            self.publish_lifter_arm()
            
            if self.lifter_detach_future and self.lifter_detach_future.done():
                try:
                    self.lifter_detach_future.result()
                    if should_log:
                        self.get_logger().info('✓ LIFTER: Crate detached! Returning home...')
                    self.lifter_state = 'returning_home'
                    self.lifter_pid_x.reset()
                    self.lifter_pid_theta.reset()
                    self.lifter_detach_future = None
                except:
                    pass

        elif self.lifter_state == 'returning_home':
            vx, w, dist, _ = self.move_to_target(self.lifter_x, self.lifter_y, self.lifter_theta,
                                                   self.lifter_home['x'], self.lifter_home['y'],
                                                   self.lifter_pid_x, self.lifter_pid_theta, dt)
            
            if should_log:
                self.get_logger().info(f'[LIFTER] Returning home: Dist={dist:.2f}m')
            
            if dist < 0.1:
                if should_log:
                    self.get_logger().info('✓✓✓ LIFTER MISSION COMPLETE!')
                self.publish_lifter_wheels(0.0, 0.0)
                self.lifter_state = 'done'
            else:
                self.publish_lifter_wheels(vx, w)

        # ========== RUNNER STATE MACHINE ==========
        if self.runner_state == 'moving_to_exchange':
            vx, w, dist, _ = self.move_to_target(self.runner_x, self.runner_y, self.runner_theta,
                                                   self.exchange_x, self.exchange_y,
                                                   self.runner_pid_x, self.runner_pid_theta, dt)
            
            if should_log:
                self.get_logger().info(f'[RUNNER] Moving to exchange: Dist={dist:.2f}m')
            
            if dist < 0.25:
                if should_log:
                    self.get_logger().info('✓ RUNNER: At exchange! Waiting for crate...')
                self.publish_runner_wheels(0.0, 0.0)
                self.runner_state = 'waiting_at_exchange'
                self.runner_timer = now
            else:
                self.publish_runner_wheels(vx, w)

        elif self.runner_state == 'waiting_at_exchange':
            self.publish_runner_wheels(0.0, 0.0)
            if self.lifter_state in ['waiting_for_runner_pickup', 'waiting_for_detach']:
                elapsed = (now - self.runner_timer).nanoseconds / 1e9
                if elapsed > 1.0:
                    if should_log:
                        self.get_logger().info('[RUNNER] Picking up crate...')
                    future = self.call_attach_service("runner1", "base_link", "crate_red_1", "box_link")
                    if future:
                        self.runner_attach_future = future
                        self.runner_state = 'waiting_for_pickup'
                        self.runner_timer = now

        elif self.runner_state == 'waiting_for_pickup':
            self.publish_runner_wheels(0.0, 0.0)
            if self.runner_attach_future and self.runner_attach_future.done():
                try:
                    self.runner_attach_future.result()
                    if should_log:
                        self.get_logger().info('✓ RUNNER: Crate picked up! Moving to drop...')
                    self.runner_state = 'moving_to_drop'
                    self.runner_pid_x.reset()
                    self.runner_pid_theta.reset()
                    self.runner_attach_future = None
                except:
                    self.runner_state = 'waiting_at_exchange'

        elif self.runner_state == 'moving_to_drop':
            vx, w, dist, _ = self.move_to_target(self.runner_x, self.runner_y, self.runner_theta,
                                                   self.drop_x, self.drop_y,
                                                   self.runner_pid_x, self.runner_pid_theta, dt)
            
            if should_log:
                self.get_logger().info(f'[RUNNER] Moving to drop: Dist={dist:.2f}m')
            
            if dist < 0.15:
                if should_log:
                    self.get_logger().info('✓ RUNNER: At drop zone! Lowering arm...')
                self.publish_runner_wheels(0.0, 0.0)
                self.runner_piston = 0.0
                self.publish_runner_piston()
                future = self.call_detach_service("runner1", "base_link", "crate_red_1", "box_link")
                if future:
                    self.runner_detach_future = future
                    self.runner_state = 'waiting_for_drop_detach'
                    self.runner_timer = now
            else:
                self.publish_runner_wheels(vx, w)

        elif self.runner_state == 'waiting_for_drop_detach':
            self.publish_runner_wheels(0.0, 0.0)
            self.runner_piston = 0.3
            self.publish_runner_piston()
            
            if self.runner_detach_future and self.runner_detach_future.done():
                try:
                    self.runner_detach_future.result()
                    if should_log:
                        self.get_logger().info('✓ RUNNER: Crate dropped! Returning home...')
                    self.runner_state = 'returning_home'
                    self.runner_pid_x.reset()
                    self.runner_pid_theta.reset()
                    self.runner_detach_future = None
                except:
                    pass

        elif self.runner_state == 'returning_home':
            vx, w, dist, _ = self.move_to_target(self.runner_x, self.runner_y, self.runner_theta,
                                                   self.runner_home['x'], self.runner_home['y'],
                                                   self.runner_pid_x, self.runner_pid_theta, dt)
            
            if should_log:
                self.get_logger().info(f'[RUNNER] Returning home: Dist={dist:.2f}m')
            
            if dist < 0.1:
                if should_log:
                    self.get_logger().info('✓✓✓ RUNNER MISSION COMPLETE!')
                self.publish_runner_wheels(0.0, 0.0)
                self.runner_state = 'done'
            else:
                self.publish_runner_wheels(vx, w)


def main(args=None):
    rclpy.init(args=args)
    controller = MultiRobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()