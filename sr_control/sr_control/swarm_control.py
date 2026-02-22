#!/usr/bin/env python3

'''
Single Lifter Robot Controller
- Pick up crate from crate zone
- Deliver to exchange zone
- Return to home
- Smooth movement with proper damping
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


class LifterController(Node):
    def __init__(self):
        super().__init__('lifter_controller')

        # ODOMETRY
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.last_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.log_interval = 1.0
        self.first_odom_received = False
        
        # TARGET POSITIONS
        self.crate_x = 4.9
        self.crate_y = 4.7
        self.exchange_x = 0.0
        self.exchange_y = 0.0
        
        
        # HOME POSITION
        self.home = {'x': -4.5, 'y': 4.0, 'theta': 0.0}
        
        # ARM STATE
        self.state = 'moving_to_crate'
        self.arm_base = 0.0
        self.arm_elbow = 0.0
        self.timer = None
        self.attach_future = None
        self.detach_future = None

        # WHEEL PARAMETERS
        self.wheel_radius = 0.1
        self.wheel_separation_y = 0.4
        self.pid_x = PID(kp=0.4, ki=0.0, kd=0.2, max_out=2.0)
        self.pid_theta = PID(kp=0.4, ki=0.0, kd=0.2, max_out=1.5)

        # Rotation should be even smoother to avoid throwing the crate
        self.pid_theta = PID(kp=1.0, ki=0.0, kd=0.2, max_out=3.0)

        # ODOMETRY SUBSCRIBER
        self.odom_sub = self.create_subscription(Odometry, '/lifter1/odom', self.odom_callback, 10)
        
        # WHEEL PUBLISHERS
        self.wheel_fl_pub = self.create_publisher(Float64, '/model/lifter1/joint/wheel_fl_joint/cmd_vel', 10)
        self.wheel_fr_pub = self.create_publisher(Float64, '/model/lifter1/joint/wheel_fr_joint/cmd_vel', 10)
        self.wheel_bl_pub = self.create_publisher(Float64, '/model/lifter1/joint/wheel_bl_joint/cmd_vel', 10)
        self.wheel_br_pub = self.create_publisher(Float64, '/model/lifter1/joint/wheel_br_joint/cmd_vel', 10)
        
        # ARM PUBLISHERS
        self.arm_base_pub = self.create_publisher(Float64, '/model/lifter1/joint/arm_joint_1/cmd_vel', 10)
        self.arm_elbow_pub = self.create_publisher(Float64, '/model/lifter1/joint/arm_joint_2/cmd_vel', 10)
        
        # SERVICE CLIENTS
        self.attach_cli = self.create_client(AttachLink, '/attach_link')
        self.detach_cli = self.create_client(DetachLink, '/detach_link')

        # CONTROL LOOP
        self.timer = self.create_timer(0.03, self.control_cb)

        self.get_logger().info('='*70)
        self.get_logger().info('Lifter Controller Started')
        self.get_logger().info('Smooth movement with arm damping')
        self.get_logger().info('='*70)

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
            self.get_logger().info(f'✓ Odometry synchronized! At ({self.current_x:.2f}, {self.current_y:.2f})')

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

    def publish_wheels(self, vx, w):
        v_FL, v_FR, v_BL, v_BR = self.differential_drive_kinematics(vx, w)
        self.wheel_fl_pub.publish(Float64(data=v_FL))
        self.wheel_fr_pub.publish(Float64(data=v_FR))
        self.wheel_bl_pub.publish(Float64(data=v_BL))
        self.wheel_br_pub.publish(Float64(data=v_BR))

    def publish_arm(self):
        self.arm_base_pub.publish(Float64(data=self.arm_base))
        self.arm_elbow_pub.publish(Float64(data=self.arm_elbow))

    def move_to_target(self, target_x, target_y, dt):
        error_x = target_x - self.current_x
        error_y = target_y - self.current_y
        dist = math.sqrt(error_x**2 + error_y**2)
        
        if dist > 0.01:
            desired_angle = math.atan2(error_y, error_x)
            angle_error = self.normalize_angle(desired_angle - self.current_theta)
        else:
            angle_error = 0.0

        if abs(angle_error) > 0.15:
            w = self.pid_theta.compute(angle_error, dt)
            vx = 0.0
        else:
            vx = self.pid_x.compute(dist, dt)
            w = 0.0

        return vx, w, dist

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

        if not self.first_odom_received:
            return

        # KEEP ARM RAISED during movement
        if self.state in ['moving_to_crate', 'moving_to_exchange', 'returning_home']:
            self.arm_base = 0.0
            self.arm_elbow = 0.0
            self.publish_arm()

        # ========== STATE MACHINE ==========
        if self.state == 'moving_to_crate':
            vx, w, dist = self.move_to_target(self.crate_x - 0.23, self.crate_y, dt)
            
            if should_log:
                self.get_logger().info(f'[MOVING] To crate: Dist={dist:.2f}m Pos=({self.current_x:.2f},{self.current_y:.2f})')
            
            if dist < 0.3:
                if should_log:
                    self.get_logger().info('✓ Crate reached! Stopping robot...')
                self.publish_wheels(0.0, 0.0)
                self.arm_base = 0.0
                self.arm_elbow = 0.0
                self.publish_arm()
                self.state = 'waiting_before_lower'
                self.timer = now
            else:
                self.publish_wheels(vx, w)

        elif self.state == 'waiting_before_lower':
            """WAIT before lowering arm - robot completely stopped"""
            self.publish_wheels(0.0, 0.0)
            elapsed = (now - self.timer).nanoseconds / 1e9
            
            if should_log:
                self.get_logger().info(f'[WAITING] Before lower: {elapsed:.1f}s elapsed (wait 2.0s)')
            
            if elapsed > 2.0:
                if should_log:
                    self.get_logger().info('[LOWERING] Arm now...')
                self.arm_base = 1.57
                self.arm_elbow = 0.0
                self.publish_arm()
                self.state = 'lowering_for_crate'
                self.timer = now

        elif self.state == 'lowering_for_crate':
            """Lower arm and wait for it to reach crate"""
            self.publish_wheels(0.0, 0.0)
            self.arm_base = 1.57
            self.arm_elbow = 0.0
            self.publish_arm()
            
            elapsed = (now - self.timer).nanoseconds / 1e9
            
            if should_log:
                self.get_logger().info(f'[LOWERING] Arm: {elapsed:.1f}s elapsed (wait 2.0s for arm to lower)')
            
            if elapsed > 2.0:
                if should_log:
                    self.get_logger().info('[ATTACHING] Calling attach service...')
                future = self.call_attach_service("lifter1", "gripper_link", "crate_red_1", "box_link")
                if future:
                    self.attach_future = future
                    self.state = 'waiting_for_attach'
                    self.timer = now

        elif self.state == 'waiting_for_attach':
            """WAIT for attach service response - robot COMPLETELY STOPPED"""
            self.publish_wheels(0.0, 0.0)
            self.arm_base = 1.57
            self.arm_elbow = 0.0
            self.publish_arm()
            
            elapsed = (now - self.timer).nanoseconds / 1e9
            
            if should_log:
                self.get_logger().info(f'[WAITING_ATTACH] Service response: {elapsed:.1f}s')
            
            if self.attach_future and self.attach_future.done():
                try:
                    result = self.attach_future.result()
                    if should_log:
                        self.get_logger().info('✓✓✓ CRATE ATTACHED! Now lifting...')
                    self.state = 'lifting_after_attach'
                    self.timer = now
                    self.attach_future = None
                except Exception as e:
                    if should_log:
                        self.get_logger().error(f'✗ Attach failed: {e}')
                    self.state = 'lowering_for_crate'

        elif self.state == 'lifting_after_attach':
            """Lift arm with crate - robot still stopped"""
            self.publish_wheels(0.0, 0.0)
            self.arm_base = 0.0
            self.arm_elbow = 0.0
            self.publish_arm()
            
            elapsed = (now - self.timer).nanoseconds / 1e9
            
            if should_log:
                self.get_logger().info(f'[LIFTING] Crate: {elapsed:.1f}s (wait 2.0s for lift)')
            
            if elapsed > 2.0:
                if should_log:
                    self.get_logger().info('✓ Arm raised with crate! Now moving to exchange...')
                self.state = 'moving_to_exchange'
                self.pid_x.reset()
                self.pid_theta.reset()

        elif self.state == 'moving_to_exchange':
            vx, w, dist = self.move_to_target(self.exchange_x, self.exchange_y, dt)
            
            if should_log:
                self.get_logger().info(f'[MOVING] To exchange: Dist={dist:.2f}m Pos=({self.current_x:.2f},{self.current_y:.2f})')
            
            if dist < 0.15:
                if should_log:
                    self.get_logger().info('✓ At exchange! Stopping robot...')
                self.publish_wheels(0.0, 0.0)
                self.arm_base = 0.0
                self.arm_elbow = 0.0
                self.publish_arm()
                self.state = 'waiting_before_detach'
                self.timer = now
            else:
                self.publish_wheels(vx, w)

        elif self.state == 'waiting_before_detach':
            """WAIT before lowering/detaching"""
            self.publish_wheels(0.0, 0.0)
            self.arm_base = 0.0
            self.arm_elbow = 0.0
            self.publish_arm()
            
            elapsed = (now - self.timer).nanoseconds / 1e9
            
            if should_log:
                self.get_logger().info(f'[WAITING] Before detach: {elapsed:.1f}s (wait 1.0s)')
            
            if elapsed > 1.0:
                if should_log:
                    self.get_logger().info('[LOWERING] Arm for detach...')
                self.arm_base = 1.57
                self.arm_elbow = 0.0
                self.publish_arm()
                self.state = 'lowering_for_detach'
                self.timer = now

        elif self.state == 'lowering_for_detach':
            """Lower arm at exchange"""
            self.publish_wheels(0.0, 0.0)
            self.arm_base = 1.57
            self.arm_elbow = 0.0
            self.publish_arm()
            
            elapsed = (now - self.timer).nanoseconds / 1e9
            
            if should_log:
                self.get_logger().info(f'[LOWERING] For detach: {elapsed:.1f}s (wait 2.0s)')
            
            if elapsed > 2.0:
                if should_log:
                    self.get_logger().info('[DETACHING] Calling detach service...')
                future = self.call_detach_service("lifter1", "gripper_link", "crate_red_1", "box_link")
                if future:
                    self.detach_future = future
                    self.state = 'waiting_for_detach'
                    self.timer = now

        elif self.state == 'waiting_for_detach':
            """WAIT for detach service response"""
            self.publish_wheels(0.0, 0.0)
            self.arm_base = 1.57
            self.arm_elbow = 0.0
            self.publish_arm()
            
            elapsed = (now - self.timer).nanoseconds / 1e9
            
            if should_log:
                self.get_logger().info(f'[WAITING_DETACH] Service response: {elapsed:.1f}s')
            
            if self.detach_future and self.detach_future.done():
                try:
                    result = self.detach_future.result()
                    if should_log:
                        self.get_logger().info('✓✓✓ CRATE DETACHED! Returning home...')
                    self.state = 'raising_after_detach'
                    self.timer = now
                    self.detach_future = None
                except Exception as e:
                    if should_log:
                        self.get_logger().error(f'✗ Detach failed: {e}')

        elif self.state == 'raising_after_detach':
            """Raise arm after detach"""
            self.publish_wheels(0.0, 0.0)
            self.arm_base = 0.0
            self.arm_elbow = 0.0
            self.publish_arm()
            
            elapsed = (now - self.timer).nanoseconds / 1e9
            
            if should_log:
                self.get_logger().info(f'[RAISING] After detach: {elapsed:.1f}s (wait 1.5s)')
            
            if elapsed > 1.5:
                if should_log:
                    self.get_logger().info('[RETURNING] Home now...')
                self.state = 'returning_home'
                self.pid_x.reset()
                self.pid_theta.reset()

        elif self.state == 'returning_home':
            vx, w, dist = self.move_to_target(self.home['x'], self.home['y'], dt)
            
            if should_log:
                self.get_logger().info(f'[RETURNING] Home: Dist={dist:.2f}m Pos=({self.current_x:.2f},{self.current_y:.2f})')
            
            if dist < 0.1:
                if should_log:
                    self.get_logger().info('='*70)
                    self.get_logger().info('✓✓✓ MISSION COMPLETE!')
                    self.get_logger().info('='*70)
                self.publish_wheels(0.0, 0.0)
                self.arm_base = 0.0
                self.arm_elbow = 0.0
                self.publish_arm()
                self.state = 'done'
            else:
                self.publish_wheels(vx, w)

def main(args=None):
    rclpy.init(args=args)
    controller = LifterController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()