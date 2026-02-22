#!/usr/bin/env python3

'''
8-Robot Swarm Controller
- Controls 4 Lifter + 4 Runner pairs in parallel
- Each pair: Lifter picks crate → delivers to exchange → Runner picks up → delivers to drop
- Synchronized with proper timing and state management
'''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
import math
import json
from scipy.spatial.transform import Rotation as R_scipy
from linkattacher_msgs.srv import AttachLink, DetachLink
from rclpy.executors import MultiThreadedExecutor


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
        self.derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.output = self.kp * error + self.ki * self.integral + self.kd * self.derivative
        self.output = max(-self.max_out, min(self.output, self.max_out))
        self.prev_error = error
        return self.output
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class RobotController(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_controller')
        
        self.robot_name = robot_name
        self.robot_type = 'lifter' if 'lifter' in robot_name else 'runner'
        self.robot_num = int(robot_name[-1])
        
        # ========== STATE VARIABLES ==========
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.first_odom = False
        
        self.state = 'waiting'
        self.current_goal = None
        self.tasks = {}
        
        self.last_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.log_interval = 2.0  # Log every 2 seconds
        
        # ========== TIMING ==========
        self.timer_start = None
        self.attach_future = None
        self.detach_future = None
        
        # ========== CONFIGURATION ==========
        self.max_vel = 8.0
        self.wheel_radius = 0.1
        self.wheel_separation_y = 0.4
        
        # ========== PID CONTROLLERS ==========
        self.pid_x = PID(kp=1.5, ki=0.0, kd=0.0, max_out=self.max_vel)
        self.pid_theta = PID(kp=1.5, ki=0.0, kd=0.0, max_out=6.0)
        
        # ========== SUBSCRIBERS ==========
        self.odom_sub = self.create_subscription(
            Odometry, f'/{robot_name}/odom', self.odom_callback, 10
        )
        self.tasks_sub = self.create_subscription(
            String, f'/{self.robot_type}_tasks', self.tasks_callback, 10
        )
        
        # ========== PUBLISHERS (Wheel Control) ==========
        self.wheel_fl_pub = self.create_publisher(Float64, f'/model/{robot_name}/joint/wheel_fl_joint/cmd_vel', 10)
        self.wheel_fr_pub = self.create_publisher(Float64, f'/model/{robot_name}/joint/wheel_fr_joint/cmd_vel', 10)
        self.wheel_bl_pub = self.create_publisher(Float64, f'/model/{robot_name}/joint/wheel_bl_joint/cmd_vel', 10)
        self.wheel_br_pub = self.create_publisher(Float64, f'/model/{robot_name}/joint/wheel_br_joint/cmd_vel', 10)
        
        # ========== PUBLISHERS (Arm Control) ==========
        self.arm_base_pub = self.create_publisher(Float64, f'/model/{robot_name}/joint/arm_joint_1/cmd_vel', 10)
        self.arm_elbow_pub = self.create_publisher(Float64, f'/model/{robot_name}/joint/arm_joint_2/cmd_vel', 10)
        
        # ========== SERVICE CLIENTS ==========
        self.attach_cli = self.create_client(AttachLink, '/attach_link')
        self.detach_cli = self.create_client(DetachLink, '/detach_link')
        
        # ========== CONTROL LOOP ==========
        self.timer = self.create_timer(0.03, self.control_loop)
        
        self.get_logger().info(f'✓ {robot_name} controller initialized ({self.robot_type})')

    def odom_callback(self, msg: Odometry):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.pose_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        if not self.first_odom:
            self.first_odom = True
            self.get_logger().info(f'✓ {self.robot_name} odometry ready')

    def tasks_callback(self, msg: String):
        data = json.loads(msg.data)
        if self.robot_name in data:
            self.tasks = data[self.robot_name]
            self.state = 'moving_to_pickup' if self.robot_type == 'lifter' else 'moving_to_exchange'
            self.get_logger().info(f'✓ {self.robot_name} received tasks')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def differential_drive_kinematics(self, vx, w):
        half_sep = self.wheel_separation_y / 2.0
        R = self.wheel_radius
        v_left = vx - w * half_sep
        v_right = vx + w * half_sep
        return v_left/R, v_right/R, v_left/R, v_right/R

    def publish_wheels(self, vx, w):
        v_FL, v_FR, v_BL, v_BR = self.differential_drive_kinematics(vx, w)
        self.wheel_fl_pub.publish(Float64(data=v_FL))
        self.wheel_fr_pub.publish(Float64(data=v_FR))
        self.wheel_bl_pub.publish(Float64(data=v_BL))
        self.wheel_br_pub.publish(Float64(data=v_BR))

    def publish_arm(self, base, elbow):
        self.arm_base_pub.publish(Float64(data=base))
        self.arm_elbow_pub.publish(Float64(data=elbow))

    def move_to_target(self, target, dt):
        """Move to target location"""
        dx = target['x'] - self.pose_x
        dy = target['y'] - self.pose_y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist > 0.01:
            desired_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(desired_angle - self.pose_theta)
        else:
            angle_error = 0.0

        if abs(angle_error) > 0.15:
            w = self.pid_theta.compute(angle_error, dt)
            vx = 0.0
        else:
            vx = self.pid_x.compute(dist, dt)
            w = 0.0

        return vx, w, dist

    def call_attach_service(self, model, link, crate):
        if not self.attach_cli.wait_for_service(timeout_sec=3):
            return None
        req = AttachLink.Request()
        req.data = json.dumps({
            "model1_name": model,
            "link1_name": link,
            "model2_name": crate,
            "link2_name": "box_link"
        })
        return self.attach_cli.call_async(req)

    def call_detach_service(self, model, link, crate):
        if not self.detach_cli.wait_for_service(timeout_sec=3):
            return None
        req = DetachLink.Request()
        req.data = json.dumps({
            "model1_name": model,
            "link1_name": link,
            "model2_name": crate,
            "link2_name": "box_link"
        })
        return self.detach_cli.call_async(req)

    def control_loop(self):
        if not self.first_odom or not self.tasks:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        time_since_log = (now - self.last_log_time).nanoseconds / 1e9
        should_log = time_since_log >= self.log_interval
        if should_log:
            self.last_log_time = now

        crate_name = self.tasks.get('crate', 'unknown')

        # ========== LIFTER STATE MACHINE ==========
        if self.robot_type == 'lifter':
            if self.state == 'moving_to_pickup':
                vx, w, dist = self.move_to_target(self.tasks['pickup'], dt)
                if should_log:
                    self.get_logger().info(f'[{self.robot_name}] Moving to pickup: {dist:.2f}m')
                
                if dist < 0.15:
                    self.state = 'waiting_before_lower'
                    self.timer_start = now
                    self.publish_wheels(0.0, 0.0)
                else:
                    self.publish_wheels(vx, w)

            elif self.state == 'waiting_before_lower':
                self.publish_wheels(0.0, 0.0)
                self.publish_arm(0.0, 0.0)
                elapsed = (now - self.timer_start).nanoseconds / 1e9
                if elapsed > 2.0:
                    self.state = 'lowering_arm'
                    self.timer_start = now

            elif self.state == 'lowering_arm':
                self.publish_wheels(0.0, 0.0)
                self.publish_arm(1.57, 0.0)
                elapsed = (now - self.timer_start).nanoseconds / 1e9
                if elapsed > 2.0:
                    self.attach_future = self.call_attach_service(self.robot_name, 'arm_link_2', crate_name)
                    self.state = 'waiting_for_attach'
                    self.timer_start = now

            elif self.state == 'waiting_for_attach':
                self.publish_wheels(0.0, 0.0)
                self.publish_arm(1.57, 0.0)
                if self.attach_future and self.attach_future.done():
                    try:
                        self.attach_future.result()
                        if should_log:
                            self.get_logger().info(f'[{self.robot_name}] ✓ Crate attached!')
                        self.state = 'lifting_crate'
                        self.timer_start = now
                    except Exception as e:
                        self.state = 'lowering_arm'

            elif self.state == 'lifting_crate':
                self.publish_wheels(0.0, 0.0)
                self.publish_arm(0.0, 0.0)
                elapsed = (now - self.timer_start).nanoseconds / 1e9
                if elapsed > 2.0:
                    self.state = 'moving_to_exchange'
                    self.pid_x.reset()
                    self.pid_theta.reset()

            elif self.state == 'moving_to_exchange':
                vx, w, dist = self.move_to_target(self.tasks['exchange'], dt)
                if should_log:
                    self.get_logger().info(f'[{self.robot_name}] Moving to exchange: {dist:.2f}m')
                
                if dist < 0.15:
                    self.state = 'waiting_for_handoff'
                    self.timer_start = now
                    self.publish_wheels(0.0, 0.0)
                else:
                    self.publish_wheels(vx, w)

            elif self.state == 'waiting_for_handoff':
                self.publish_wheels(0.0, 0.0)
                self.publish_arm(0.0, 0.0)
                elapsed = (now - self.timer_start).nanoseconds / 1e9
                if elapsed > 3.0:
                    self.publish_arm(1.57, 0.0)
                    self.detach_future = self.call_detach_service(self.robot_name, 'arm_link_2', crate_name)
                    self.state = 'waiting_for_detach'

            elif self.state == 'waiting_for_detach':
                self.publish_wheels(0.0, 0.0)
                self.publish_arm(1.57, 0.0)
                if self.detach_future and self.detach_future.done():
                    try:
                        self.detach_future.result()
                        if should_log:
                            self.get_logger().info(f'[{self.robot_name}] ✓ Crate detached!')
                        self.state = 'returning_home'
                        self.pid_x.reset()
                        self.pid_theta.reset()
                    except:
                        pass

            elif self.state == 'returning_home':
                vx, w, dist = self.move_to_target(self.tasks['home'], dt)
                if should_log:
                    self.get_logger().info(f'[{self.robot_name}] Returning home: {dist:.2f}m')
                
                if dist < 0.1:
                    self.publish_wheels(0.0, 0.0)
                    self.publish_arm(0.0, 0.0)
                    self.state = 'done'
                    if should_log:
                        self.get_logger().info(f'✓✓✓ [{self.robot_name}] MISSION COMPLETE!')
                else:
                    self.publish_wheels(vx, w)

        # ========== RUNNER STATE MACHINE ==========
        else:  # robot_type == 'runner'
            if self.state == 'moving_to_exchange':
                vx, w, dist = self.move_to_target(self.tasks['exchange'], dt)
                if should_log:
                    self.get_logger().info(f'[{self.robot_name}] Moving to exchange: {dist:.2f}m')
                
                if dist < 0.15:
                    self.state = 'waiting_for_crate'
                    self.timer_start = now
                    self.publish_wheels(0.0, 0.0)
                else:
                    self.publish_wheels(vx, w)

            elif self.state == 'waiting_for_crate':
                self.publish_wheels(0.0, 0.0)
                self.publish_arm(0.0, 0.0)
                elapsed = (now - self.timer_start).nanoseconds / 1e9
                if elapsed > 2.5:
                    self.attach_future = self.call_attach_service(self.robot_name, 'arm_link_2', crate_name)
                    self.state = 'waiting_for_pickup'

            elif self.state == 'waiting_for_pickup':
                self.publish_wheels(0.0, 0.0)
                if self.attach_future and self.attach_future.done():
                    try:
                        self.attach_future.result()
                        if should_log:
                            self.get_logger().info(f'[{self.robot_name}] ✓ Crate received!')
                        self.state = 'moving_to_drop'
                        self.pid_x.reset()
                        self.pid_theta.reset()
                    except:
                        self.state = 'waiting_for_crate'

            elif self.state == 'moving_to_drop':
                vx, w, dist = self.move_to_target(self.tasks['drop'], dt)
                if should_log:
                    self.get_logger().info(f'[{self.robot_name}] Moving to drop: {dist:.2f}m')
                
                if dist < 0.15:
                    self.state = 'lowering_at_drop'
                    self.timer_start = now
                    self.publish_wheels(0.0, 0.0)
                else:
                    self.publish_wheels(vx, w)

            elif self.state == 'lowering_at_drop':
                self.publish_wheels(0.0, 0.0)
                self.publish_arm(1.57, 0.0)
                elapsed = (now - self.timer_start).nanoseconds / 1e9
                if elapsed > 2.0:
                    self.detach_future = self.call_detach_service(self.robot_name, 'arm_link_2', crate_name)
                    self.state = 'waiting_for_drop_detach'

            elif self.state == 'waiting_for_drop_detach':
                self.publish_wheels(0.0, 0.0)
                self.publish_arm(1.57, 0.0)
                if self.detach_future and self.detach_future.done():
                    try:
                        self.detach_future.result()
                        if should_log:
                            self.get_logger().info(f'[{self.robot_name}] ✓ Crate dropped!')
                        self.state = 'returning_home'
                        self.pid_x.reset()
                        self.pid_theta.reset()
                    except:
                        pass

            elif self.state == 'returning_home':
                vx, w, dist = self.move_to_target(self.tasks['home'], dt)
                if should_log:
                    self.get_logger().info(f'[{self.robot_name}] Returning home: {dist:.2f}m')
                
                if dist < 0.1:
                    self.publish_wheels(0.0, 0.0)
                    self.publish_arm(0.0, 0.0)
                    self.state = 'done'
                    if should_log:
                        self.get_logger().info(f'✓✓✓ [{self.robot_name}] MISSION COMPLETE!')
                else:
                    self.publish_wheels(vx, w)


def main(args=None):
    rclpy.init(args=args)
    
    # Create controllers for all 8 robots
    robots = [
        'lifter1', 'lifter2', 'lifter3', 'lifter4',
        'runner1', 'runner2', 'runner3', 'runner4'
    ]
    
    controllers = [RobotController(bot) for bot in robots]
    
    # Use multi-threaded executor to run all robots in parallel
    executor = MultiThreadedExecutor()
    for controller in controllers:
        executor.add_node(controller)
    
    executor.spin()
    
    for controller in controllers:
        controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()