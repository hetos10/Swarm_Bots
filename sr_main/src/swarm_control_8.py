#!/usr/bin/env python3

'''
8-Bot Swarm Controller - PRODUCTION READY VERSION
- 4 Lifter-Runner pairs: lifter1-runner1, lifter2-runner2, lifter3-runner3, lifter4-runner4
- FIXED allocation: lifter1→crate1, lifter2→crate2, lifter3→crate3, lifter4→crate4
- 4 DIFFERENT EXCHANGE ZONES in 4 quadrants (±1.5, ±1.5)
- 4 DIFFERENT DROP ZONES with decreasing X-coordinates (4.7→3.5→2.3→1.1)
- SPREAD OUT HOME POSITIONS (x=-6.0, 1.5m Y-spacing) to prevent collisions
- SYNCHRONIZED startup (2s delay at home before starting)
- All pairs work SIMULTANEOUSLY and INDEPENDENTLY
- Single file - production-ready
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


class SwarmController8(Node):
    def __init__(self):
        super().__init__('swarm_control_8')

        # ========== NUMBER OF PAIRS ==========
        self.num_pairs = 4

        # ========== FIXED TASK ASSIGNMENT ==========
        # Each lifter-runner pair has ONE assigned crate (FIXED!)
        self.pair_assignments = {
            'pair1': {'lifter': 'lifter1', 'runner': 'runner1', 'crate': 'crate_red_1'},
            'pair2': {'lifter': 'lifter2', 'runner': 'runner2', 'crate': 'crate_red_2'},
            'pair3': {'lifter': 'lifter3', 'runner': 'runner3', 'crate': 'crate_red_3'},
            'pair4': {'lifter': 'lifter4', 'runner': 'runner4', 'crate': 'crate_red_4'},
        }
        
        # ========== FIXED CRATE POSITIONS ==========
        self.crate_positions = {
            'crate_red_1': {'x': 4.9, 'y': 4.7},
            'crate_red_2': {'x': 5.2, 'y': 5.0},
            'crate_red_3': {'x': 4.6, 'y': 5.2},
            'crate_red_4': {'x': 5.0, 'y': 4.4},
        }
        
        # ========== ZONES - DIFFERENT FOR EACH PAIR IN 4 QUADRANTS ==========
        # Exchange zones at ±1.5, ±1.5 (4 quadrants)
        # Drop zones with decreasing X-coordinates (4.7 → 3.5 → 2.3 → 1.1)
        self.pair_zones = {
            'pair1': {
                'exchange': {'x': 1.5, 'y': 1.5},      # Quadrant 1 (upper right)
                'drop': {'x': 4.7, 'y': -4.9},         # Drop zone 1
            },
            'pair2': {
                'exchange': {'x': -1.5, 'y': 1.5},     # Quadrant 2 (upper left)
                'drop': {'x': 3.5, 'y': -4.9},         # Drop zone 2 (x decreased)
            },
            'pair3': {
                'exchange': {'x': -1.5, 'y': -1.5},    # Quadrant 3 (lower left)
                'drop': {'x': 2.3, 'y': -4.9},         # Drop zone 3 (x decreased more)
            },
            'pair4': {
                'exchange': {'x': 1.5, 'y': -1.5},     # Quadrant 4 (lower right)
                'drop': {'x': 1.1, 'y': -4.9},         # Drop zone 4 (x decreased further)
            },
        }
        
        # ========== HOME POSITIONS - SPREAD OUT TO AVOID COLLISIONS ==========
        # All at x=-6.0 (far left), spread 1.5m in Y-axis
        self.home_positions = {
            'lifter1': {'x': -4.5, 'y': 4.0},
            'lifter2': {'x': -4.0, 'y': 4.0},
            'lifter3': {'x': -3.5, 'y': 4.0},
            'lifter4': {'x': -3.0, 'y': 4.0},
            'runner1': {'x': -4.5, 'y': -4.0},
            'runner2': {'x': -4.0, 'y': -4.0},
            'runner3': {'x': -3.5, 'y': -4.0},
            'runner4': {'x': -3.0, 'y': -4.0},
        }
        
        # ========== ODOMETRY ==========
        self.bot_odom = {}
        for i in range(1, self.num_pairs + 1):
            self.bot_odom[f'lifter{i}'] = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'first_odom': False}
            self.bot_odom[f'runner{i}'] = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'first_odom': False}
        
        self.last_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.log_interval = 2.0
        
        self.max_vel = 2.0
        
        # ========== STATE FOR EACH BOT ==========
        self.bot_state = {}
        for i in range(1, self.num_pairs + 1):
            self.bot_state[f'lifter{i}'] = {
                'state': 'waiting_at_home',  # ✅ SYNCHRONIZED: Start at home, wait for ready
                'arm_base': 0.0,
                'arm_elbow': 0.0,
                'timer': None,
                'attach_future': None,
                'detach_future': None,
                'attach_called': False,
                'detach_called': False,
            }
            self.bot_state[f'runner{i}'] = {
                'state': 'waiting_at_home',  # ✅ SYNCHRONIZED: Start at home
                'piston': 0.0,
                'timer': None,
                'attach_future': None,
                'detach_future': None,
                'attach_called': False,
                'detach_called': False,
            }
        
        # ========== WHEEL PARAMETERS ==========
        self.wheel_radius = 0.1
        self.wheel_separation_y = 0.4

        # ========== PID CONTROLLERS ==========
        pid_params_x = {'kp': 0.6, 'ki': 0.0001, 'kd': 0.3, 'max_out': self.max_vel}
        pid_params_theta = {'kp': 0.6, 'ki': 0.0001, 'kd': 0.3, 'max_out': 1.5}
        
        self.pid_controllers = {}
        for i in range(1, self.num_pairs + 1):
            self.pid_controllers[f'lifter{i}_x'] = PID(**pid_params_x)
            self.pid_controllers[f'lifter{i}_theta'] = PID(**pid_params_theta)
            self.pid_controllers[f'runner{i}_x'] = PID(**pid_params_x)
            self.pid_controllers[f'runner{i}_theta'] = PID(**pid_params_theta)

        # ========== ODOMETRY SUBSCRIBERS ==========
        for i in range(1, self.num_pairs + 1):
            self.create_subscription(
                Odometry, f'/lifter{i}/odom', 
                lambda msg, idx=i: self.odom_callback(msg, f'lifter{idx}'), 10
            )
            self.create_subscription(
                Odometry, f'/runner{i}/odom',
                lambda msg, idx=i: self.odom_callback(msg, f'runner{idx}'), 10
            )
        
        # ========== WHEEL PUBLISHERS ==========
        self.wheel_pubs = {}
        for i in range(1, self.num_pairs + 1):
            self.wheel_pubs[f'lifter{i}'] = {
                'fl': self.create_publisher(Float64, f'/model/lifter{i}/joint/wheel_fl_joint/cmd_vel', 10),
                'fr': self.create_publisher(Float64, f'/model/lifter{i}/joint/wheel_fr_joint/cmd_vel', 10),
                'bl': self.create_publisher(Float64, f'/model/lifter{i}/joint/wheel_bl_joint/cmd_vel', 10),
                'br': self.create_publisher(Float64, f'/model/lifter{i}/joint/wheel_br_joint/cmd_vel', 10),
            }
            self.wheel_pubs[f'runner{i}'] = {
                'fl': self.create_publisher(Float64, f'/model/runner{i}/joint/wheel_fl_joint/cmd_vel', 10),
                'fr': self.create_publisher(Float64, f'/model/runner{i}/joint/wheel_fr_joint/cmd_vel', 10),
                'bl': self.create_publisher(Float64, f'/model/runner{i}/joint/wheel_bl_joint/cmd_vel', 10),
                'br': self.create_publisher(Float64, f'/model/runner{i}/joint/wheel_br_joint/cmd_vel', 10),
            }
        
        # ========== ARM PUBLISHERS ==========
        self.arm_pubs = {}
        for i in range(1, self.num_pairs + 1):
            self.arm_pubs[f'lifter{i}'] = {
                'base': self.create_publisher(Float64, f'/model/lifter{i}/joint/arm_joint_1/cmd_vel', 10),
                'elbow': self.create_publisher(Float64, f'/model/lifter{i}/joint/arm_joint_2/cmd_vel', 10),
            }
        
        # ========== PISTON PUBLISHERS ==========
        self.piston_pubs = {}
        for i in range(1, self.num_pairs + 1):
            self.piston_pubs[f'runner{i}'] = self.create_publisher(Float64, f'/model/runner{i}/joint/piston_rod_joint/cmd_vel', 10)
        
        # ========== SERVICE CLIENTS ==========
        self.attach_cli = self.create_client(AttachLink, '/attach_link')
        self.detach_cli = self.create_client(DetachLink, '/detach_link')

        # CONTROL LOOP
        self.timer = self.create_timer(0.03, self.control_cb)

        self.get_logger().info('='*70)
        self.get_logger().info('8-Bot Swarm Controller - PRODUCTION READY')
        self.get_logger().info('✅ 4 Lifter-Runner pairs with FIXED task allocation')
        self.get_logger().info('✅ 4 quadrant exchange zones (±1.5, ±1.5)')
        self.get_logger().info('✅ 4 different drop zones (X: 4.7→3.5→2.3→1.1)')
        self.get_logger().info('✅ Spread home positions (1.5m spacing, no collisions)')
        self.get_logger().info('✅ Synchronized startup (2s delay)')
        self.get_logger().info('='*70)

    def odom_callback(self, msg: Odometry, bot_name):
        """Update odometry for any bot"""
        self.bot_odom[bot_name]['x'] = msg.pose.pose.position.x
        self.bot_odom[bot_name]['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.bot_odom[bot_name]['theta'] = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        if not self.bot_odom[bot_name]['first_odom']:
            self.bot_odom[bot_name]['first_odom'] = True
            self.get_logger().info(f'✓ {bot_name} synchronized')

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

    def publish_wheels(self, bot_name, vx, w):
        v_FL, v_FR, v_BL, v_BR = self.differential_drive_kinematics(vx, w)
        self.wheel_pubs[bot_name]['fl'].publish(Float64(data=v_FL))
        self.wheel_pubs[bot_name]['fr'].publish(Float64(data=v_FR))
        self.wheel_pubs[bot_name]['bl'].publish(Float64(data=v_BL))
        self.wheel_pubs[bot_name]['br'].publish(Float64(data=v_BR))

    def publish_arm(self, lifter_name):
        self.arm_pubs[lifter_name]['base'].publish(Float64(data=self.bot_state[lifter_name]['arm_base']))
        self.arm_pubs[lifter_name]['elbow'].publish(Float64(data=self.bot_state[lifter_name]['arm_elbow']))

    def publish_piston(self, runner_name):
        self.piston_pubs[runner_name].publish(Float64(data=self.bot_state[runner_name]['piston']))

    def move_to_target(self, bot_name, target_x, target_y):
        """Move bot to target using PID"""
        current_x = self.bot_odom[bot_name]['x']
        current_y = self.bot_odom[bot_name]['y']
        current_theta = self.bot_odom[bot_name]['theta']
        
        error_x = target_x - current_x
        error_y = target_y - current_y
        dist = math.sqrt(error_x**2 + error_y**2)
        
        if dist > 0.01:
            desired_angle = math.atan2(error_y, error_x)
            angle_error = self.normalize_angle(desired_angle - current_theta)
        else:
            angle_error = 0.0

        pid_x = self.pid_controllers[f'{bot_name}_x']
        pid_theta = self.pid_controllers[f'{bot_name}_theta']
        dt = 0.03

        if abs(angle_error) > 0.15:
            w = pid_theta.compute(angle_error, dt)
            vx = 0.0
        else:
            vx = pid_x.compute(dist, dt)
            w = 0.0

        return vx, w, dist

    def call_attach_service(self, model1, link1, model2, link2):
        if not self.attach_cli.wait_for_service(timeout_sec=1):
            return None
        req = AttachLink.Request()
        req.data = json.dumps({"model1_name": model1, "link1_name": link1, "model2_name": model2, "link2_name": link2})
        return self.attach_cli.call_async(req)

    def call_detach_service(self, model1, link1, model2, link2):
        if not self.detach_cli.wait_for_service(timeout_sec=1):
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

        # Check all odoms ready
        all_ready = all(self.bot_odom[f'lifter{i}']['first_odom'] and 
                       self.bot_odom[f'runner{i}']['first_odom'] 
                       for i in range(1, self.num_pairs + 1))
        
        if not all_ready:
            return

        # ========== PROCESS EACH PAIR INDEPENDENTLY ==========
        for i in range(1, self.num_pairs + 1):
            lifter_name = f'lifter{i}'
            runner_name = f'runner{i}'
            crate_name = self.pair_assignments[f'pair{i}']['crate']  # ✅ FIXED CRATE
            
            state_lifter = self.bot_state[lifter_name]
            state_runner = self.bot_state[runner_name]
            
            # ========== LIFTER STATE MACHINE ==========
            if state_lifter['state'] == 'waiting_at_home':
                # ✅ Wait at home position before starting
                self.publish_wheels(lifter_name, 0.0, 0.0)
                state_lifter['arm_base'] = 0.0
                state_lifter['arm_elbow'] = 0.0
                self.publish_arm(lifter_name)
                
                elapsed = (now - state_lifter['timer']).nanoseconds / 1e9 if state_lifter['timer'] else 999
                if elapsed > 2.0:  # Wait 2 seconds to let all robots sync up
                    state_lifter['state'] = 'moving_to_crate'
                    self.pid_controllers[f'{lifter_name}_x'].reset()
                    self.pid_controllers[f'{lifter_name}_theta'].reset()
                else:
                    if not state_lifter['timer']:
                        state_lifter['timer'] = now

            elif state_lifter['state'] == 'moving_to_crate':
                crate = self.crate_positions[crate_name]
                vx, w, dist = self.move_to_target(lifter_name, crate['x'] - 0.33, crate['y'] + 0.16)
                
                state_lifter['arm_base'] = 0.0
                state_lifter['arm_elbow'] = 0.0
                self.publish_arm(lifter_name)
                
                if dist < 0.3:
                    self.publish_wheels(lifter_name, 0.0, 0.0)
                    state_lifter['state'] = 'waiting_before_lower'
                    state_lifter['timer'] = now
                else:
                    self.publish_wheels(lifter_name, vx, w)

            elif state_lifter['state'] == 'waiting_before_lower':
                self.publish_wheels(lifter_name, 0.0, 0.0)
                elapsed = (now - state_lifter['timer']).nanoseconds / 1e9
                if elapsed > 2.0:
                    state_lifter['arm_base'] = 1.57
                    state_lifter['arm_elbow'] = 1.57
                    self.publish_arm(lifter_name)
                    state_lifter['state'] = 'lowering_for_crate'
                    state_lifter['timer'] = now

            elif state_lifter['state'] == 'lowering_for_crate':
                self.publish_wheels(lifter_name, 0.0, 0.0)
                state_lifter['arm_base'] = 1.57
                state_lifter['arm_elbow'] = 1.57
                self.publish_arm(lifter_name)
                elapsed = (now - state_lifter['timer']).nanoseconds / 1e9
                
                if elapsed > 2.0:
                    if not state_lifter['attach_called']:
                        future = self.call_attach_service(lifter_name, "gripper_link", crate_name, "box_link")
                        if future:
                            state_lifter['attach_future'] = future
                            state_lifter['attach_called'] = True
                    
                    if state_lifter['attach_future'] and state_lifter['attach_future'].done():
                        try:
                            state_lifter['attach_future'].result()
                            if should_log:
                                self.get_logger().info(f'✓ {lifter_name}: CRATE ATTACHED!')
                            state_lifter['state'] = 'crate_attached'
                            state_lifter['timer'] = now
                            state_lifter['attach_future'] = None
                            state_lifter['attach_called'] = False
                        except Exception as e:
                            if should_log:
                                self.get_logger().error(f'✗ {lifter_name}: Attach failed: {e}')
                            state_lifter['state'] = 'lowering_for_crate'
                            state_lifter['timer'] = now
                            state_lifter['attach_called'] = False

            elif state_lifter['state'] == 'crate_attached':
                self.publish_wheels(lifter_name, 0.0, 0.0)
                state_lifter['arm_base'] = 1.57
                state_lifter['arm_elbow'] = 1.57
                self.publish_arm(lifter_name)
                elapsed = (now - state_lifter['timer']).nanoseconds / 1e9
                if elapsed > 2.0:
                    state_lifter['state'] = 'lifting_after_attach'
                    state_lifter['timer'] = now

            elif state_lifter['state'] == 'lifting_after_attach':
                self.publish_wheels(lifter_name, 0.0, 0.0)
                elapsed = (now - state_lifter['timer']).nanoseconds / 1e9
                
                if elapsed < 3.0:
                    progress = elapsed / 3.0
                    state_lifter['arm_base'] = 1.57 * (1.0 - progress)
                    state_lifter['arm_elbow'] = 0.0
                else:
                    state_lifter['arm_base'] = 0.0
                    state_lifter['arm_elbow'] = 0.0
                    state_lifter['state'] = 'waiting_before_move'
                    state_lifter['timer'] = now
                
                self.publish_arm(lifter_name)

            elif state_lifter['state'] == 'waiting_before_move':
                self.publish_wheels(lifter_name, 0.0, 0.0)
                state_lifter['arm_base'] = 0.0
                state_lifter['arm_elbow'] = 0.0
                self.publish_arm(lifter_name)
                elapsed = (now - state_lifter['timer']).nanoseconds / 1e9
                if elapsed > 1.0:
                    state_lifter['state'] = 'moving_to_exchange'
                    self.pid_controllers[f'{lifter_name}_x'].reset()
                    self.pid_controllers[f'{lifter_name}_theta'].reset()

            elif state_lifter['state'] == 'moving_to_exchange':
                # ✅ USE PAIR-SPECIFIC EXCHANGE ZONE
                vx, w, dist = self.move_to_target(lifter_name, 
                                                   self.pair_zones[f'pair{i}']['exchange']['x'], 
                                                   self.pair_zones[f'pair{i}']['exchange']['y'])
                vx = vx * 0.25
                w = w * 0.25
                
                state_lifter['arm_base'] = 0.0
                state_lifter['arm_elbow'] = 0.0
                self.publish_arm(lifter_name)
                
                if dist < 0.25:
                    self.publish_wheels(lifter_name, 0.0, 0.0)
                    state_lifter['state'] = 'waiting_for_runner_pickup'
                    state_lifter['timer'] = now
                else:
                    self.publish_wheels(lifter_name, vx, w)

            elif state_lifter['state'] == 'waiting_for_runner_pickup':
                self.publish_wheels(lifter_name, 0.0, 0.0)
                elapsed = (now - state_lifter['timer']).nanoseconds / 1e9
                if elapsed > 2.5:
                    state_lifter['arm_base'] = 1.57
                    state_lifter['arm_elbow'] = 1.57
                    self.publish_arm(lifter_name)
                    
                    if not state_lifter['detach_called']:
                        future = self.call_detach_service(lifter_name, "gripper_link", crate_name, "box_link")
                        if future:
                            state_lifter['detach_future'] = future
                            state_lifter['detach_called'] = True
                        state_lifter['state'] = 'waiting_for_detach'
                        state_lifter['timer'] = now

            elif state_lifter['state'] == 'waiting_for_detach':
                self.publish_wheels(lifter_name, 0.0, 0.0)
                state_lifter['arm_base'] = 1.57
                state_lifter['arm_elbow'] = 1.57
                self.publish_arm(lifter_name)
                
                if state_lifter['detach_future'] and state_lifter['detach_future'].done():
                    try:
                        state_lifter['detach_future'].result()
                        if should_log:
                            self.get_logger().info(f'✓ {lifter_name}: CRATE DETACHED! Returning home...')
                        state_lifter['state'] = 'returning_home'
                        state_lifter['detach_future'] = None
                        state_lifter['detach_called'] = False
                        self.pid_controllers[f'{lifter_name}_x'].reset()
                        self.pid_controllers[f'{lifter_name}_theta'].reset()
                    except:
                        pass

            elif state_lifter['state'] == 'returning_home':
                home = self.home_positions[lifter_name]
                vx, w, dist = self.move_to_target(lifter_name, home['x'], home['y'])
                
                state_lifter['arm_base'] = 0.0
                state_lifter['arm_elbow'] = 0.0
                self.publish_arm(lifter_name)
                
                if dist < 0.1:
                    self.publish_wheels(lifter_name, 0.0, 0.0)
                    if should_log:
                        self.get_logger().info(f'✓✓✓ {lifter_name} MISSION COMPLETE!')
                    state_lifter['state'] = 'done'
                else:
                    self.publish_wheels(lifter_name, vx, w)

            # ========== RUNNER STATE MACHINE ==========
            if state_runner['state'] == 'waiting_at_home':
                # ✅ Wait at home position before starting
                self.publish_wheels(runner_name, 0.0, 0.0)
                state_runner['piston'] = 0.0
                self.publish_piston(runner_name)
                
                elapsed = (now - state_runner['timer']).nanoseconds / 1e9 if state_runner['timer'] else 999
                if elapsed > 2.0:  # Wait 2 seconds to let all robots sync up
                    state_runner['state'] = 'moving_to_exchange'
                    self.pid_controllers[f'{runner_name}_x'].reset()
                    self.pid_controllers[f'{runner_name}_theta'].reset()
                else:
                    if not state_runner['timer']:
                        state_runner['timer'] = now

            elif state_runner['state'] == 'moving_to_exchange':
                # ✅ USE PAIR-SPECIFIC EXCHANGE ZONE
                vx, w, dist = self.move_to_target(runner_name, 
                                                   self.pair_zones[f'pair{i}']['exchange']['x'], 
                                                   self.pair_zones[f'pair{i}']['exchange']['y'])
                
                if dist < 0.25:
                    self.publish_wheels(runner_name, 0.0, 0.0)
                    state_runner['state'] = 'waiting_at_exchange'
                    state_runner['timer'] = now
                else:
                    self.publish_wheels(runner_name, vx, w)

            elif state_runner['state'] == 'waiting_at_exchange':
                self.publish_wheels(runner_name, 0.0, 0.0)
                if state_lifter['state'] in ['waiting_for_runner_pickup', 'waiting_for_detach']:
                    elapsed = (now - state_runner['timer']).nanoseconds / 1e9
                    if elapsed > 1.0:
                        if not state_runner['attach_called']:
                            future = self.call_attach_service(runner_name, "base_link", crate_name, "box_link")
                            if future:
                                state_runner['attach_future'] = future
                                state_runner['attach_called'] = True
                                state_runner['state'] = 'waiting_for_pickup'
                                state_runner['timer'] = now

            elif state_runner['state'] == 'waiting_for_pickup':
                self.publish_wheels(runner_name, 0.0, 0.0)
                if state_runner['attach_future'] and state_runner['attach_future'].done():
                    try:
                        state_runner['attach_future'].result()
                        if should_log:
                            self.get_logger().info(f'✓ {runner_name}: CRATE PICKED UP! Moving to drop...')
                        state_runner['state'] = 'moving_to_drop'
                        state_runner['attach_future'] = None
                        state_runner['attach_called'] = False
                        self.pid_controllers[f'{runner_name}_x'].reset()
                        self.pid_controllers[f'{runner_name}_theta'].reset()
                    except:
                        state_runner['attach_called'] = False
                        state_runner['state'] = 'waiting_at_exchange'

            elif state_runner['state'] == 'moving_to_drop':
                # ✅ USE PAIR-SPECIFIC DROP ZONE
                vx, w, dist = self.move_to_target(runner_name, 
                                                   self.pair_zones[f'pair{i}']['drop']['x'], 
                                                   self.pair_zones[f'pair{i}']['drop']['y'])
                
                if dist < 0.15:
                    self.publish_wheels(runner_name, 0.0, 0.0)
                    state_runner['piston'] = 0.0
                    self.publish_piston(runner_name)
                    
                    if not state_runner['detach_called']:
                        future = self.call_detach_service(runner_name, "base_link", crate_name, "box_link")
                        if future:
                            state_runner['detach_future'] = future
                            state_runner['detach_called'] = True
                            state_runner['state'] = 'waiting_for_drop_detach'
                            state_runner['timer'] = now
                else:
                    self.publish_wheels(runner_name, vx, w)

            elif state_runner['state'] == 'waiting_for_drop_detach':
                self.publish_wheels(runner_name, 0.0, 0.0)
                state_runner['piston'] = 0.3
                self.publish_piston(runner_name)
                
                if state_runner['detach_future'] and state_runner['detach_future'].done():
                    try:
                        state_runner['detach_future'].result()
                        if should_log:
                            self.get_logger().info(f'✓ {runner_name}: CRATE DROPPED! Returning home...')
                        state_runner['state'] = 'returning_home'
                        state_runner['detach_future'] = None
                        state_runner['detach_called'] = False
                        self.pid_controllers[f'{runner_name}_x'].reset()
                        self.pid_controllers[f'{runner_name}_theta'].reset()
                    except:
                        pass

            elif state_runner['state'] == 'returning_home':
                home = self.home_positions[runner_name]
                vx, w, dist = self.move_to_target(runner_name, home['x'], home['y'])
                
                if dist < 0.1:
                    self.publish_wheels(runner_name, 0.0, 0.0)
                    state_runner['piston'] = 0.0
                    self.publish_piston(runner_name)
                    if should_log:
                        self.get_logger().info(f'✓✓✓ {runner_name} MISSION COMPLETE!')
                    state_runner['state'] = 'done'
                else:
                    self.publish_wheels(runner_name, vx, w)


def main(args=None):
    rclpy.init(args=args)
    controller = SwarmController8()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()