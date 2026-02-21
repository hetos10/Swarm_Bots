#!/usr/bin/env python3

'''
This Python file runs a ROS 2 node of name holonomic_pid_controller which holds the position of a holonomic robot
and drives it through a series of predefined goals using PID controllers on [x, y, θ] with collision avoidance.
'''

# ---------------------- Import Required Libraries ----------------------------
import rclpy
from rclpy.node import Node
from hb_interfaces.msg import BotCmd , BotCmdArray
from hb_interfaces.msg import Poses2D , Pose2D
from std_msgs.msg import String
import numpy as np
from linkattacher_msgs.srv import AttachLink, DetachLink
import time
import json
from rclpy.executors import MultiThreadedExecutor

# ---------------------- PID Controller Class --------------------------------
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
        self.output = self.kp * error + self.ki * self.integral + self.kd* self.derivative
        self.output =max(-self.max_out, min(self.output, self.max_out))
        self.prev_error = error
        return self.output
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

# ---------------------- Main Node Class -------------------------------------
class HolonomicPIDController(Node):
    # Shared class variable to store all bot poses for collision detection
    all_bot_poses = {}
    all_bot_states = {}  # Track what each bot is doing
    d_zone_reservations = {}  # Track which bot is heading to which d_zone
    
    def __init__(self,bot_id):
        super().__init__('holonomic_pid_controller')

        # ---------------- Robot Parameters ----------------
        self.bot_id = bot_id
        self.current_pose = None
        self.last_time = self.get_clock().now()
        self.tar_pose = None
        self.x1 = None
        self.y1 = None
        self.yaw1 = None
        self.filled = False
        self.goals = []
        self.goal_idx = 0
        self.crate_color=[]
        self.crate_id=[]
        self.assigned_crates = {}
        self.delivered_crates = set()
        self.all_crates_info = {}
        
        # Collision avoidance parameters
        self.collision_radius = 200.0  # Safety distance between robots (mm)
        self.lookahead_distance = 300.0  # Distance to check for potential collisions
        self.is_waiting_for_clearance = False
        self.wait_start_time = None
        self.max_wait_time = 10.0  # Maximum time to wait before trying alternate path
        
        self.max_vel = 1.5
        
        # Docking position
        self.dock_position = None
        if self.bot_id==0:
            self.dock_position = {'x': 1218.0,'y': 205.0,'theta': 0.0}
        elif self.bot_id==2:
            self.dock_position = {'x': 1568.0,'y': 202.0,'theta': 0.0}
        elif self.bot_id==4:
            self.dock_position = {'x': 864.0,'y': 204.0,'theta': 0.0}
            
        # Arm control parameters
        self.pickup_state = 'waiting_to_start'
        self.arm_base_angle = 90.0
        self.arm_elbow_angle = 90.0
        self.state_timer = None
        self.startup_timer = None
        self.attach_future = None
        self.detach_future = None
        self.bot_name = ['hb_crystal','hb_frostbite','hb_glacio']

        # ---------------- UNIFIED PID Parameters ----------------
        self.pid_params = {
            'position': {'kp': 0.08, 'ki': 0.0001, 'kd': 0.06, 'max_out': self.max_vel},
            'theta': {'kp': 0.5, 'ki': 0.00001, 'kd': 0.05, 'max_out': self.max_vel / 2}
        }

        self.pid_position = PID(**self.pid_params['position'])
        self.pid_theta = PID(**self.pid_params['theta'])

        # ---------------- ROS 2 Publishers & Subscribers ----------------
        self.subscriber_bot = self.create_subscription(Poses2D,'/bot_pose',self.pose_cb,10)
        self.subscriber_crates = self.create_subscription(String, '/assigned_crates', self.goal_cb, 10)
        self.subscriber_crate_poses = self.create_subscription(Poses2D, '/crate_pose', self.crate_pose_cb, 10)
        
        self.publisher = self.create_publisher(BotCmdArray, '/bot_cmd', 10)

        self.attach_cli = self.create_client(AttachLink, '/attach_link')
        self.detach_cli = self.create_client(DetachLink, '/detach_link')

        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.03, self.control_cb)

        # Startup delays with priority
        if self.bot_id == 0:
            self.startup_delay = 0.0
            self.priority = 1  # Highest priority
        elif self.bot_id == 2:
            self.startup_delay = 8.0
            self.priority = 2
        else:  # Bot 4
            self.startup_delay = 16.0
            self.priority = 3  # Lowest priority
            
        self.startup_start_time = self.get_clock().now()
        self.get_logger().info(f'Holonomic PID Controller started for Bot {self.bot_id}. Priority: {self.priority}')

    # ----------- Collision Avoidance Methods ----------------
    
    def check_collision_risk(self, my_x, my_y, target_x, target_y):
        """
        Check if moving towards target will cause collision with other bots.
        Returns: (has_collision, blocking_bot_id, should_wait)
        """
        if self.bot_id not in HolonomicPIDController.all_bot_poses:
            return False, None, False
            
        # Calculate direction vector to target
        dx = target_x - my_x
        dy = target_y - my_y
        distance_to_target = np.sqrt(dx**2 + dy**2)
        
        if distance_to_target < 1.0:
            return False, None, False
            
        # Normalized direction
        dir_x = dx / distance_to_target
        dir_y = dy / distance_to_target
        
        for other_id, other_pose in HolonomicPIDController.all_bot_poses.items():
            if other_id == self.bot_id:
                continue
                
            # Calculate distance to other bot
            other_x, other_y = other_pose['x'], other_pose['y']
            dist_to_other = np.sqrt((other_x - my_x)**2 + (other_y - my_y)**2)
            
            # Check immediate collision
            if dist_to_other < self.collision_radius:
                # Determine who should wait based on priority
                other_priority = self.get_bot_priority(other_id)
                should_wait = self.priority > other_priority
                return True, other_id, should_wait
            
            # Check if other bot is in our path (lookahead)
            if dist_to_other < self.lookahead_distance:
                # Project other bot position onto our path
                to_other_x = other_x - my_x
                to_other_y = other_y - my_y
                projection = to_other_x * dir_x + to_other_y * dir_y
                
                if 0 < projection < min(self.lookahead_distance, distance_to_target):
                    # Calculate perpendicular distance to path
                    closest_x = my_x + projection * dir_x
                    closest_y = my_y + projection * dir_y
                    perp_dist = np.sqrt((other_x - closest_x)**2 + (other_y - closest_y)**2)
                    
                    if perp_dist < self.collision_radius:
                        other_priority = self.get_bot_priority(other_id)
                        should_wait = self.priority > other_priority
                        return True, other_id, should_wait
        
        return False, None, False
    
    def get_bot_priority(self, bot_id):
        """Get priority of a bot (lower number = higher priority)"""
        if bot_id == 0:
            return 1
        elif bot_id == 2:
            return 2
        else:
            return 3
    
    def check_d_zone_collision(self, target_d_zone):
        """
        Check if another bot is already at or heading to the same D zone.
        Returns: (has_collision, should_wait)
        """
        d_zone_centers = {
            'red': [1215.0, 1215.0],
            'green': [820.0, 2017.5],
            'blue': [1616.0, 2017.5]
        }
        
        # Determine which D zone we're heading to
        my_d_zone_color = None
        for color, center in d_zone_centers.items():
            if abs(target_d_zone[0] - center[0]) < 50 and abs(target_d_zone[1] - center[1]) < 50:
                my_d_zone_color = color
                break
        
        if my_d_zone_color is None:
            return False, False
        
        # Check if any other bot is in or heading to the same D zone
        for other_id, other_state in HolonomicPIDController.all_bot_states.items():
            if other_id == self.bot_id:
                continue
            
            # Check if other bot is in D zone states
            if other_state in ['moving_to_d_zone', 'aligning_at_d_zone', 'lowering_at_d', 'detaching', 'lifting_after_detach']:
                other_pose = HolonomicPIDController.all_bot_poses.get(other_id)
                if other_pose:
                    other_x, other_y = other_pose['x'], other_pose['y']
                    
                    # Check if other bot is near the same D zone
                    d_zone_center = d_zone_centers[my_d_zone_color]
                    dist_to_d_zone = np.sqrt((other_x - d_zone_center[0])**2 + 
                                            (other_y - d_zone_center[1])**2)
                    
                    if dist_to_d_zone < 400.0:  # Within D zone area
                        # Lower priority bot should wait
                        other_priority = self.get_bot_priority(other_id)
                        should_wait = self.priority > other_priority
                        return True, should_wait
        
        return False, False
    
    def check_docking_collision(self):
        """
        Check if any bot is currently docking while we're moving.
        Returns: (has_collision, should_wait)
        """
        if not self.dock_position:
            return False, False
        
        for other_id, other_state in HolonomicPIDController.all_bot_states.items():
            if other_id == self.bot_id:
                continue
            
            # Check if other bot is in docking states
            if other_state in ['returning_to_dock', 'aligning_theta']:
                other_pose = HolonomicPIDController.all_bot_poses.get(other_id)
                if other_pose:
                    other_x, other_y = other_pose['x'], other_pose['y']
                    
                    # Get other bot's dock position
                    other_dock = None
                    if other_id == 0:
                        other_dock = {'x': 1218.0, 'y': 205.0}
                    elif other_id == 2:
                        other_dock = {'x': 1568.0, 'y': 202.0}
                    elif other_id == 4:
                        other_dock = {'x': 864.0, 'y': 204.0}
                    
                    if other_dock:
                        # Check if other bot is near its dock
                        dist_to_dock = np.sqrt((other_x - other_dock['x'])**2 + 
                                              (other_y - other_dock['y'])**2)
                        
                        if dist_to_dock < 300.0:  # Near docking area
                            # Check if we're close to that docking area
                            my_dist_to_other_dock = np.sqrt((self.current_pose.x - other_dock['x'])**2 + 
                                                           (self.current_pose.y - other_dock['y'])**2)
                            
                            if my_dist_to_other_dock < 400.0:
                                # We're too close to a docking bot
                                other_priority = self.get_bot_priority(other_id)
                                should_wait = self.priority > other_priority
                                return True, should_wait
        
        return False, False
    
    def calculate_avoidance_velocity(self, vx, vy, w, blocking_bot_id):
        """
        Calculate modified velocity to avoid collision.
        Uses potential field approach to navigate around obstacles.
        """
        if blocking_bot_id is None:
            return vx, vy, w
        
        other_pose = HolonomicPIDController.all_bot_poses.get(blocking_bot_id)
        if not other_pose:
            return vx, vy, w
        
        # Calculate repulsive force from other bot
        dx = self.current_pose.x - other_pose['x']
        dy = self.current_pose.y - other_pose['y']
        dist = np.sqrt(dx**2 + dy**2)
        
        if dist < 1.0:
            dist = 1.0
        
        # Repulsive force (inversely proportional to distance)
        repulsion_gain = 0.5
        repulsion_x = repulsion_gain * (dx / dist) * (1.0 / dist)
        repulsion_y = repulsion_gain * (dy / dist) * (1.0 / dist)
        
        # Combine with original velocity
        new_vx = 0.7 * vx + 0.3 * repulsion_x
        new_vy = 0.7 * vy + 0.3 * repulsion_y
        
        return new_vx, new_vy, w * 0.8  # Reduce rotation during avoidance

    # ----------- Service Methods ----------------
    
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
        self.get_logger().info(f'Attach service called for {model2}:{link2} to {model1}:{link1}')
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
        self.get_logger().info(f'Detach service called for {model2}:{link2} from {model1}:{link1}')
        return self.detach_future

    # ---------------- Subscriber Callbacks ----------------
    
    def pose_cb(self, msg: Poses2D):
        for pose in msg.poses:
            if pose.id == self.bot_id:
                self.current_pose = pose
                # Update shared pose dictionary
                HolonomicPIDController.all_bot_poses[self.bot_id] = {
                    'x': pose.x,
                    'y': pose.y,
                    'theta': pose.w
                }
            elif pose.id in [0, 2, 4]:
                # Store other bot poses
                HolonomicPIDController.all_bot_poses[pose.id] = {
                    'x': pose.x,
                    'y': pose.y,
                    'theta': pose.w
                }

    def goal_cb(self, msg: String):
        try:
            self.assigned_crates = json.loads(msg.data)
            if not self.filled:
                self.goals = []
                self.crate_color = []
                self.crate_id = []
                self.goal_idx = 0
                
                for crate in self.assigned_crates[str(self.bot_id)]:
                    self.goals.append([crate['x'], crate['y']])
                    self.goals.append(crate['d_zone'])
                    self.crate_color.append(crate['color'])
                    self.crate_id.append(crate['id'])
                self.filled=True
                self.get_logger().info(f'Bot {self.bot_id} assigned {len(self.assigned_crates[str(self.bot_id)])} crates')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON data: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in goal_cb: {e}')

    def crate_pose_cb(self, msg):
        for pose in msg.poses:
            if pose.id not in [0, 2, 4]:
                self.all_crates_info[pose.id] = {'x': pose.x, 'y': pose.y}
                
                if (self.goals and self.goal_idx < len(self.goals) and 
                    pose.id in self.crate_id and self.pickup_state == 'moving'):
                    crate_index = self.crate_id.index(pose.id)
                    if crate_index * 2 == self.goal_idx:
                        current_goal = self.goals[self.goal_idx]
                        distance_moved = np.sqrt((current_goal[0] - pose.x)**2 + (current_goal[1] - pose.y)**2)
                        if distance_moved > 50:
                            self.goals[self.goal_idx] = [pose.x, pose.y]
                            self.get_logger().info(f'Updated goal for moving crate {pose.id}')

    def get_current_crate_index(self):
        if not self.crate_id:
            return 0
        return min(len(self.crate_id) - 1, (self.goal_idx) // 2)

    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    # ---------------- Control Loop ----------------
    def control_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now
        if self.current_pose is None:
            return
        
        # Update shared state
        HolonomicPIDController.all_bot_states[self.bot_id] = self.pickup_state
            
        # Startup delay handling
        if self.pickup_state == 'waiting_to_start':
            elapsed = (now - self.startup_start_time).nanoseconds / 1e9
            if elapsed >= self.startup_delay:
                self.pickup_state = 'moving'
                self.get_logger().info(f'Bot {self.bot_id} startup delay complete')
            else:
                self.publish_wheel_velocities([0.0, 0.0, 0.0])
                return
        
        if self.goal_idx >= len(self.goals) and self.pickup_state == 'final_done':
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            return
            
        x, y, theta = self.current_pose.x, self.current_pose.y, self.current_pose.w
        sin_yaw = np.sin(np.deg2rad(theta))
        cos_yaw = np.cos(np.deg2rad(theta))
        x_arm = x - (130*sin_yaw)
        y_arm = y + (130*cos_yaw)
        
        if not self.goals:
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            return
            
        if self.pickup_state == 'moving':
            tar_x, tar_y = self.goals[self.goal_idx]
            error_x = tar_x - x_arm
            error_y = tar_y - y_arm
            dist_to_goal = np.sqrt(error_x**2 + error_y**2)
            
            # COLLISION AVOIDANCE CHECK
            has_collision, blocking_bot, should_wait = self.check_collision_risk(x, y, tar_x, tar_y)
            docking_collision, should_wait_dock = self.check_docking_collision()
            
            if (has_collision and should_wait) or (docking_collision and should_wait_dock):
                if not self.is_waiting_for_clearance:
                    self.is_waiting_for_clearance = True
                    self.wait_start_time = now
                    self.get_logger().info(f"Bot {self.bot_id} waiting for clearance from Bot {blocking_bot}")
                
                # Check if waited too long
                wait_duration = (now - self.wait_start_time).nanoseconds / 1e9
                if wait_duration > self.max_wait_time:
                    self.get_logger().info(f"Bot {self.bot_id} waited too long, proceeding with caution")
                    self.is_waiting_for_clearance = False
                else:
                    self.publish_wheel_velocities([0.0, 0.0, 0.0])
                    return
            else:
                self.is_waiting_for_clearance = False

            if dist_to_goal < 15.0:
                self.get_logger().info(f"Bot {self.bot_id}: Goal {self.goal_idx} reached")
                self.publish_wheel_velocities([0.0, 0.0, 0.0])
                self.arm_base_angle = 100.0
                self.arm_elbow_angle = 90.0
                self.pickup_state = 'lowering'
                self.state_timer = now
            else:
                position_error = np.sqrt(error_x**2 + error_y**2)
                desired_angle = np.arctan2(error_y, error_x)
                current_angle = np.deg2rad(theta)
                angle_error = self.normalize_angle(np.rad2deg(desired_angle - current_angle))
                
                base_vel = self.pid_position.compute(position_error, dt)
                w = self.pid_theta.compute(angle_error, dt) * 0.2
                
                vx = base_vel * np.cos(desired_angle - current_angle)
                vy = base_vel * np.sin(desired_angle - current_angle)
                
                # Apply collision avoidance
                if has_collision and not should_wait:
                    vx, vy, w = self.calculate_avoidance_velocity(vx, vy, w, blocking_bot)
                
                alpha = np.array([np.pi/6, 5*np.pi/6, 9*np.pi/6])
                M = np.array([
                    (np.cos(alpha[0] + np.pi/2), np.cos(alpha[1] + np.pi/2), np.cos(alpha[2] + np.pi/2)),
                    (np.sin(alpha[0] + np.pi/2), np.sin(alpha[1] + np.pi/2), np.sin(alpha[2] + np.pi/2)),
                    (1, 1, 1)
                ])
                m_inv = np.linalg.inv(M)
                body_vel = np.array([vx, vy, w]).reshape(3, 1)
                wheel_vel = np.matmul(m_inv, body_vel).flatten()
                self.publish_wheel_velocities(wheel_vel)
        
        elif self.pickup_state == 'lowering':
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            elapsed = (now - self.state_timer).nanoseconds / 1e9
            if elapsed > 1.5:
                if self.attach_cli.wait_for_service(timeout_sec=3.0):
                    crate_index = self.get_current_crate_index()
                    future = self.call_attach_service_async(
                        self.bot_name[int(self.bot_id*0.5)], 
                        "arm_link_2", 
                        f"crate_{self.crate_color[crate_index]}_{self.crate_id[crate_index]}", 
                        f"box_link_{self.crate_id[crate_index]}"
                    )
                    if future is not None:
                        self.attach_future = future
                        self.pickup_state = 'attaching'
                        self.state_timer = now
                else:
                    self.state_timer = now 
        
        elif self.pickup_state == 'attaching':
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            if self.attach_future is not None and self.attach_future.done():
                try:
                    result = self.attach_future.result()
                    self.arm_base_angle = 90.0
                    self.arm_elbow_angle = 90.0
                    self.pickup_state = 'lifting'
                    self.state_timer = now
                    self.attach_future = None
                except Exception as e:
                    self.get_logger().error(f"Attachment failed: {e}")
                    self.pickup_state = 'lowering'
                    self.attach_future = None

        elif self.pickup_state == 'lifting':
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            elapsed = (now - self.state_timer).nanoseconds / 1e9
            if elapsed > 0.5:
                self.arm_base_angle = 0.0
                self.arm_elbow_angle = 90.0
                self.pickup_state = 'moving_to_top'
                self.state_timer = now

        elif self.pickup_state == 'moving_to_top':
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            elapsed = (now - self.state_timer).nanoseconds / 1e9
            if elapsed > 1.0:
                self.pickup_state = 'moving_to_d_zone'
                self.goal_idx +=1
                self.pid_position.reset()
                self.pid_theta.reset()

        elif self.pickup_state == 'moving_to_d_zone':
            tar_x = self.goals[self.goal_idx][0]
            tar_y = self.goals[self.goal_idx][1]
            error_x = tar_x - x
            error_y = tar_y - y
            dist_to_d = np.sqrt(error_x**2 + error_y**2)
            
            # D-ZONE COLLISION AVOIDANCE
            d_zone_collision, should_wait_d = self.check_d_zone_collision([tar_x, tar_y])
            has_collision, blocking_bot, should_wait = self.check_collision_risk(x, y, tar_x, tar_y)
            
            if (d_zone_collision and should_wait_d) or (has_collision and should_wait):
                if not self.is_waiting_for_clearance:
                    self.is_waiting_for_clearance = True
                    self.wait_start_time = now
                    self.get_logger().info(f"Bot {self.bot_id} waiting at D-zone approach")
                
                wait_duration = (now - self.wait_start_time).nanoseconds / 1e9
                if wait_duration > self.max_wait_time:
                    self.get_logger().info(f"Bot {self.bot_id} proceeding to D-zone with caution")
                    self.is_waiting_for_clearance = False
                else:
                    self.publish_wheel_velocities([0.0, 0.0, 0.0])
                    return
            else:
                self.is_waiting_for_clearance = False

            if dist_to_d < 50.0:
                self.get_logger().info(f"Reached D zone! Position: ({x:.1f}, {y:.1f})")
                self.publish_wheel_velocities([0.0, 0.0, 0.0])
                self.pickup_state = 'aligning_at_d_zone'  # NEW STATE: Align yaw to 0 before detaching
                self.pid_theta.reset()
            else:
                position_error = np.sqrt(error_x**2 + error_y**2)
                desired_angle = np.arctan2(error_y, error_x)
                current_angle = np.deg2rad(theta)
                angle_error = self.normalize_angle(np.rad2deg(desired_angle - current_angle))
                
                base_vel = self.pid_position.compute(position_error, dt)
                w = self.pid_theta.compute(angle_error, dt) * 0.2
                
                vx = base_vel * np.cos(desired_angle - current_angle)
                vy = base_vel * np.sin(desired_angle - current_angle)
                
                # Apply collision avoidance
                if has_collision and not should_wait:
                    vx, vy, w = self.calculate_avoidance_velocity(vx, vy, w, blocking_bot)
                
                alpha = np.array([np.pi/6, 5*np.pi/6, 9*np.pi/6])
                M = np.array([
                    (np.cos(alpha[0] + np.pi/2), np.cos(alpha[1] + np.pi/2), np.cos(alpha[2] + np.pi/2)),
                    (np.sin(alpha[0] + np.pi/2), np.sin(alpha[1] + np.pi/2), np.sin(alpha[2] + np.pi/2)),
                    (1, 1, 1)
                ])
                m_inv = np.linalg.inv(M)
                body_vel = np.array([vx, vy, w]).reshape(3, 1)
                wheel_vel = np.matmul(m_inv, body_vel).flatten()
                self.publish_wheel_velocities(wheel_vel)

        # NEW STATE: Align yaw to 0 at D-zone before detaching
        elif self.pickup_state == 'aligning_at_d_zone':
            tar_theta = 0.0  # Target yaw = 0 degrees
            error_theta = self.normalize_angle(theta - tar_theta)
            
            if abs(error_theta) < 2.0:  # Within tolerance
                self.publish_wheel_velocities([0.0, 0.0, 0.0])
                self.arm_base_angle = 100.0
                self.arm_elbow_angle = 90.0
                self.pickup_state = 'lowering_at_d'
                self.state_timer = now
                self.get_logger().info(f"Bot {self.bot_id} aligned to yaw=0 at D-zone")
            else:
                # Only rotate to align yaw, no translation
                vx = 0.0
                vy = 0.0
                w = self.pid_theta.compute(error_theta, dt) * 0.5
                
                alpha = np.array([np.pi/6, 5*np.pi/6, 9*np.pi/6])
                M = np.array([
                    (np.cos(alpha[0] + np.pi/2), np.cos(alpha[1] + np.pi/2), np.cos(alpha[2] + np.pi/2)),
                    (np.sin(alpha[0] + np.pi/2), np.sin(alpha[1] + np.pi/2), np.sin(alpha[2] + np.pi/2)),
                    (1, 1, 1)
                ])
                m_inv = np.linalg.inv(M)
                body_vel = np.array([vx, vy, w]).reshape(3, 1)
                wheel_vel = np.matmul(m_inv, body_vel).flatten()
                self.publish_wheel_velocities(wheel_vel)

        elif self.pickup_state == 'lowering_at_d':
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            elapsed = (now - self.state_timer).nanoseconds / 1e9
            if elapsed > 1.5:
                if self.detach_cli.wait_for_service(timeout_sec=3.0):
                    crate_index = self.get_current_crate_index()
                    future = self.call_detach_service_async(
                        self.bot_name[int(self.bot_id * 0.5)], 
                        "arm_link_2", 
                        f"crate_{self.crate_color[crate_index]}_{self.crate_id[crate_index]}", 
                        f"box_link_{self.crate_id[crate_index]}"
                    )
                    if future is not None:
                        self.detach_future = future
                        self.pickup_state = 'detaching'
                        self.state_timer = now
                else:
                    self.state_timer = now

        elif self.pickup_state == 'detaching':
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            if self.detach_future is not None and self.detach_future.done():
                try:
                    result = self.detach_future.result()
                    self.arm_base_angle = 90.0
                    self.arm_elbow_angle = 90.0
                    self.pickup_state = 'lifting_after_detach'
                    self.state_timer = now
                    self.detach_future = None
                except Exception as e:
                    self.get_logger().error(f"Detachment failed: {e}")
                    self.pickup_state = 'lowering_at_d'
                    self.detach_future = None

        elif self.pickup_state == 'lifting_after_detach':
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            elapsed = (now - self.state_timer).nanoseconds / 1e9
            if elapsed > 1.0:
                current_crate_index = self.get_current_crate_index()
                if current_crate_index < len(self.crate_id):
                    current_crate_id = self.crate_id[current_crate_index]
                    self.delivered_crates.add(current_crate_id)
                
                if self.goal_idx < len(self.goals) - 1:
                    self.goal_idx += 1
                    self.pickup_state = "moving"
                else:
                    self.pickup_state = "returning_to_dock"
                
                self.pid_position.reset()
                self.pid_theta.reset()

        elif self.pickup_state == 'returning_to_dock':
            tar_x = self.dock_position['x']
            tar_y = self.dock_position['y']
            error_x = tar_x - x
            error_y = tar_y - y
            dist_to_dock = np.sqrt(error_x**2 + error_y**2)
            
            # DOCKING COLLISION AVOIDANCE
            has_collision, blocking_bot, should_wait = self.check_collision_risk(x, y, tar_x, tar_y)
            
            if has_collision and should_wait:
                if not self.is_waiting_for_clearance:
                    self.is_waiting_for_clearance = True
                    self.wait_start_time = now
                    self.get_logger().info(f"Bot {self.bot_id} waiting before docking")
                
                wait_duration = (now - self.wait_start_time).nanoseconds / 1e9
                if wait_duration > self.max_wait_time:
                    self.is_waiting_for_clearance = False
                else:
                    self.publish_wheel_velocities([0.0, 0.0, 0.0])
                    return
            else:
                self.is_waiting_for_clearance = False

            if dist_to_dock < 30.0:
                self.publish_wheel_velocities([0.0, 0.0, 0.0])
                self.pid_theta.reset()
                self.pickup_state = 'aligning_theta'
            else:
                position_error = np.sqrt(error_x**2 + error_y**2)
                desired_angle = np.arctan2(error_y, error_x)
                current_angle = np.deg2rad(theta)
                angle_error = self.normalize_angle(np.rad2deg(desired_angle - current_angle))
                
                base_vel = self.pid_position.compute(position_error, dt)
                w = self.pid_theta.compute(angle_error, dt) * 0.2
                
                vx = base_vel * np.cos(desired_angle - current_angle)
                vy = base_vel * np.sin(desired_angle - current_angle)
                
                # Apply collision avoidance
                if has_collision and not should_wait:
                    vx, vy, w = self.calculate_avoidance_velocity(vx, vy, w, blocking_bot)
                
                alpha = np.array([np.pi/6, 5*np.pi/6, 9*np.pi/6])
                M = np.array([
                    (np.cos(alpha[0] + np.pi/2), np.cos(alpha[1] + np.pi/2), np.cos(alpha[2] + np.pi/2)),
                    (np.sin(alpha[0] + np.pi/2), np.sin(alpha[1] + np.pi/2), np.sin(alpha[2] + np.pi/2)),
                    (1, 1, 1)
                ])
                m_inv = np.linalg.inv(M)
                body_vel = np.array([vx, vy, w]).reshape(3, 1)
                wheel_vel = np.matmul(m_inv, body_vel).flatten()
                self.publish_wheel_velocities(wheel_vel)

        elif self.pickup_state == 'aligning_theta':
            tar_theta = 0.0  # Target yaw = 0 degrees
            error_theta = self.normalize_angle(theta - tar_theta)
            
            if abs(error_theta) < 2.0:  # Within tolerance
                self.publish_wheel_velocities([0.0, 0.0, 0.0])
                self.pickup_state = 'final_done'
                self.get_logger().info(f"Bot {self.bot_id} successfully docked with yaw=0")
            else:
                # Only rotate to align yaw, no translation
                vx = 0.0
                vy = 0.0
                w = self.pid_theta.compute(error_theta, dt) * 0.5
                
                alpha = np.array([np.pi/6, 5*np.pi/6, 9*np.pi/6])
                M = np.array([
                    (np.cos(alpha[0] + np.pi/2), np.cos(alpha[1] + np.pi/2), np.cos(alpha[2] + np.pi/2)),
                    (np.sin(alpha[0] + np.pi/2), np.sin(alpha[1] + np.pi/2), np.sin(alpha[2] + np.pi/2)),
                    (1, 1, 1)
                ])
                m_inv = np.linalg.inv(M)
                body_vel = np.array([vx, vy, w]).reshape(3, 1)
                wheel_vel = np.matmul(m_inv, body_vel).flatten()
                self.publish_wheel_velocities(wheel_vel)
                
    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel):
        cmd = BotCmd()
        cmd.id = self.bot_id
        cmd.m1 = float(wheel_vel[0])*30.0
        cmd.m2 = float(wheel_vel[1])*30.0
        cmd.m3 = float(wheel_vel[2])*30.0
        cmd.base = float(self.arm_base_angle)
        cmd.elbow = float(self.arm_elbow_angle)

        msg = BotCmdArray()
        msg.cmds.append(cmd)
        self.publisher.publish(msg)

# ---------------------- Main Function -------------------------------------

def main(args=None):
    rclpy.init(args=args)

    # Create three controller nodes with bot_id 0, 2, 4
    node0 = HolonomicPIDController(0)
    node2 = HolonomicPIDController(2)
    node4 = HolonomicPIDController(4)

    # Multithreaded executor to run timers/callbacks of all nodes concurrently
    executor = MultiThreadedExecutor()

    # Add nodes to executor
    executor.add_node(node0)
    executor.add_node(node2)
    executor.add_node(node4)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # shutdown executor and cleanup nodes
        executor.shutdown()
        node0.destroy_node()
        node2.destroy_node()
        node4.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()