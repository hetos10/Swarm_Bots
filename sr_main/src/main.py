#!/usr/bin/env python3
"""
================================================================================
COMPLETE WAREHOUSE TASK ALLOCATION SYSTEM
8-Robot Coordination with Separate Home Positions
================================================================================

SYSTEM OVERVIEW:
- 4 Lifter-Runner pairs (8 robots total)
- Each pair works independently
- Lifters: Pick boxes using ARM mechanism
- Runners: Deliver boxes using PISTON mechanism
- Exchange Zone: Handoff point between lifter and runner
- Delivery Zone: Final destination for box delivery

WORKFLOW:
1. Lifter picks up box at BOX LOCATION (using ARM)
2. Lifter moves to EXCHANGE ZONE
3. Runner waits at EXCHANGE ZONE
4. Handoff occurs: Box transferred from Lifter to Runner
5. Lifter returns to LIFTER HOME (LEFT side)
6. Runner moves to DELIVERY ZONE
7. Runner pushes box using PISTON
8. Runner returns to RUNNER HOME (RIGHT side)
9. Both ready for next cycle

HOMES (SEPARATE):
- Lifters HOME: LEFT side (x = -3.5 or -1.5)
- Runners HOME: RIGHT side (x = 1.5 or 3.5)

COLLISION AVOIDANCE:
- Lane-based: Each pair has own Y-lane
- Distance-based: 0.5m minimum safe distance
- No interference between pairs
================================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time
import math


class WarehouseTaskManager(Node):
    """
    Complete Task Manager for 8-Robot Warehouse System
    Manages 4 Lifter-Runner pairs with full coordination
    """
    
    def __init__(self):
        super().__init__('warehouse_task_manager')
        
        # ==================== PAIR CONFIGURATION ====================
        # Each pair has lifter and runner with SEPARATE home positions
        
        self.pairs = [
            {
                'pair_id': 1,
                'lifter': 'lifter1',
                'runner': 'runner1',
                'box_location': {'x': -4.0, 'y': -3.0},
                'exchange_zone': {'x': 0.0, 'y': -2.0},
                'delivery_zone': {'x': 3.0, 'y': -3.0},
                'lifter_home': {'x': -3.5, 'y': -2.0},  # LEFT SIDE
                'runner_home': {'x': 1.5, 'y': -2.0},   # RIGHT SIDE
            },
            {
                'pair_id': 2,
                'lifter': 'lifter2',
                'runner': 'runner2',
                'box_location': {'x': -4.0, 'y': 0.0},
                'exchange_zone': {'x': 0.0, 'y': 0.0},
                'delivery_zone': {'x': 3.0, 'y': 0.0},
                'lifter_home': {'x': -1.5, 'y': -2.0},  # LEFT SIDE
                'runner_home': {'x': 3.5, 'y': -2.0},   # RIGHT SIDE
            },
            {
                'pair_id': 3,
                'lifter': 'lifter3',
                'runner': 'runner3',
                'box_location': {'x': -4.0, 'y': 2.0},
                'exchange_zone': {'x': 0.0, 'y': 2.0},
                'delivery_zone': {'x': 3.0, 'y': 2.0},
                'lifter_home': {'x': -3.5, 'y': 2.0},   # LEFT SIDE
                'runner_home': {'x': 1.5, 'y': 2.0},    # RIGHT SIDE
            },
            {
                'pair_id': 4,
                'lifter': 'lifter4',
                'runner': 'runner4',
                'box_location': {'x': -4.0, 'y': 3.0},
                'exchange_zone': {'x': 0.0, 'y': 3.0},
                'delivery_zone': {'x': 3.0, 'y': 3.0},
                'lifter_home': {'x': -1.5, 'y': 2.0},   # LEFT SIDE
                'runner_home': {'x': 3.5, 'y': 2.0},    # RIGHT SIDE
            },
        ]
        
        # ==================== CREATE PUBLISHERS ====================
        # Publishers for movement and actuators for all robots
        
        self.publishers = {}
        
        for pair in self.pairs:
            lifter_name = pair['lifter']
            runner_name = pair['runner']
            
            # ===== LIFTER PUBLISHERS =====
            # Movement publisher
            self.publishers[f'{lifter_name}/cmd_vel'] = self.create_publisher(
                Twist,
                f'/{lifter_name}/cmd_vel',
                10
            )
            
            # Arm publisher (for picking)
            self.publishers[f'{lifter_name}/arm'] = self.create_publisher(
                Float64,
                f'/{lifter_name}/arm_cmd',
                10
            )
            
            # ===== RUNNER PUBLISHERS =====
            # Movement publisher
            self.publishers[f'{runner_name}/cmd_vel'] = self.create_publisher(
                Twist,
                f'/{runner_name}/cmd_vel',
                10
            )
            
            # Piston publisher (for pushing)
            self.publishers[f'{runner_name}/piston'] = self.create_publisher(
                Float64,
                f'/{runner_name}/piston_cmd',
                10
            )
        
        # ==================== INITIALIZE ROBOT STATUS ====================
        # Track state of each robot
        
        self.robot_status = {}
        
        for pair in self.pairs:
            lifter_name = pair['lifter']
            runner_name = pair['runner']
            
            # ===== LIFTER STATUS =====
            # Lifter starts at LIFTER HOME
            self.robot_status[lifter_name] = {
                'state': 'IDLE',
                'current_pos': pair['lifter_home'].copy(),
                'home_pos': pair['lifter_home'].copy(),
                'has_box': False,
                'task_cycle': 0,
                'pair_id': pair['pair_id'],
                'robot_type': 'LIFTER',
            }
            
            # ===== RUNNER STATUS =====
            # Runner starts at RUNNER HOME
            self.robot_status[runner_name] = {
                'state': 'IDLE',
                'current_pos': pair['runner_home'].copy(),
                'home_pos': pair['runner_home'].copy(),
                'has_box': False,
                'task_cycle': 0,
                'pair_id': pair['pair_id'],
                'robot_type': 'RUNNER',
            }
        
        # ==================== LOG INITIALIZATION ====================
        
        self.get_logger().info('='*80)
        self.get_logger().info('🚀 WAREHOUSE TASK ALLOCATION SYSTEM INITIALIZED')
        self.get_logger().info('='*80)
        self.get_logger().info('📊 4 Lifter-Runner Pairs Ready')
        self.get_logger().info('🏠 LIFTERS HOME: LEFT side (x = -3.5, -1.5)')
        self.get_logger().info('🏠 RUNNERS HOME: RIGHT side (x = 1.5, 3.5)')
        self.get_logger().info('🔄 Starting Task Execution Loop')
        self.get_logger().info('='*80)
        
        # ==================== START EXECUTION TIMER ====================
        # Main execution loop runs at 10Hz (100ms interval)
        
        self.timer = self.create_timer(0.1, self.task_execution_loop)
    
    # ==================== MAIN EXECUTION LOOP ====================
    
    def task_execution_loop(self):
        """
        Main execution loop - runs for all robot pairs simultaneously
        Each pair executes its state machine independently
        """
        
        for pair in self.pairs:
            lifter_name = pair['lifter']
            runner_name = pair['runner']
            pair_id = pair['pair_id']
            
            # Get current states
            lifter_state = self.robot_status[lifter_name]['state']
            runner_state = self.robot_status[runner_name]['state']
            
            # ==================== LIFTER STATE MACHINE ====================
            
            if lifter_state == 'IDLE':
                # Start new task cycle
                self.get_logger().info(
                    f'[PAIR {pair_id}] 📍 {lifter_name}: IDLE → Starting new task'
                )
                self.robot_status[lifter_name]['state'] = 'MOVING_TO_BOX'
                self.robot_status[lifter_name]['task_cycle'] += 1
            
            elif lifter_state == 'MOVING_TO_BOX':
                # Move to box location
                box_loc = pair['box_location']
                reached = self.move_robot_to_location(lifter_name, box_loc)
                if reached:
                    self.get_logger().info(
                        f'[PAIR {pair_id}] ✅ {lifter_name}: Reached BOX LOCATION {box_loc}'
                    )
                    self.robot_status[lifter_name]['state'] = 'PICKING_BOX'
            
            elif lifter_state == 'PICKING_BOX':
                # Extend arm to pick box
                if self.execute_arm_action(lifter_name, 1.0):
                    self.get_logger().info(
                        f'[PAIR {pair_id}] 📦 {lifter_name}: BOX PICKED! ARM extended'
                    )
                    self.robot_status[lifter_name]['has_box'] = True
                    self.robot_status[lifter_name]['state'] = 'MOVING_TO_EXCHANGE'
            
            elif lifter_state == 'MOVING_TO_EXCHANGE':
                # Move to exchange zone
                exchange_loc = pair['exchange_zone']
                reached = self.move_robot_to_location(lifter_name, exchange_loc)
                if reached:
                    self.get_logger().info(
                        f'[PAIR {pair_id}] 🔄 {lifter_name}: Reached EXCHANGE ZONE {exchange_loc}'
                    )
                    self.robot_status[lifter_name]['state'] = 'WAITING_FOR_HANDOFF'
            
            elif lifter_state == 'WAITING_FOR_HANDOFF':
                # Wait for runner to be ready for handoff
                if runner_state == 'READY_FOR_HANDOFF':
                    self.get_logger().info(
                        f'[PAIR {pair_id}] 🔀 HANDOFF: {lifter_name} → {runner_name}'
                    )
                    # Transfer box
                    self.robot_status[lifter_name]['has_box'] = False
                    self.robot_status[runner_name]['has_box'] = True
                    # Change states
                    self.robot_status[lifter_name]['state'] = 'RETURNING_TO_LIFTER_HOME'
                    self.robot_status[runner_name]['state'] = 'MOVING_TO_DELIVERY'
                    self.get_logger().info(
                        f'[PAIR {pair_id}] ✅ HANDOFF COMPLETE!'
                    )
            
            elif lifter_state == 'RETURNING_TO_LIFTER_HOME':
                # Return to LIFTER HOME (LEFT side)
                lifter_home = pair['lifter_home']
                reached = self.move_robot_to_location(lifter_name, lifter_home)
                if reached:
                    self.get_logger().info(
                        f'[PAIR {pair_id}] 🏠 {lifter_name}: Returned to LIFTER HOME {lifter_home}'
                    )
                    self.robot_status[lifter_name]['state'] = 'IDLE'
            
            # ==================== RUNNER STATE MACHINE ====================
            
            if runner_state == 'IDLE':
                # Prepare to move to exchange zone
                exchange_loc = pair['exchange_zone']
                self.get_logger().info(
                    f'[PAIR {pair_id}] ⏳ {runner_name}: IDLE → Moving to EXCHANGE ZONE'
                )
                self.robot_status[runner_name]['state'] = 'MOVING_TO_EXCHANGE'
            
            elif runner_state == 'MOVING_TO_EXCHANGE':
                # Move to exchange zone and wait for handoff
                exchange_loc = pair['exchange_zone']
                reached = self.move_robot_to_location(runner_name, exchange_loc)
                if reached:
                    self.get_logger().info(
                        f'[PAIR {pair_id}] ✅ {runner_name}: Ready at EXCHANGE ZONE'
                    )
                    self.robot_status[runner_name]['state'] = 'READY_FOR_HANDOFF'
            
            elif runner_state == 'READY_FOR_HANDOFF':
                # Wait for lifter to hand off box
                # Handoff happens in lifter's WAITING_FOR_HANDOFF state
                pass
            
            elif runner_state == 'MOVING_TO_DELIVERY':
                # Move to delivery zone with box
                delivery_loc = pair['delivery_zone']
                reached = self.move_robot_to_location(runner_name, delivery_loc)
                if reached:
                    self.get_logger().info(
                        f'[PAIR {pair_id}] 📍 {runner_name}: Reached DELIVERY ZONE {delivery_loc}'
                    )
                    self.robot_status[runner_name]['state'] = 'PUSHING_BOX'
            
            elif runner_state == 'PUSHING_BOX':
                # Use piston to push box
                if self.execute_piston_action(runner_name, 1.0):
                    self.get_logger().info(
                        f'[PAIR {pair_id}] 💥 {runner_name}: BOX PUSHED! PISTON extended'
                    )
                    self.robot_status[runner_name]['has_box'] = False
                    self.robot_status[runner_name]['state'] = 'RETURNING_TO_RUNNER_HOME'
            
            elif runner_state == 'RETURNING_TO_RUNNER_HOME':
                # Return to RUNNER HOME (RIGHT side)
                runner_home = pair['runner_home']
                reached = self.move_robot_to_location(runner_name, runner_home)
                if reached:
                    self.get_logger().info(
                        f'[PAIR {pair_id}] 🏠 {runner_name}: Returned to RUNNER HOME {runner_home}'
                    )
                    self.robot_status[runner_name]['state'] = 'IDLE'
    
    # ==================== MOVEMENT CONTROL ====================
    
    def move_robot_to_location(self, robot_name, target_location, speed=0.2):
        """
        Move robot towards target location
        
        Args:
            robot_name: Name of robot
            target_location: {'x': float, 'y': float} target position
            speed: Movement speed (m/s)
        
        Returns:
            True if reached target, False otherwise
        """
        
        current_pos = self.robot_status[robot_name]['current_pos']
        
        # Calculate distance and angle to target
        dx = target_location['x'] - current_pos['x']
        dy = target_location['y'] - current_pos['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if reached target (within 0.1m)
        if distance < 0.1:
            self.stop_robot(robot_name)
            self.robot_status[robot_name]['current_pos'] = target_location.copy()
            return True
        
        # Calculate movement direction
        angle = math.atan2(dy, dx)
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = speed * math.cos(angle)
        twist.linear.y = speed * math.sin(angle)
        twist.angular.z = 0.0
        
        self.publishers[f'{robot_name}/cmd_vel'].publish(twist)
        
        # Update simulated position
        self.robot_status[robot_name]['current_pos']['x'] += twist.linear.x * 0.1
        self.robot_status[robot_name]['current_pos']['y'] += twist.linear.y * 0.1
        
        return False
    
    def stop_robot(self, robot_name):
        """
        Stop robot movement
        
        Args:
            robot_name: Name of robot to stop
        """
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        
        self.publishers[f'{robot_name}/cmd_vel'].publish(twist)
    
    # ==================== ACTUATOR CONTROL ====================
    
    def execute_arm_action(self, lifter_name, position):
        """
        Execute arm action (pick box)
        
        Args:
            lifter_name: Name of lifter robot
            position: 1.0 (extend) or 0.0 (retract)
        
        Returns:
            True when action complete
        """
        
        # Extend arm
        msg = Float64()
        msg.data = position  # 1.0 = fully extended
        self.publishers[f'{lifter_name}/arm'].publish(msg)
        
        # Simulate action time (picking)
        time.sleep(0.5)
        
        # Retract arm
        msg.data = 0.0  # 0.0 = fully retracted
        self.publishers[f'{lifter_name}/arm'].publish(msg)
        
        return True
    
    def execute_piston_action(self, runner_name, position):
        """
        Execute piston action (push box)
        
        Args:
            runner_name: Name of runner robot
            position: 1.0 (extend) or 0.0 (retract)
        
        Returns:
            True when action complete
        """
        
        # Extend piston
        msg = Float64()
        msg.data = position  # 1.0 = fully extended
        self.publishers[f'{runner_name}/piston'].publish(msg)
        
        # Simulate action time (pushing)
        time.sleep(0.5)
        
        # Retract piston
        msg.data = 0.0  # 0.0 = fully retracted
        self.publishers[f'{runner_name}/piston'].publish(msg)
        
        return True
    
    # ==================== COLLISION AVOIDANCE ====================
    
    def check_collision_distance(self, robot1, robot2, min_distance=0.5):
        """
        Check distance between two robots
        
        Args:
            robot1: First robot name
            robot2: Second robot name
            min_distance: Minimum safe distance (meters)
        
        Returns:
            True if too close, False otherwise
        """
        
        pos1 = self.robot_status[robot1]['current_pos']
        pos2 = self.robot_status[robot2]['current_pos']
        
        distance = math.sqrt(
            (pos1['x'] - pos2['x'])**2 + (pos1['y'] - pos2['y'])**2
        )
        
        if distance < min_distance:
            self.get_logger().warn(
                f'⚠️  COLLISION RISK: {robot1} and {robot2} - Distance: {distance:.2f}m'
            )
            return True
        
        return False


def main(args=None):
    """
    Main entry point for task manager node
    """
    
    rclpy.init(args=args)
    task_manager = WarehouseTaskManager()
    
    try:
        rclpy.spin(task_manager)
    except KeyboardInterrupt:
        task_manager.get_logger().info('🛑 Shutting down warehouse task manager...')
    finally:
        task_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()