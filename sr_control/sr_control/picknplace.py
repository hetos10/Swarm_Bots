#!/usr/bin/env python3
"""
================================================================================
ARM PICK AND PLACE SERVICE NODE
ROS2 Service-Based Arm Control for Pick and Place Operations

================================================================================
FEATURES:
- Service-based arm control using linkattacher plugin
- Pick operation: Lower arm → Attach → Lift
- Place operation: Lower arm → Detach → Lift
- Async service calls for non-blocking behavior
- Detailed logging and error handling
- State machine for arm operations
- Professional structure and documentation

SERVICE INTERFACE:
- Service Type: AttachLink (custom service from hb_interfaces)
- Service Type: DetachLink (custom service from hb_interfaces)

ROBOT CONFIGURATION:
- Arm Base Link: arm_link_2 (attachment point)
- Base Angle Range: 0-180 degrees (vertical to horizontal)
- Elbow Angle Range: 0-180 degrees (folded to extended)

STATE MACHINE:
- IDLE: Waiting for commands
- LOWERING: Moving arm down
- ATTACHING: Calling attach service
- LIFTING: Moving arm up
- PLACING: Lowering arm at destination
- DETACHING: Calling detach service
- RECOVERING: Recovery from errors

================================================================================
"""

import rclpy
from rclpy.node import Node
from hb_interfaces.msg import BotCmd, BotCmdArray
from hb_interfaces.msg import Poses2D, Pose2D
from linkattacher_msgs.srv import AttachLink, DetachLink
import json
import time


class ArmPickPlaceService(Node):
    """
    ROS2 Node for ARM Pick and Place Operations
    Handles attach/detach operations using services
    """
    
    def __init__(self):
        super().__init__('arm_pick_place_service')
        
        # ==================== ROBOT CONFIGURATION ====================
        
        self.bot_id = 0  # Robot identifier
        self.current_pose = None  # Current robot pose
        self.last_time = self.get_clock().now()
        
        # ARM CONFIGURATION
        self.arm_config = {
            'base_link': 'arm_link_2',  # Attachment point on robot
            'base_angle_down': 100.0,    # Lowered position (degrees)
            'base_angle_up': 0.0,        # Raised position (degrees)
            'elbow_angle': 90.0,         # Elbow joint angle (degrees)
        }
        
        # Current arm state
        self.arm_state = {
            'base_angle': self.arm_config['base_angle_up'],
            'elbow_angle': self.arm_config['elbow_angle'],
        }
        
        # ==================== OPERATION STATE MACHINE ====================
        
        self.operation_state = 'IDLE'  # Current operation state
        self.state_timer = None  # Timer for state transitions
        self.operation_timeout = 3.0  # Timeout for operations (seconds)
        
        # Operation parameters
        self.current_operation = None  # Current pick/place operation
        self.current_crate = None  # Current crate being handled
        self.attach_future = None  # Future for attach service
        self.detach_future = None  # Future for detach service
        
        # ==================== ROS2 SERVICE CLIENTS ====================
        
        self.attach_client = self.create_client(AttachLink, '/attach_link')
        self.detach_client = self.create_client(DetachLink, '/detach_link')
        
        # Wait for services to be available
        self.wait_for_services()
        
        # ==================== ROS2 PUBLISHERS & SUBSCRIBERS ====================
        
        # Subscribe to robot pose for current position
        self.pose_subscriber = self.create_subscription(
            Poses2D,
            '/bot_pose',
            self.pose_callback,
            10
        )
        
        # Publisher for arm commands
        self.cmd_publisher = self.create_publisher(
            BotCmdArray,
            '/bot_cmd',
            10
        )
        
        # ==================== CONTROL LOOP TIMER ====================
        
        self.timer = self.create_timer(0.03, self.control_loop)
        
        self.get_logger().info('='*80)
        self.get_logger().info('🤖 ARM PICK AND PLACE SERVICE NODE INITIALIZED')
        self.get_logger().info('='*80)
        self.get_logger().info('📍 Bot ID: {}'.format(self.bot_id))
        self.get_logger().info('🔗 Attach Service: /attach_link')
        self.get_logger().info('🔗 Detach Service: /detach_link')
        self.get_logger().info('⚙️ Arm Base Link: {}'.format(self.arm_config['base_link']))
        self.get_logger().info('='*80)
    
    # ==================== SERVICE MANAGEMENT ====================
    
    def wait_for_services(self):
        """
        Wait for required services to be available
        Blocks until both attach and detach services are ready
        """
        self.get_logger().info('⏳ Waiting for attach/detach services...')
        
        # Wait for attach service
        while not self.attach_client.wait_for_service(timeout_sec=5):
            self.get_logger().warn('⚠️  Attach service not available, retrying...')
        
        self.get_logger().info('✅ Attach service available')
        
        # Wait for detach service
        while not self.detach_client.wait_for_service(timeout_sec=5):
            self.get_logger().warn('⚠️  Detach service not available, retrying...')
        
        self.get_logger().info('✅ Detach service available')
    
    def call_attach_service_async(self, robot_model, robot_link, crate_model, crate_link):
        """
        Call attach service asynchronously
        
        Args:
            robot_model (str): Name of robot model
            robot_link (str): Name of attachment link on robot
            crate_model (str): Name of crate model
            crate_link (str): Name of attachment link on crate
        
        Returns:
            Future object for async result handling
        """
        try:
            if not self.attach_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('❌ Attach service not available')
                return None
            
            # Create request with JSON data
            request = AttachLink.Request()
            request.data = json.dumps({
                'model1_name': robot_model,
                'link1_name': robot_link,
                'model2_name': crate_model,
                'link2_name': crate_link,
            })
            
            # Call service asynchronously
            future = self.attach_client.call_async(request)
            
            self.get_logger().info(
                f'🔗 Attach service called: {robot_model}:{robot_link} ← {crate_model}:{crate_link}'
            )
            
            return future
        
        except Exception as e:
            self.get_logger().error(f'❌ Attach service error: {e}')
            return None
    
    def call_detach_service_async(self, robot_model, robot_link, crate_model, crate_link):
        """
        Call detach service asynchronously
        
        Args:
            robot_model (str): Name of robot model
            robot_link (str): Name of attachment link on robot
            crate_model (str): Name of crate model
            crate_link (str): Name of attachment link on crate
        
        Returns:
            Future object for async result handling
        """
        try:
            if not self.detach_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('❌ Detach service not available')
                return None
            
            # Create request with JSON data
            request = DetachLink.Request()
            request.data = json.dumps({
                'model1_name': robot_model,
                'link1_name': robot_link,
                'model2_name': crate_model,
                'link2_name': crate_link,
            })
            
            # Call service asynchronously
            future = self.detach_client.call_async(request)
            
            self.get_logger().info(
                f'🔓 Detach service called: {robot_model}:{robot_link} → {crate_model}:{crate_link}'
            )
            
            return future
        
        except Exception as e:
            self.get_logger().error(f'❌ Detach service error: {e}')
            return None
    
    # ==================== SUBSCRIBER CALLBACKS ====================
    
    def pose_callback(self, msg: Poses2D):
        """
        Callback for /bot_pose subscription
        Updates current robot pose
        """
        for pose in msg.poses:
            if pose.id == self.bot_id:
                self.current_pose = pose
                break
    
    # ==================== ARM CONTROL ====================
    
    def move_arm(self, base_angle, elbow_angle):
        """
        Move arm to specified angles
        
        Args:
            base_angle (float): Base joint angle in degrees
            elbow_angle (float): Elbow joint angle in degrees
        """
        self.arm_state['base_angle'] = base_angle
        self.arm_state['elbow_angle'] = elbow_angle
    
    def lower_arm(self):
        """Lower arm for pickup/placement"""
        self.move_arm(
            self.arm_config['base_angle_down'],
            self.arm_config['elbow_angle']
        )
        self.get_logger().info('⬇️  Lowering arm...')
    
    def raise_arm(self):
        """Raise arm after pickup/placement"""
        self.move_arm(
            self.arm_config['base_angle_up'],
            self.arm_config['elbow_angle']
        )
        self.get_logger().info('⬆️  Raising arm...')
    
    def publish_arm_command(self):
        """
        Publish arm command to /bot_cmd topic
        """
        cmd = BotCmd()
        cmd.id = self.bot_id
        cmd.m1 = 0.0  # No wheel movement (for arm-only control)
        cmd.m2 = 0.0
        cmd.m3 = 0.0
        cmd.base = float(self.arm_state['base_angle'])
        cmd.elbow = float(self.arm_state['elbow_angle'])
        
        msg = BotCmdArray()
        msg.cmds.append(cmd)
        self.cmd_publisher.publish(msg)
    
    # ==================== PICK OPERATION ====================
    
    def start_pick_operation(self, crate_model, crate_link):
        """
        Start pick operation for a crate
        
        Args:
            crate_model (str): Model name of crate (e.g., 'crate_red_1')
            crate_link (str): Link name of crate (e.g., 'box_link_1')
        """
        self.get_logger().info('='*80)
        self.get_logger().info(f'🎯 PICK OPERATION START: {crate_model}')
        self.get_logger().info('='*80)
        
        self.current_operation = 'PICK'
        self.current_crate = {
            'model': crate_model,
            'link': crate_link,
        }
        self.operation_state = 'LOWERING'
        self.state_timer = self.get_clock().now()
    
    def start_place_operation(self, crate_model, crate_link):
        """
        Start place operation for a crate
        
        Args:
            crate_model (str): Model name of crate
            crate_link (str): Link name of crate
        """
        self.get_logger().info('='*80)
        self.get_logger().info(f'🎯 PLACE OPERATION START: {crate_model}')
        self.get_logger().info('='*80)
        
        self.current_operation = 'PLACE'
        self.current_crate = {
            'model': crate_model,
            'link': crate_link,
        }
        self.operation_state = 'LOWERING'
        self.state_timer = self.get_clock().now()
    
    # ==================== STATE MACHINE ====================
    
    def control_loop(self):
        """
        Main control loop - state machine for pick/place operations
        Runs at 33Hz (0.03s interval)
        """
        
        # Publish current arm command
        self.publish_arm_command()
        
        # If no operation, stay idle
        if self.current_operation is None:
            self.operation_state = 'IDLE'
            return
        
        now = self.get_clock().now()
        elapsed = (now - self.state_timer).nanoseconds / 1e9
        
        # ==================== PICK OPERATION STATE MACHINE ====================
        
        if self.current_operation == 'PICK':
            
            if self.operation_state == 'LOWERING':
                # Lower arm for pickup
                self.lower_arm()
                
                if elapsed > 1.5:  # Wait for arm to lower
                    self.operation_state = 'ATTACHING'
                    self.state_timer = now
                    self.get_logger().info('📍 Arm lowered. Calling attach service...')
            
            elif self.operation_state == 'ATTACHING':
                # Call attach service
                if self.attach_future is None:
                    self.attach_future = self.call_attach_service_async(
                        'hb_crystal',
                        self.arm_config['base_link'],
                        self.current_crate['model'],
                        self.current_crate['link']
                    )
                    self.state_timer = now
                
                # Check if service call completed
                if self.attach_future is not None and self.attach_future.done():
                    try:
                        result = self.attach_future.result()
                        self.get_logger().info('✅ Attachment successful!')
                        self.operation_state = 'LIFTING'
                        self.state_timer = now
                        self.attach_future = None
                    except Exception as e:
                        self.get_logger().error(f'❌ Attachment failed: {e}')
                        self.operation_state = 'RECOVERING'
                        self.state_timer = now
                        self.attach_future = None
                
                # Timeout check
                elif elapsed > self.operation_timeout:
                    self.get_logger().error('❌ Attach service timeout!')
                    self.operation_state = 'RECOVERING'
                    self.state_timer = now
                    self.attach_future = None
            
            elif self.operation_state == 'LIFTING':
                # Raise arm after attachment
                self.raise_arm()
                
                if elapsed > 1.0:  # Wait for arm to raise
                    self.get_logger().info('✅ PICK OPERATION COMPLETE')
                    self.operation_state = 'IDLE'
                    self.current_operation = None
                    self.current_crate = None
        
        # ==================== PLACE OPERATION STATE MACHINE ====================
        
        elif self.current_operation == 'PLACE':
            
            if self.operation_state == 'LOWERING':
                # Lower arm for placement
                self.lower_arm()
                
                if elapsed > 1.5:  # Wait for arm to lower
                    self.operation_state = 'DETACHING'
                    self.state_timer = now
                    self.get_logger().info('📍 Arm lowered. Calling detach service...')
            
            elif self.operation_state == 'DETACHING':
                # Call detach service
                if self.detach_future is None:
                    self.detach_future = self.call_detach_service_async(
                        'hb_crystal',
                        self.arm_config['base_link'],
                        self.current_crate['model'],
                        self.current_crate['link']
                    )
                    self.state_timer = now
                
                # Check if service call completed
                if self.detach_future is not None and self.detach_future.done():
                    try:
                        result = self.detach_future.result()
                        self.get_logger().info('✅ Detachment successful!')
                        self.operation_state = 'LIFTING'
                        self.state_timer = now
                        self.detach_future = None
                    except Exception as e:
                        self.get_logger().error(f'❌ Detachment failed: {e}')
                        self.operation_state = 'RECOVERING'
                        self.state_timer = now
                        self.detach_future = None
                
                # Timeout check
                elif elapsed > self.operation_timeout:
                    self.get_logger().error('❌ Detach service timeout!')
                    self.operation_state = 'RECOVERING'
                    self.state_timer = now
                    self.detach_future = None
            
            elif self.operation_state == 'LIFTING':
                # Raise arm after detachment
                self.raise_arm()
                
                if elapsed > 1.0:  # Wait for arm to raise
                    self.get_logger().info('✅ PLACE OPERATION COMPLETE')
                    self.operation_state = 'IDLE'
                    self.current_operation = None
                    self.current_crate = None
        
        # ==================== ERROR RECOVERY ====================
        
        if self.operation_state == 'RECOVERING':
            # Raise arm to safe position
            self.raise_arm()
            
            if elapsed > 2.0:
                self.get_logger().warn('⚠️  Operation failed. Recovering to IDLE state.')
                self.operation_state = 'IDLE'
                self.current_operation = None
                self.current_crate = None
                self.attach_future = None
                self.detach_future = None


def main(args=None):
    """
    Main entry point for ARM Pick and Place Service Node
    """
    rclpy.init(args=args)
    node = ArmPickPlaceService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down ARM Pick and Place Service...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()