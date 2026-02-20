#!/usr/bin/env python3
"""
Arm Joint Controller
Moves arm joints from 0° to 90° (0 to 1.57 radians)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_control')
        
        self.robot_name = "lifter1"
        
        # Create publishers for arm joints
        self.arm_joint_1_pub = self.create_publisher(
            Float64,
            f'/{self.robot_name}/cmd_arm_joint_1',
            10
        )
        
        self.arm_joint_2_pub = self.create_publisher(
            Float64,
            f'/{self.robot_name}/cmd_arm_joint_2',
            10
        )
        
        self.get_logger().info('='*60)
        self.get_logger().info('Arm Controller Initialized!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Robot: {self.robot_name}')
        self.get_logger().info(f'Joint 1 topic: /{self.robot_name}/cmd_arm_joint_1')
        self.get_logger().info(f'Joint 2 topic: /{self.robot_name}/cmd_arm_joint_2')
        self.get_logger().info('='*60)
    
    def move_arm(self, joint_1_pos, joint_2_pos):
        """
        Move arm to desired position
        
        Input:
            joint_1_pos: Position for arm_joint_1 (radians)
            joint_2_pos: Position for arm_joint_2 (radians)
        """
        msg1 = Float64(data=float(joint_1_pos))
        msg2 = Float64(data=float(joint_2_pos))
        
        self.arm_joint_1_pub.publish(msg1)
        self.arm_joint_2_pub.publish(msg2)
        
        self.get_logger().info(f'Arm position: J1={joint_1_pos:.2f} rad, J2={joint_2_pos:.2f} rad')
    
    def arm_home(self):
        """Move arm to home position (0°, 0°)"""
        self.get_logger().info('Moving arm to HOME position (0°, 0°)')
        self.move_arm(0.0, 0.0)
        time.sleep(1.0)
    
    def arm_extended(self):
        """Move arm to extended position (90°, 90°)"""
        self.get_logger().info('Moving arm to EXTENDED position (90°, 90°)')
        self.move_arm(1.57, 1.57)  # 1.57 radians = 90 degrees
        time.sleep(1.0)
    
    def arm_mid(self):
        """Move arm to mid position (45°, 45°)"""
        self.get_logger().info('Moving arm to MID position (45°, 45°)')
        self.move_arm(0.785, 0.785)  # 0.785 radians = 45 degrees
        time.sleep(1.0)
    
    def demo_sequence(self):
        """Run a demo sequence"""
        self.get_logger().info('='*60)
        self.get_logger().info('STARTING ARM DEMO SEQUENCE')
        self.get_logger().info('='*60)
        
        # Move to home
        self.get_logger().info('\n[1/4] Moving to HOME...')
        self.arm_home()
        time.sleep(2.0)
        
        # Move to mid
        self.get_logger().info('\n[2/4] Moving to MID...')
        self.arm_mid()
        time.sleep(2.0)
        
        # Move to extended
        self.get_logger().info('\n[3/4] Moving to EXTENDED...')
        self.arm_extended()
        time.sleep(2.0)
        
        # Return to home
        self.get_logger().info('\n[4/4] Returning to HOME...')
        self.arm_home()
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('DEMO SEQUENCE COMPLETED!')
        self.get_logger().info('='*60)

def main(args=None):
    rclpy.init(args=args)
    
    controller = ArmController()
    
    try:
        # Run demo
        controller.demo_sequence()
        
        # Keep running for manual control
        controller.get_logger().info('\nController running... (Press Ctrl+C to stop)')
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Arm controller stopped by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()