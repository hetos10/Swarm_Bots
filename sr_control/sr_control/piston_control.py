#!/usr/bin/env python3
"""
Piston Joint Controller
Moves piston from 0.0 (retracted) to 0.2 (extended)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class PistonController(Node):
    def __init__(self):
        super().__init__('piston_control')
        
        self.robot_name = "runner1"
        
        # Create publisher for piston
        self.piston_pub = self.create_publisher(
            Float64,
            f'/{self.robot_name}/cmd_piston',
            10
        )
        
        self.get_logger().info('='*60)
        self.get_logger().info('Piston Controller Initialized!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Robot: {self.robot_name}')
        self.get_logger().info(f'Piston topic: /{self.robot_name}/cmd_piston')
        self.get_logger().info('Retracted position: 0.0 m')
        self.get_logger().info('Extended position: 0.2 m')
        self.get_logger().info('='*60)
    
    def move_piston(self, position):
        """
        Move piston to desired position
        
        Input:
            position: Position for piston (meters)
                     0.0 = fully retracted
                     0.2 = fully extended
        """
        msg = Float64(data=float(position))
        self.piston_pub.publish(msg)
        
        self.get_logger().info(f'Piston position: {position:.3f} m')
    
    def piston_retract(self):
        """Move piston to retracted position (0.0 m)"""
        self.get_logger().info('Moving piston to RETRACTED position (0.0 m)')
        self.move_piston(0.0)
        time.sleep(1.0)
    
    def piston_extend(self):
        """Move piston to extended position (0.2 m)"""
        self.get_logger().info('Moving piston to EXTENDED position (0.2 m)')
        self.move_piston(0.2)
        time.sleep(1.0)
    
    def piston_mid(self):
        """Move piston to mid position (0.1 m)"""
        self.get_logger().info('Moving piston to MID position (0.1 m)')
        self.move_piston(0.1)
        time.sleep(1.0)
    
    def demo_sequence(self):
        """Run a demo sequence"""
        self.get_logger().info('='*60)
        self.get_logger().info('STARTING PISTON DEMO SEQUENCE')
        self.get_logger().info('='*60)
        
        # Move to retracted
        self.get_logger().info('\n[1/4] Moving to RETRACTED...')
        self.piston_retract()
        time.sleep(2.0)
        
        # Move to mid
        self.get_logger().info('\n[2/4] Moving to MID...')
        self.piston_mid()
        time.sleep(2.0)
        
        # Move to extended
        self.get_logger().info('\n[3/4] Moving to EXTENDED...')
        self.piston_extend()
        time.sleep(2.0)
        
        # Return to retracted
        self.get_logger().info('\n[4/4] Returning to RETRACTED...')
        self.piston_retract()
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('DEMO SEQUENCE COMPLETED!')
        self.get_logger().info('='*60)

def main(args=None):
    rclpy.init(args=args)
    
    controller = PistonController()
    
    try:
        # Run demo
        controller.demo_sequence()
        
        # Keep running for manual control
        controller.get_logger().info('\nController running... (Press Ctrl+C to stop)')
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Piston controller stopped by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()