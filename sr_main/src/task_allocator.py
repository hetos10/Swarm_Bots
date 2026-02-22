#!/usr/bin/env python3

'''
Task Allocator for 8-Robot Swarm
- Pairs: (lifter1, runner1), (lifter2, runner2), (lifter3, runner3), (lifter4, runner4)
- Each pair gets ONE crate to transport
- Assigns pickup, exchange, and drop zones
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator_node')

        # ========== ROBOT PAIRINGS ==========
        # Each lifter is paired with a runner
        self.pairs = [
            {
                'lifter': 'lifter1',
                'runner': 'runner1',
                'crate': 'crate_red_1',
                'pickup': {'x': 4.9, 'y': 4.7},        # Crate pickup location
                'exchange': {'x': 1.5, 'y': 1.5},      # Exchange zone (lifter gives to runner)
                'drop': {'x': 4.9, 'y': -4.9}           # Final drop zone
            },
            {
                'lifter': 'lifter2',
                'runner': 'runner2',
                'crate': 'crate_green_2',
                'pickup': {'x': 4.9, 'y': 4.2},
                'exchange': {'x': -1.5, 'y': 1.5},
                'drop': {'x': 4.6, 'y': -4.9}
            },
            {
                'lifter': 'lifter3',
                'runner': 'runner3',
                'crate': 'crate_blue_3',
                'pickup': {'x': 4.9, 'y': 3.7},
                'exchange': {'x': 1.5, 'y': -1.5},
                'drop': {'x': 4.3, 'y': -4.9}
            },
            {
                'lifter': 'lifter4',
                'runner': 'runner4',
                'crate': 'crate_yellow_4',
                'pickup': {'x': 4.9, 'y': 3.2},
                'exchange': {'x': -1.5, 'y': -1.5},
                'drop': {'x': 4.0, 'y': -4.9}
            }
        ]

        # ========== PUBLISHERS ==========
        self.pub_lifter_tasks = self.create_publisher(String, '/lifter_tasks', 10)
        self.pub_runner_tasks = self.create_publisher(String, '/runner_tasks', 10)
        
        # ========== TIMER ==========
        # Allocate tasks immediately on startup
        self.timer = self.create_timer(0.1, self.allocate_tasks_once)
        self.allocated = False

        self.get_logger().info('='*70)
        self.get_logger().info('Task Allocator Started')
        self.get_logger().info('Pairing 4 Lifters with 4 Runners')
        self.get_logger().info('='*70)

    def allocate_tasks_once(self):
        """Allocate tasks ONCE on startup"""
        if self.allocated:
            return
        
        # ========== BUILD TASK DICTIONARIES ==========
        lifter_tasks = {}
        runner_tasks = {}

        for pair in self.pairs:
            lifter_name = pair['lifter']
            runner_name = pair['runner']
            
            # LIFTER TASK: Pick up crate and deliver to exchange
            lifter_task = {
                'robot_id': lifter_name,
                'crate': pair['crate'],
                'pickup': pair['pickup'],
                'exchange': pair['exchange'],
                'home': self.get_home_position(lifter_name)
            }
            lifter_tasks[lifter_name] = lifter_task
            
            # RUNNER TASK: Get crate from exchange and deliver to drop
            runner_task = {
                'robot_id': runner_name,
                'crate': pair['crate'],
                'exchange': pair['exchange'],
                'drop': pair['drop'],
                'home': self.get_home_position(runner_name)
            }
            runner_tasks[runner_name] = runner_task

        # ========== PUBLISH TASKS ==========
        lifter_msg = String()
        lifter_msg.data = json.dumps(lifter_tasks, indent=2)
        self.pub_lifter_tasks.publish(lifter_msg)
        
        runner_msg = String()
        runner_msg.data = json.dumps(runner_tasks, indent=2)
        self.pub_runner_tasks.publish(runner_msg)

        # ========== LOG ALLOCATION ==========
        self.get_logger().info('✓ Tasks allocated to all 8 robots!')
        self.get_logger().info('')
        self.get_logger().info('PAIRS:')
        for pair in self.pairs:
            self.get_logger().info(f'  Pair: {pair["lifter"]} ↔ {pair["runner"]} | Crate: {pair["crate"]}')
        
        self.allocated = True

    def get_home_position(self, robot_name):
        """Get home position for each robot based on type"""
        if 'lifter' in robot_name:
            # Lifter home positions
            lifter_num = int(robot_name[-1])
            if lifter_num == 1:
                return {'x': -4.5, 'y': 4.0}
            elif lifter_num == 2:
                return {'x': -3.5, 'y': 4.0}
            elif lifter_num == 3:
                return {'x': -4.5, 'y': 3.0}
            else:  # lifter_num == 4
                return {'x': -3.5, 'y': 3.0}
        else:
            # Runner home positions
            runner_num = int(robot_name[-1])
            if runner_num == 1:
                return {'x': -4.5, 'y': -4.0}
            elif runner_num == 2:
                return {'x': -3.5, 'y': -4.0}
            elif runner_num == 3:
                return {'x': -4.5, 'y': -3.0}
            else:  # runner_num == 4
                return {'x': -3.5, 'y': -3.0}


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    
    # Spin once to allocate tasks, then shutdown
    rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()