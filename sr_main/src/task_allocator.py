#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hb_interfaces.msg import Poses2D, Pose2D
from std_msgs.msg import String
import json

class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator_node')
        self.subscriber_crate = self.create_subscription(Poses2D, '/crate_pose', self.crate_cb, 10)
        self.assigned_crates_pub = self.create_publisher(String, '/assigned_crates', 10)
        
        self.pairs = [
            ('lifter1', 'runner1', {'x': 1.5, 'y': 1.5}),
            ('lifter2', 'runner2', {'x': -1.5, 'y': 1.5}),
            ('lifter3', 'runner3', {'x': 1.5, 'y': -1.5}),
            ('lifter4', 'runner4', {'x': -1.5, 'y': -1.5})
        ]
        self.crate_names = ["crate_red_1", "crate_green_2", "crate_blue_3", "crate_yellow_4"]
        self.crate_received = False

    def crate_cb(self, msg):
        if not self.crate_received:
            allocated = {f'lifter{i}': [] for i in range(1, 5)}
            allocated.update({f'runner{i}': [] for i in range(1, 5)})

            # Assign crates to pairs based on your provided order
            for i, crate_name in enumerate(self.crate_names):
                lifter, runner, exchange = self.pairs[i]
                
                # Create the task packet
                task = {
                    'name': crate_name,
                    'pickup': {'x': 4.7, 'y': 4.9 - (i * 0.5)}, # Matching your provided coordinates
                    'exchange': exchange,
                    'd_zone': {'x': 0.0, 'y': 0.0} # Placeholder for final drop
                }
                
                allocated[lifter].append(task)
                allocated[runner].append(task)

            msg_out = String()
            msg_out.data = json.dumps(allocated)
            self.assigned_crates_pub.publish(msg_out)
            self.crate_received = True
            self.get_logger().info("Tasks allocated to 4 pairs successfully.")

def main():
    rclpy.init()
    node = TaskAllocator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()