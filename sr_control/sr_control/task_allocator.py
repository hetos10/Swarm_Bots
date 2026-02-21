#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hb_interfaces.msg import Poses2D, Pose2D
import numpy as np
import time
import json
from std_msgs.msg import String

# ---------------------- Crate Class --------------------------------
class Crate:
    def __init__(self, crate_id, pose=None):
        self.id = crate_id
        self.x = pose.x
        self.y = pose.y  # Offset for arm pickup height
        self.w = pose.w
        self.color = None
        self.d_zone = None
        self.assign_params(pose)
    
    def assign_params(self, pose):
        """Calculate color and d_zone based on crate ID"""
        if self.id % 3 == 0:
            self.color = "red"
            d = {'x_min': 1020, 'x_max': 1410, 'y_min': 1075, 'y_max': 1355}
        elif self.id % 3 == 1:
            self.color = "green"
            d = {'x_min': 675, 'x_max': 965, 'y_min': 1920, 'y_max': 2115}
        elif self.id % 3 == 2:
            self.color = "blue"
            d = {'x_min': 1470, 'x_max': 1762, 'y_min': 1920, 'y_max': 2115}
        else:
            d = {'x_min': 0, 'x_max': 0, 'y_min': 0, 'y_max': 0}
        
        # Calculate dropoff zone center
        d_center_x = (d['x_min'] + d['x_max']) / 2
        d_center_y = (d['y_min'] + d['y_max']) / 2
        self.d_zone = {'x': d_center_x, 'y': d_center_y, 'w': 0}
    
    def to_dict(self):
        """Convert Crate object to dictionary for JSON serialization"""
        return {
            'id': self.id,
            'x': self.x,
            'y': self.y,
            'w': self.w,
            'color': self.color,
            'd_zone': self.d_zone
        }

class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator_node')
        self.subscriber_crate = self.create_subscription(Poses2D, '/crate_pose', self.crate_cb, 10)
        self.assigned_crates_pub = self.create_publisher(String, '/assigned_crates', 10)
        self.get_logger().info("Task Allocator Node Started. Waiting for crate poses...")
        
        self.crate_received = False
        self.crates_dict = None
        
        # Timer to publish assigned crates periodically after first reception
        self.timer = self.create_timer(0.5, self.publish_timer_callback)
    
    def crate_cb(self, msg):
        """Callback when crate poses are received"""
        self.get_logger().info(f"Received crate_pose message with {len(msg.poses)} poses")
        
        # Process crates only once, on first message
        if not self.crate_received:
            self.get_logger().info("Processing and allocating tasks...")
            self.crates_dict = self.allocate_tasks(msg)
            self.crate_received = True
            self.get_logger().info(f"Allocation complete!")
            self.get_logger().info(f"Assigned Crates:\n{json.dumps(self.crates_dict, indent=2)}")
    
    def publish_timer_callback(self):
        """Publish assigned crates periodically after first reception"""
        if self.crate_received and self.crates_dict is not None:
            assigned_crates_msg = String()
            assigned_crates_msg.data = json.dumps(self.crates_dict)
            self.assigned_crates_pub.publish(assigned_crates_msg)
            self.get_logger().debug("Published assigned crates to /assigned_crates topic")
    
    def allocate_tasks(self, msg):
        """
        Allocate crate pickup and dropoff tasks to robots in round-robin fashion.
        
        Output format:
        {
            '0': [
                {'x': pickup_x, 'y': pickup_y, 'w': pickup_w, 'id': crate_id, 'color': color, 'd_zone': {'x': drop_x, 'y': drop_y, 'w': drop_w}},
                ...
            ],
            '2': [...],
            '4': [...]
        }
        """
        crate_obj_list = []
        
        # Log all received poses for debugging
        self.get_logger().info(f"Processing {len(msg.poses)} total poses from /crate_pose")
        
        # Create Crate objects from poses
        # Robot IDs are typically 0, 2, 4 - we skip those and process actual crates
        for pose in msg.poses:
            try:
                crate = Crate(pose.id, pose)
                crate_obj_list.append(crate)
                self.get_logger().info(
                    f"Crate {crate.id}: pickup=({crate.x:.1f}, {crate.y:.1f}), "
                    f"color={crate.color}, dropoff=({crate.d_zone['x']:.1f}, {crate.d_zone['y']:.1f})"
                )
            except Exception as e:
                self.get_logger().error(f"Error processing pose ID {pose.id}: {e}")
        
        self.get_logger().info(f"Total valid crates created: {len(crate_obj_list)}")
        
        # Initialize robot assignments
        assigned_crates = {
            '0': [],
            '2': [],
            '4': []
        }
        
        # Round-robin allocation: Distribute crates among robots 0, 2, 4
        robot_ids = ['0', '2', '4']
        for idx, crate in enumerate(crate_obj_list):
            robot_id = robot_ids[idx % 3]  # Cycle through robots
            assigned_crates[robot_id].append(crate.to_dict())
            self.get_logger().info(f"Allocated crate {crate.id} to robot {robot_id}")
        
        # Log final allocation
        self.get_logger().info(f"Allocation Summary:")
        for robot_id in ['0', '2', '4']:
            self.get_logger().info(f"  Robot {robot_id}: {len(assigned_crates[robot_id])} crates")
        
        return assigned_crates

def main(args=None):
    rclpy.init(args=args)
    task_allocator = TaskAllocator()
    rclpy.spin(task_allocator)
    task_allocator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()