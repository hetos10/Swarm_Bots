#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hb_interfaces.msg import BotCmd, BotCmdArray, Poses2D
from std_msgs.msg import String
from linkattacher_msgs.srv import AttachLink, DetachLink
import numpy as np
import json
from rclpy.executors import MultiThreadedExecutor

class SwarmController8(Node):
    all_bot_poses = {}
    all_bot_states = {}

    def __init__(self, bot_name):
        super().__init__(f'controller_{bot_name}')
        self.bot_name = bot_name
        
        # Priority: Runners (1-4) move first, Lifters (5-8) wait
        priority_map = {f'runner{i}': i for i in range(1, 5)}
        priority_map.update({f'lifter{i}': i+4 for i in range(1, 5)})
        self.priority = priority_map[bot_name]
        
        self.current_pose = None
        self.goals = []
        self.state = 'waiting'
        self.startup_delay = self.priority * 2.0
        self.start_time = self.get_clock().now()

        # PID constants
        self.kp_pos = 0.08
        self.kp_theta = 0.5

        # Namespaced Topics
        self.sub_pose = self.create_subscription(Poses2D, '/bot_pose', self.pose_cb, 10)
        self.sub_tasks = self.create_subscription(String, '/assigned_crates', self.task_cb, 10)
        self.pub_cmd = self.create_publisher(BotCmdArray, f'/{self.bot_name}/bot_cmd', 10) # Individual topic
        
        self.attach_cli = self.create_client(AttachLink, '/attach_link')
        self.timer = self.create_timer(0.05, self.control_loop)

    def pose_cb(self, msg):
        for pose in msg.poses:
            # Update shared dictionary for collision avoidance
            if pose.id == self.bot_name or str(pose.id) == self.bot_name: # Handle string/int IDs
                self.current_pose = pose
            SwarmController8.all_bot_poses[str(pose.id)] = pose

    def task_cb(self, msg):
        data = json.loads(msg.data)
        if self.bot_name in data:
            self.goals = data[self.bot_name]
            self.state = 'ready'

    def control_loop(self):
        if not self.current_pose or not self.goals:
            return
        
        # Shared state tracking
        SwarmController8.all_bot_states[self.bot_name] = self.state

        # Check Startup Delay
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.startup_delay:
            return

        # Simple Collision Avoidance
        for other_name, other_pose in SwarmController8.all_bot_poses.items():
            if other_name == self.bot_name: continue
            dist = np.sqrt((self.current_pose.x - other_pose.x)**2 + (self.current_pose.y - other_pose.y)**2)
            if dist < 400.0 and self.priority > getattr(other_pose, 'priority', 0):
                self.stop_robot()
                return

        # Execute Movement to Pickup or Exchange
        self.move_to_goal()

    def move_to_goal(self):
        target = self.goals[0]['pickup'] if self.state == 'ready' else self.goals[0]['exchange']
        dx = target['x'] - self.current_pose.x
        dy = target['y'] - self.current_pose.y
        dist = np.sqrt(dx**2 + dy**2)

        if dist < 20.0:
            self.stop_robot()
            # If Lifter, trigger attachment; if Runner, wait for hand-off
            return

        # Mecanum IK Math
        vx = self.kp_pos * dx
        vy = self.kp_pos * dy
        self.publish_velocities(vx, vy, 0.0)

    def publish_velocities(self, vx, vy, wz):
        # Your Mecanum Matrix Transformation
        # ...
        msg = BotCmdArray()
        cmd = BotCmd(id=0) # ID 0 inside namespaced topic
        cmd.m1 = float(vx + vy + wz)
        cmd.m2 = float(vx - vy - wz)
        cmd.m3 = float(-vx - vy + wz)
        msg.cmds.append(cmd)
        self.pub_cmd.publish(msg)

    def stop_robot(self):
        self.publish_velocities(0.0, 0.0, 0.0)

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    bot_names = [f'lifter{i}' for i in range(1, 5)] + [f'runner{i}' for i in range(1, 5)]
    nodes = [SwarmController8(name) for name in bot_names]
    for node in nodes: executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()