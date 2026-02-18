import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from linkattacher_msgs.srv import AttachLink, DetachLink
import math

class PairMission(Node):
    def __init__(self):
        super().__init__('pair_mission_node')
        
        # Poses updated based on your world file and requirements
        self.targets = {
            'P1': {'x': 4.7, 'y': 4.9},   # Red Crate Location
            'E':  {'x': 0.0, 'y': 0.0},    # Exchange Zone
            'D1': {'x': 4.7, 'y': -4.9},   # Delivery Zone
            'L_HOME': {'x': -4.5, 'y': 4.0},
            'R_HOME': {'x': -4.5, 'y': -4.0}
        }

        # Publishers (using /reference as per your hardware/controller setup)
        self.l1_vel = self.create_publisher(TwistStamped, '/lifter1/mecanum_controller/reference', 10)
        self.r1_vel = self.create_publisher(TwistStamped, '/runner1/mecanum_controller/reference', 10)
        self.l1_arm = self.create_publisher(JointTrajectory, '/lifter1/arm_controller/joint_trajectory', 10)
        self.r1_piston = self.create_publisher(Float64MultiArray, '/runner1/piston_controller/commands', 10)

        # Subscribers
        self.create_subscription(Odometry, '/lifter1/mecanum_controller/odometry', self.l1_odom_cb, 10)
        self.create_subscription(Odometry, '/runner1/mecanum_controller/odometry', self.r1_odom_cb, 10)

        # Service Clients 
        self.attach_cli = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_cli = self.create_client(DetachLink, '/DETACHLINK')

        self.l1_pose = None
        self.r1_pose = None
        self.state = "MOVE_L1_TO_P1"
        self.timer = self.create_timer(0.1, self.loop)

    def l1_odom_cb(self, msg): self.l1_pose = msg.pose.pose.position
    def r1_odom_cb(self, msg): self.r1_pose = msg.pose.pose.position

    def call_link(self, mode, m1, l1, m2, l2):
        client = self.attach_cli if mode == "attach" else self.detach_cli
        req = AttachLink.Request() if mode == "attach" else DetachLink.Request()
        req.model1_name, req.link1_name = m1, l1
        req.model2_name, req.link2_name = m2, l2
        client.call_async(req)
        self.get_logger().info(f"Service Call: {mode} {m1} to {m2}")

    def go_to(self, current, target_key, pub):
        if not current: return False
        goal = self.targets[target_key]
        dx, dy = goal['x'] - current.x, goal['y'] - current.y
        dist = math.sqrt(dx**2 + dy**2)
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'
        
        if dist > 0.15: # Tightened tolerance to 0.15 for better attachment accuracy
            cmd.twist.linear.x = 0.5 * dx
            cmd.twist.linear.y = 0.5 * dy
            pub.publish(cmd)
            return False
        
        pub.publish(cmd) # Stop
        return True

    def loop(self):
        if self.state == "MOVE_L1_TO_P1":
            if self.go_to(self.l1_pose, 'P1', self.l1_vel):
                # Move Arm Up
                msg = JointTrajectory()
                msg.joint_names = ['arm_joint_1', 'arm_joint_2']
                point = JointTrajectoryPoint(positions=[1.57, 1.57], time_from_start=rclpy.duration.Duration(seconds=2).to_msg())
                msg.points.append(point)
                self.l1_arm.publish(msg)
                self.state = "ATTACH_CRATE_L1"

        elif self.state == "ATTACH_CRATE_L1":
            self.call_link("attach", "lifter1", "arm_link2", "crate_red_1", "box_link")
            self.state = "MOVE_L1_TO_E"

        elif self.state == "MOVE_L1_TO_E":
            if self.go_to(self.l1_pose, 'E', self.l1_vel):
                self.state = "MOVE_R1_TO_E"

        elif self.state == "MOVE_R1_TO_E":
            if self.go_to(self.r1_pose, 'E', self.r1_vel):
                self.state = "TRANSFER_TO_RUNNER"

        elif self.state == "TRANSFER_TO_RUNNER":
            self.call_link("detach", "lifter1", "arm_link2", "crate_red_1", "box_link")
            self.call_link("attach", "runner1", "base_link", "crate_red_1", "box_link")
            self.get_logger().info("Transfer complete. Lifter heading home.")
            self.state = "DELIVER_TO_D1"

        elif self.state == "DELIVER_TO_D1":
            # Concurrent movement: Lifter to Home, Runner to D1
            self.go_to(self.l1_pose, 'L_HOME', self.l1_vel)
            if self.go_to(self.r1_pose, 'D1', self.r1_vel):
                self.call_link("detach", "runner1", "base_link", "crate_red_1", "box_link")
                p_msg = Float64MultiArray(data=[0.2]) # Push crate
                self.r1_piston.publish(p_msg)
                self.state = "RETURN_HOME"

        elif self.state == "RETURN_HOME":
            # Ensure Lifter finishes its journey home and Runner heads to R_HOME
            l_at_home = self.go_to(self.l1_pose, 'L_HOME', self.l1_vel)
            r_at_home = self.go_to(self.r1_pose, 'R_HOME', self.r1_vel)
            
            if l_at_home and r_at_home:
                self.get_logger().info("All robots are at home stations. Mission Complete.")
                self.state = "FINISHED"

def main():
    rclpy.init()
    node = PairMission()
    rclpy.spin(node)
    rclpy.shutdown()