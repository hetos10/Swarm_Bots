import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math


class ArmMotion(Node):

    def __init__(self):
        super().__init__('arm_motion')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(2.0, self.send_goal)

    def send_goal(self):
        traj = JointTrajectory()
        traj.joint_names = ['arm_joint_1', 'arm_joint_2']

        point = JointTrajectoryPoint()
        point.positions = [math.radians(90), math.radians(90)]
        point.time_from_start.sec = 2

        traj.points.append(point)

        self.pub.publish(traj)
        self.get_logger().info("Arm moved to 90°")
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = ArmMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()