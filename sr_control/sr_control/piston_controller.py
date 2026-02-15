import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PistonMotion(Node):

    def __init__(self):
        super().__init__('piston_motion')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/piston_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(2.0, self.extend)

    def extend(self):
        traj = JointTrajectory()
        traj.joint_names = ['piston_rod_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.2]   # extend
        point.time_from_start.sec = 2

        traj.points.append(point)
        self.pub.publish(traj)

        self.get_logger().info("Piston extended")
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = PistonMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()