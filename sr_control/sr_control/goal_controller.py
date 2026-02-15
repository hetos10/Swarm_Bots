import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math


class GoToGoal(Node):

    def __init__(self):
        super().__init__('go_to_goal')

        # Publisher (TwistStamped)
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/mecanum_controller/reference',
            10
        )

        # Subscriber (odometry)
        self.create_subscription(
            Odometry,
            '/mecanum_controller/odometry',
            self.odom_callback,
            10
        )

        # Target pose
        self.goal_x = 1.0
        self.goal_y = 0.0

        self.x = 0.0
        self.y = 0.0

        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def control_loop(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        dist = math.sqrt(dx * dx + dy * dy)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        if dist > 0.05:
            msg.twist.linear.x = 0.5 * dx
            msg.twist.linear.y = 0.5 * dy
        else:
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            self.get_logger().info("Goal reached")

        msg.twist.angular.z = 0.0

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()