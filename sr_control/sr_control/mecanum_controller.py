import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


class MecanumController(Node):

    def __init__(self):
        super().__init__('mecanum_controller')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Goal position
        self.goal_x = 2.0
        self.goal_y = 1.0

        self.current_x = 0.0
        self.current_y = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def control_loop(self):
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y

        distance = math.sqrt(dx**2 + dy**2)

        cmd = Twist()

        if distance > 0.05:
            cmd.linear.x = 0.5 * dx
            cmd.linear.y = 0.5 * dy
        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            self.get_logger().info("Goal reached")

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()