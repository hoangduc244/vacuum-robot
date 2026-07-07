import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotNode(Node):
    def __init__(self):
        super().__init__("robot_node")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("Robot node started. Publishing fake /cmd_vel commands.")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.10
        msg.angular.z = 0.0

        self.cmd_pub.publish(msg)
        self.get_logger().info("Published /cmd_vel: forward 0.10 m/s")


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
