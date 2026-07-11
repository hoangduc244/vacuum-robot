import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial


class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.get_logger().info(f'Trying to open serial port: {port}')

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            self.get_logger().info('Serial connected successfully.')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'Could not open serial port: {e}')

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.serial_timer = self.create_timer(0.02, self.read_serial)

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        command = f'V:{linear:.2f},W:{angular:.2f}\n'
        self.get_logger().info(f'Sending to Arduino: {command.strip()}')

        if self.ser is not None:
            try:
                self.ser.write(command.encode())
            except Exception as e:
                self.get_logger().error(f'Serial write failed: {e}')

    def read_serial(self):
        if self.ser is None:
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode(errors='ignore').strip()

                if line.startswith('ODOM:'):
                    self.handle_odom_line(line)

        except Exception as e:
            self.get_logger().error(f'Serial read failed: {e}')

    def handle_odom_line(self, line):
        try:
            data = line.replace('ODOM:', '').split(',')

            if len(data) != 3:
                return

            x = float(data[0])
            y = float(data[1])
            theta = float(data[2])

            self.publish_odom(x, y, theta)

        except Exception as e:
            self.get_logger().warn(f'Bad odom line: {line} | {e}')

    def publish_odom(self, x, y, theta):
        now = self.get_clock().now().to_msg()

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()