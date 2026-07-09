import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
            self.ser = serial.Serial(port, baudrate, timeout=1)
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


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
