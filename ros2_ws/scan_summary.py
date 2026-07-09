import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanSummary(Node):
    def __init__(self):
        super().__init__('scan_summary')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def angle_in_window(self, angle_deg, center_deg, width_deg):
        # Handles wraparound near -180 / +180
        diff = (angle_deg - center_deg + 180.0) % 360.0 - 180.0
        return abs(diff) <= width_deg / 2.0

    def get_min_range_in_window(self, msg, center_deg, width_deg):
        best_distance = None

        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            if self.angle_in_window(angle_deg, center_deg, width_deg):
                if best_distance is None or distance < best_distance:
                    best_distance = distance

        return best_distance

    def get_closest_object(self, msg):
        closest_distance = None
        closest_angle_deg = None

        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            if closest_distance is None or distance < closest_distance:
                closest_distance = distance
                closest_angle_deg = angle_deg

        return closest_distance, closest_angle_deg

    def scan_callback(self, msg):
        # Confirmed from your test:
        # robot front = about 180 / -180 deg
        # robot left  = about -90 deg
        # robot right = about +90 deg
        front = self.get_min_range_in_window(msg, 180, 20)
        left = self.get_min_range_in_window(msg, -90, 20)
        right = self.get_min_range_in_window(msg, 90, 20)

        closest_distance, closest_angle_deg = self.get_closest_object(msg)

        def show(value):
            if value is None:
                return "no data"
            return f"{value:.2f} m"

        if closest_distance is None:
            closest_text = "no object"
        else:
            closest_text = f"{closest_distance:.2f} m at {closest_angle_deg:.1f} deg"

        print(
            f"Robot Front: {show(front)} | "
            f"Robot Left: {show(left)} | "
            f"Robot Right: {show(right)} | "
            f"Closest: {closest_text}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanSummary()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
