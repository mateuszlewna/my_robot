import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
import math


class MoveSequence(Node):
    def __init__(self):
        super().__init__('move_sequence')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Inicjalizacja listenera TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Timer do cyklicznego aktualizowania pozycji z TF
        self.create_timer(0.5, self.update_pose)

    def update_pose(self):
        try:
            # Pobierz transformację z map do base_link
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.current_x = trans.transform.translation.x
            self.current_y = trans.transform.translation.y
            # Konwersja kwaternionu na yaw
            q = trans.transform.rotation
            self.current_yaw = math.atan2(
                2.0 * (q.w * q.z),
                1.0 - 2.0 * (q.z * q.z)
            )
            self.get_logger().info(f"TF Pose: x={self.current_x}, y={self.current_y}, yaw={self.current_yaw}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def shortest_angular_distance(self, from_angle, to_angle):
        """Oblicza najmniejszą różnicę kątów z uwzględnieniem zakresu ±π."""
        diff = to_angle - from_angle
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def move_forward(self, distance):
        start_x, start_y = self.current_x, self.current_y
        cmd = Twist()
        cmd.linear.x = 0.5  # Stała prędkość liniowa
        start_time = self.get_clock().now()
        timeout = 20.0  # Zwiększono timeout do 20 sekund
        while math.sqrt((self.current_x - start_x) ** 2 +
                        (self.current_y - start_y) ** 2) < distance:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().warn(f"Timeout during move_forward ({distance}m)")
                break
            current_distance = math.sqrt((self.current_x - start_x) ** 2 +
                                        (self.current_y - start_y) ** 2)
            self.get_logger().info(f"Current distance: {current_distance:.3f}/{distance}m")
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.5)
        cmd.linear.x = 0.0
        self.pub.publish(cmd)
        self.get_logger().info(f"Finished move_forward ({distance}m)")

    def rotate(self, angle):
        start_yaw = self.current_yaw
        cmd = Twist()
        cmd.angular.z = 0.8 if angle > 0 else -0.8  # Zwiększono prędkość kątową
        start_time = self.get_clock().now()
        timeout = 20.0  # Zwiększono timeout do 20 sekund
        turned_angle = 0.0
        while abs(turned_angle) < abs(angle):
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().warn(f"Timeout during rotate ({angle}rad)")
                break
            turned_angle = self.shortest_angular_distance(start_yaw, self.current_yaw)
            self.get_logger().info(f"Current turned angle: {turned_angle:.3f}/{angle}rad")
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.5)
        cmd.angular.z = 0.0
        self.pub.publish(cmd)
        self.get_logger().info(f"Finished rotate ({angle}rad)")

    def execute_sequence(self):
        moves = [
            ("forward", 1.0),
            ("turn", math.pi / 2),
            ("forward", 0.5),
            ("turn", math.pi / 2),
            ("forward", 1.0),
            ("turn", math.pi / 2),
            ("forward", 0.5),
            ("turn", math.pi / 2)
        ]

        for action, value in moves:
            self.get_logger().info(f"Starting {action} ({value})")
            if action == "forward":
                self.move_forward(value)
            elif action == "turn":
                self.rotate(value)
            self.get_logger().info(f"Completed {action} ({value})")


def main():
    rclpy.init()
    node = MoveSequence()
    node.execute_sequence()
    rclpy.shutdown()


if __name__ == '__main__':
    main()