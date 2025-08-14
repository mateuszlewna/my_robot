import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
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
        self.pose_ready = False

        # Timer do cyklicznego aktualizowania pozycji z TF
        self.create_timer(0.1, self.update_pose)

    def update_pose(self):
        try:
            # Pobierz transformację z map do base_link
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.current_x = trans.transform.translation.x
            self.current_y = trans.transform.translation.y

            # Konwersja kwaternionu na yaw
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

            self.pose_ready = True
            self.get_logger().info(f"TF Pose: x={self.current_x}, y={self.current_y}, yaw={self.current_yaw}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def wait_for_pose(self, timeout=2.0):
        """Czeka aż pojawi się pierwszy poprawny TF."""
        start = self.get_clock().now()
        while not self.pose_ready:
            if (self.get_clock().now() - start) > Duration(seconds=timeout):
                raise RuntimeError("Brak TF: nie udało się uzyskać pozycji w czasie")
            rclpy.spin_once(self, timeout_sec=0.05)

    def get_pose_now(self):
        """Pobiera aktualny TF, żeby ustawić punkt startowy."""
        if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=0.5)):
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw
        else:
            return self.current_x, self.current_y, self.current_yaw

    def shortest_angular_distance(self, from_angle, to_angle):
        """Oblicza najmniejszą różnicę kątów z uwzględnieniem zakresu ±π."""
        diff = to_angle - from_angle
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def move_forward(self, distance):
        self.wait_for_pose()
        start_x, start_y, _ = self.get_pose_now()

        cmd = Twist()
        cmd.linear.x = 0.5  # Stała prędkość liniowa
        start_time = self.get_clock().now()
        timeout = 20.0

        while True:
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            current_distance = math.hypot(dx, dy)
            self.get_logger().info(f"Current distance: {current_distance:.3f}/{distance}m")

            if current_distance >= distance:
                break

            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().warn(f"Timeout during move_forward ({distance}m)")
                break

            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        cmd.linear.x = 0.0
        self.pub.publish(cmd)
        self.get_logger().info(f"Finished move_forward ({distance}m)")

    def rotate(self, angle):
        self.wait_for_pose()
        start_yaw = self.current_yaw
        turned_angle = 0.0

        cmd = Twist()
        cmd.angular.z = 1.0 if angle > 0 else -1.0
        start_time = self.get_clock().now()
        timeout = 20.0

        while abs(turned_angle) < abs(angle):
            turned_angle = self.shortest_angular_distance(start_yaw, self.current_yaw)
            self.get_logger().info(f"Current turned angle: {turned_angle:.3f}/{angle}rad")

            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().warn(f"Timeout during rotate ({angle}rad)")
                break

            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

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

    try:
        node.wait_for_pose(timeout=3.0)
    except Exception as e:
        node.get_logger().error(str(e))

    node.execute_sequence()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
