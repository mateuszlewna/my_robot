import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import tf2_ros
import math
from collections import deque


class MoveSequence(Node):
    def __init__(self):
        super().__init__('move_sequence')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF listener (map -> base_link z AMCL/Nav2)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Bufor do wygładzania pozycji
        self.position_buffer = deque(maxlen=5)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.pose_ready = False

        # Pozycja początkowa do porównania
        self.initial_pose = None

        # Timer do aktualizacji pozycji
        self.create_timer(0.1, self.update_pose)

    def update_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation

            # konwersja kwaternionu na yaw
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            self.position_buffer.append((x, y, yaw))
            if len(self.position_buffer) == self.position_buffer.maxlen:
                avg_x = sum(p[0] for p in self.position_buffer) / len(self.position_buffer)
                avg_y = sum(p[1] for p in self.position_buffer) / len(self.position_buffer)
                avg_yaw = sum(p[2] for p in self.position_buffer) / len(self.position_buffer)
                self.current_x, self.current_y, self.current_yaw = avg_x, avg_y, avg_yaw
                self.pose_ready = True

                if self.initial_pose is None:
                    self.initial_pose = (self.current_x, self.current_y, self.current_yaw)
                    self.get_logger().info(
                        f"Initial Pose: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={self.current_yaw:.2f}"
                    )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def wait_for_pose(self, timeout=3.0):
        start = self.get_clock().now()
        while not self.pose_ready:
            if (self.get_clock().now() - start) > Duration(seconds=timeout):
                raise RuntimeError("Brak TF: nie udało się uzyskać pozycji w czasie")
            rclpy.spin_once(self, timeout_sec=0.05)

    def shortest_angular_distance(self, from_angle, to_angle):
        diff = to_angle - from_angle
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def move_forward(self, distance):
        self.wait_for_pose()
        start_x, start_y, _ = self.current_x, self.current_y, self.current_yaw

        cmd = Twist()
        cmd.linear.x = 0.3
        while True:
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            current_distance = math.hypot(dx, dy)
            self.get_logger().info(f"Distance: {current_distance:.2f}/{distance:.2f}m")

            if current_distance >= distance:
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
        cmd.angular.z = 0.6 if angle > 0 else -0.6

        while abs(turned_angle) < abs(angle):
            turned_angle = self.shortest_angular_distance(start_yaw, self.current_yaw)
            self.get_logger().info(f"Turned: {turned_angle:.2f}/{angle:.2f}rad")
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        cmd.angular.z = 0.0
        self.pub.publish(cmd)
        self.get_logger().info(f"Finished rotate ({angle}rad)")

    def calculate_position_error(self):
        if self.initial_pose is None:
            self.get_logger().warn("Initial pose not set, cannot calculate error")
            return None

        final_x, final_y, final_yaw = self.current_x, self.current_y, self.current_yaw
        init_x, init_y, init_yaw = self.initial_pose
        x_error = final_x - init_x
        y_error = final_y - init_y
        position_error = math.sqrt(x_error ** 2 + y_error ** 2)
        angular_error = abs(self.shortest_angular_distance(final_yaw, init_yaw))

        self.get_logger().info(
            f"Position Error: {position_error:.3f}m "
            f"(x_error: {x_error:.3f}, y_error: {y_error:.3f}), "
            f"Angular Error: {angular_error:.3f}rad"
        )
        return position_error, angular_error

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

        self.calculate_position_error()


def main():
    rclpy.init()
    node = MoveSequence()
    try:
        node.wait_for_pose(timeout=3.0)
        node.execute_sequence()
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
