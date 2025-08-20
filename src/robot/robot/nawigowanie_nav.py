import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import tf2_ros
import math
from collections import deque
import time

class MoveSequence(Node):
    def __init__(self):
        super().__init__('move_sequence')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF listener dla map -> base_link (Nav2/AMCL)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Bufor do wygładzania pozycji
        self.position_buffer = deque(maxlen=5)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.initial_pose = None
        self.pose_ready = False
        self.last_log_time = 0.0

        # Timer do aktualizacji pozycji
        self.create_timer(0.1, self.update_pose)

    def update_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            self.position_buffer.append((x, y, yaw))
            if len(self.position_buffer) == self.position_buffer.maxlen:
                self.current_x = sum(p[0] for p in self.position_buffer) / len(self.position_buffer)
                self.current_y = sum(p[1] for p in self.position_buffer) / len(self.position_buffer)
                self.current_yaw = sum(p[2] for p in self.position_buffer) / len(self.position_buffer)
                self.pose_ready = True

                if self.initial_pose is None:
                    self.initial_pose = (self.current_x, self.current_y, self.current_yaw)
                    self.get_logger().info(
                        f"Początkowa pozycja: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={self.current_yaw:.2f}"
                    )

                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - self.last_log_time >= 1.0:
                    self.get_logger().debug(
                        f"Wygładzona pozycja: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={self.current_yaw:.2f}"
                    )
                    self.last_log_time = current_time

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self.last_log_time >= 1.0:
                self.get_logger().warn(f"Błąd TF: {e}")
                self.last_log_time = current_time

    def wait_for_pose(self, timeout=5.0):
        start = self.get_clock().now()
        while not self.pose_ready:
            if (self.get_clock().now() - start) > Duration(seconds=timeout):
                raise RuntimeError("Nie udało się uzyskać pozycji z TF w czasie")
            rclpy.spin_once(self, timeout_sec=0.05)

    def shortest_angular_distance(self, from_angle, to_angle):
        diff = to_angle - from_angle
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def move_forward(self, distance, speed=0.3, timeout=30.0):
        self.wait_for_pose()
        start_x, start_y = self.current_x, self.current_y
        start_yaw = self.current_yaw  # Zachowaj początkowy yaw dla korekcji
        cmd = Twist()
        cmd.linear.x = speed
        kp_angular = 0.5  # Wzmocnienie dla korekcji orientacji
        start_time = self.get_clock().now()
        last_log_distance = -0.1

        while True:
            current_distance = math.sqrt((self.current_x - start_x) ** 2 + (self.current_y - start_y) ** 2)
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_distance >= distance or (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                break

            # Korekcja orientacji podczas ruchu do przodu
            yaw_error = self.shortest_angular_distance(self.current_yaw, start_yaw)
            cmd.angular.z = max(min(kp_angular * yaw_error, 0.3), -0.3)  # Ogranicz korekcję do ±0.3 rad/s

            if abs(current_distance - last_log_distance) >= 0.1 and current_time - self.last_log_time >= 1.0:
                self.get_logger().info(f"Dystans: {current_distance:.2f}/{distance:.2f}m, Błąd yaw: {yaw_error:.2f}rad")
                self.last_log_time = current_time
                last_log_distance = current_distance
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)
        self.get_logger().info(f"Zakończono ruch do przodu ({distance}m)")
        time.sleep(3.0)  # Zwiększona pauza

    def rotate(self, angle, angular_speed=0.7, timeout=60.0):
        self.wait_for_pose()
        start_yaw = self.current_yaw
        target_yaw = start_yaw + angle
        cmd = Twist()
        kp = 1.2  # Zwiększone wzmocnienie proporcjonalne
        max_angular_speed = angular_speed  # Maksymalna prędkość: 0.7 rad/s
        min_angular_speed = 0.6  # Zmniejszona minimalna prędkość: 0.4 rad/s
        start_time = self.get_clock().now()
        last_log_error = float('inf')

        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            error = self.shortest_angular_distance(self.current_yaw, target_yaw)
            turned_angle = self.shortest_angular_distance(start_yaw, self.current_yaw)
            if abs(error) < 0.02:  # Zwiększona precyzja: 0.02 rad (~1.15°)
                break
            proportional_vel = kp * error
            cmd.angular.z = max(min(proportional_vel, max_angular_speed), -max_angular_speed)
            if abs(proportional_vel) < min_angular_speed:
                cmd.angular.z = min_angular_speed * (1 if error > 0 else -1)
            current_time = self.get_clock().now().nanoseconds / 1e9
            if abs(error - last_log_error) >= 0.01 and current_time - self.last_log_time >= 1.0:
                self.get_logger().info(f"Obrót: {turned_angle:.2f}/{angle:.2f}rad, Błąd: {error:.2f}rad")
                self.last_log_time = current_time
                last_log_error = error
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        cmd.angular.z = 0.0
        self.pub.publish(cmd)
        self.get_logger().info(f"Zakończono obrót ({angle:.2f}rad)")
        time.sleep(3.0)  # Zwiększona pauza

    def calculate_position_error(self):
        if self.initial_pose is None:
            self.get_logger().warn("Początkowa pozycja nieustalona, nie można obliczyć błędu")
            return None
        final_x, final_y, final_yaw = self.current_x, self.current_y, self.current_yaw
        init_x, init_y, init_yaw = self.initial_pose
        x_error = final_x - init_x
        y_error = final_y - init_y
        position_error = math.sqrt(x_error ** 2 + y_error ** 2)
        angular_error = abs(self.shortest_angular_distance(final_yaw, init_yaw))
        self.get_logger().info(
            f"Błąd pozycji: {position_error:.3f}m (x: {x_error:.3f}m, y: {y_error:.3f}m), "
            f"Błąd kąta: {angular_error:.3f}rad"
        )
        return position_error, angular_error

    def execute_sequence(self):
        moves = [
            ("forward", 1.0),  # Dłuższy bok prostokąta
            ("turn", math.pi / 2),
            ("forward", 0.5),  # Krótki bok prostokąta
            ("turn", math.pi / 2),
            ("forward", 1.0),  # Dłuższy bok
            ("turn", math.pi / 2),
            ("forward", 0.5),  # Krótki bok
            ("turn", math.pi / 2)
        ]

        for i, (action, value) in enumerate(moves, 1):
            self.get_logger().info(f"Rozpoczynanie {action} ({value}), Krok {i}/{len(moves)}")
            try:
                if action == "forward":
                    self.move_forward(value)
                elif action == "turn":
                    self.rotate(value)
                self.get_logger().info(f"Zakończono {action} ({value}), Krok {i}/{len(moves)}")
            except Exception as e:
                self.get_logger().error(f"Błąd podczas {action} ({value}): {e}")
                break

        self.get_logger().info("Zakończono sekwencję ruchów")
        self.calculate_position_error()

def main():
    rclpy.init()
    node = MoveSequence()
    try:
        node.wait_for_pose(timeout=5.0)
        node.execute_sequence()
    except KeyboardInterrupt:
        node.get_logger().info("Przerywanie programu")
        cmd = Twist()
        node.pub.publish(cmd)
    except Exception as e:
        node.get_logger().error(f"Nieoczekiwany błąd: {e}")
        cmd = Twist()
        node.pub.publish(cmd)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()