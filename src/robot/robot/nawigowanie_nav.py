import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
import math
from collections import deque
import time


class AMCLTest(Node):
    def __init__(self):
        super().__init__('amcl_test')

        # Publisher cmd_vel (bez leading slash!)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subskrypcja na amcl_pose z poprawnym QoS
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_callback,
            qos_profile_sensor_data
        )

        # Bufor pozycji (do wygładzania)
        self.position_buffer = deque(maxlen=5)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.initial_pose = None
        self.last_log_time = 0.0
        self.trajectory_poses = []  # do analizy błędu

    def amcl_callback(self, msg):
        # Wyciąganie pozycji i yaw z kwaternionu
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # Dodaj do bufora
        self.position_buffer.append((x, y, yaw))

        if len(self.position_buffer) == self.position_buffer.maxlen:
            self.current_x = sum(p[0] for p in self.position_buffer) / len(self.position_buffer)
            self.current_y = sum(p[1] for p in self.position_buffer) / len(self.position_buffer)
            self.current_yaw = sum(p[2] for p in self.position_buffer) / len(self.position_buffer)

            if self.initial_pose is None:
                self.initial_pose = (self.current_x, self.current_y, self.current_yaw)
                self.get_logger().info(
                    f"Initial Pose: x={self.initial_pose[0]:.2f}, "
                    f"y={self.initial_pose[1]:.2f}, yaw={self.initial_pose[2]:.2f}"
                )
                self.trajectory_poses.append((self.current_x, self.current_y, self.current_yaw))

            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self.last_log_time >= 1.0:
                self.get_logger().info(
                    f"AMCL Pose: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={self.current_yaw:.2f}"
                )
                self.last_log_time = current_time

    def shortest_angular_distance(self, from_angle, to_angle):
        diff = to_angle - from_angle
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def move_forward(self, distance):
        while len(self.position_buffer) < self.position_buffer.maxlen:
            self.get_logger().info("Waiting for position_buffer to fill...")
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        start_x, start_y = self.current_x, self.current_y
        cmd = Twist()
        cmd.linear.x = 0.3
        start_time = self.get_clock().now()
        timeout = 60.0
        last_log_distance = -0.1

        while True:
            current_distance = math.sqrt(
                (self.current_x - start_x) ** 2 + (self.current_y - start_y) ** 2
            )
            if current_distance >= distance * 0.95 or \
               (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                break

            current_time = self.get_clock().now().nanoseconds / 1e9
            if abs(current_distance - last_log_distance) >= 0.1 and \
               current_time - self.last_log_time >= 1.0:
                self.get_logger().info(
                    f"Moving forward: {current_distance:.3f}/{distance:.2f} m"
                )
                self.last_log_time = current_time
                last_log_distance = current_distance

            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        cmd.linear.x = 0.0
        self.pub.publish(cmd)
        self.get_logger().info(f"Finished move_forward ({distance} m)")
        time.sleep(3.0)
        self.trajectory_poses.append((self.current_x, self.current_y, self.current_yaw))

    def rotate(self, angle):
        start_yaw = self.current_yaw
        cmd = Twist()
        start_time = self.get_clock().now()
        timeout = 60.0
        target_yaw = start_yaw + angle
        kp = 1.0
        max_angular_vel = 0.7
        min_angular_vel = 0.6
        last_log_error = float('inf')

        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            error = self.shortest_angular_distance(self.current_yaw, target_yaw)
            turned_angle = self.shortest_angular_distance(start_yaw, self.current_yaw)

            if abs(error) < 0.1:
                break

            proportional_vel = kp * error
            cmd.angular.z = max(min(proportional_vel, max_angular_vel), -max_angular_vel)
            if abs(proportional_vel) < min_angular_vel:
                cmd.angular.z = min_angular_vel * (1 if error > 0 else -1)

            current_time = self.get_clock().now().nanoseconds / 1e9
            if abs(error - last_log_error) >= 0.01 and current_time - self.last_log_time >= 1.0:
                self.get_logger().info(
                    f"Rotating: turned={turned_angle:.3f}/{angle:.2f} rad, "
                    f"error={error:.3f}, angular vel={cmd.angular.z:.3f}"
                )
                self.last_log_time = current_time
                last_log_error = error

            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        cmd.angular.z = 0.0
        self.pub.publish(cmd)
        self.get_logger().info(f"Finished rotate ({angle:.2f} rad)")
        time.sleep(3.0)
        self.trajectory_poses.append((self.current_x, self.current_y, self.current_yaw))

    def calculate_position_error(self):
        if self.initial_pose is None or len(self.trajectory_poses) < 2:
            self.get_logger().warn("Insufficient pose data to calculate error")
            return None

        final_x, final_y, final_yaw = self.trajectory_poses[-1]
        init_x, init_y, init_yaw = self.initial_pose
        x_error = final_x - init_x
        y_error = final_y - init_y
        position_error = math.sqrt(x_error ** 2 + y_error ** 2)
        angular_error = abs(self.shortest_angular_distance(final_yaw, init_yaw))

        self.get_logger().info(
            f"Position Error: {position_error:.3f} m "
            f"(x_error: {x_error:.3f}, y_error: {y_error:.3f}), "
            f"Angular Error: {angular_error:.3f} rad"
        )
        return position_error, angular_error

    def execute_sequence(self):
        self.get_logger().info("Waiting for AMCL initialization...")
        start_time = self.get_clock().now()

        while len(self.position_buffer) < self.position_buffer.maxlen:
            self.get_logger().info("Waiting for AMCL pose messages...")
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > 10.0:
                self.get_logger().warn("AMCL initialization timeout")
                break
            time.sleep(0.1)

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

        for i, (action, value) in enumerate(moves):
            self.get_logger().info(f"Step {i+1}/{len(moves)}: {action} {value}")
            try:
                if action == "forward":
                    self.move_forward(value)
                elif action == "turn":
                    self.rotate(value)
            except Exception as e:
                self.get_logger().error(f"Error during {action}: {e}")
                break

        self.get_logger().info("Trajectory execution completed")
        self.calculate_position_error()


def main():
    rclpy.init()
    node = AMCLTest()
    try:
        node.execute_sequence()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gracefully")
        cmd = Twist()
        node.pub.publish(cmd)
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
        cmd = Twist()
        node.pub.publish(cmd)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
