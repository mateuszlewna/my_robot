    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    import tf2_ros
    import math
    from collections import deque
    import time
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    import numpy as np

    class MoveSequence(Node):
        def __init__(self):
            super().__init__('move_sequence')
            self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

            # Initialize TF listener
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

            # Subscribe to LiDAR data (for monitoring, optional)
            self.scan_sub = self.create_subscription(
                LaserScan, '/scan', self.scan_callback, 10)

            # Subscribe to odometry (e.g., from SLAM or AMCL)
            self.odom_sub = self.create_subscription(
                Odometry, '/odom', self.odom_callback, 10)

            # Position buffer for smoothing
            self.position_buffer = deque(maxlen=5)
            self.current_x = 0.0
            self.current_y = 0.0
            self.current_yaw = 0.0
            self.initial_pose = None
            self.last_log_time = 0.0  # Global rate limiter for logging

            # Timer to update pose
            self.create_timer(0.1, self.update_pose)

        def scan_callback(self, msg):
            pass  # No logging to reduce clutter

        def odom_callback(self, msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
            self.position_buffer.append((x, y, yaw))
            if len(self.position_buffer) == self.position_buffer.maxlen:
                avg_x = sum(p[0] for p in self.position_buffer) / len(self.position_buffer)
                avg_y = sum(p[1] for p in self.position_buffer) / len(self.position_buffer)
                avg_yaw = sum(p[2] for p in self.position_buffer) / len(self.position_buffer)
                self.current_x = avg_x
                self.current_y = avg_y
                self.current_yaw = avg_yaw
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - self.last_log_time >= 1.0:
                    self.get_logger().debug(f"Smoothed Pose: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={self.current_yaw:.2f}")
                    self.last_log_time = current_time

        def update_pose(self):
            try:
                trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                q = trans.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
                self.position_buffer.append((x, y, yaw))
                if len(self.position_buffer) == self.position_buffer.maxlen:
                    avg_x = sum(p[0] for p in self.position_buffer) / len(self.position_buffer)
                    avg_y = sum(p[1] for p in self.position_buffer) / len(self.position_buffer)
                    avg_yaw = sum(p[2] for p in self.position_buffer) / len(self.position_buffer)
                    self.current_x = avg_x
                    self.current_y = avg_y
                    self.current_yaw = avg_yaw
                    current_time = self.get_clock().now().nanoseconds / 1e9
                    if self.initial_pose is None:
                        self.initial_pose = (self.current_x, self.current_y, self.current_yaw)
                        self.get_logger().info(f"Initial Pose: x={self.initial_pose[0]:.2f}, y={self.initial_pose[1]:.2f}, yaw={self.initial_pose[2]:.2f}")
                    elif current_time - self.last_log_time >= 1.0:
                        self.get_logger().debug(f"TF Smoothed Pose: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={self.current_yaw:.2f}")
                        self.last_log_time = current_time
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - self.last_log_time >= 1.0:
                    self.get_logger().warn(f"TF lookup failed: {e}")
                    self.last_log_time = current_time

        def shortest_angular_distance(self, from_angle, to_angle):
            diff = to_angle - from_angle
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            return diff

        def move_forward(self, distance):
            start_x, start_y = self.current_x, self.current_y
            cmd = Twist()
            cmd.linear.x = 0.3
            start_time = self.get_clock().now()
            timeout = 30.0
            last_log_distance = -0.1
            while True:
                current_distance = math.sqrt((self.current_x - start_x) ** 2 + (self.current_y - start_y) ** 2)
                if current_distance >= distance or (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                    break
                current_time = self.get_clock().now().nanoseconds / 1e9
                if abs(current_distance - last_log_distance) >= 0.1 and current_time - self.last_log_time >= 1.0:
                    self.get_logger().info(f"Current distance: {current_distance:.3f}/{distance}m")
                    self.last_log_time = current_time
                    last_log_distance = current_distance
                self.pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
            cmd.linear.x = 0.0
            self.pub.publish(cmd)
            self.get_logger().info(f"Finished move_forward ({distance}m)")
            time.sleep(3.0)
            self.get_logger().info("Waiting 3 seconds at corner")

        def rotate(self, angle):
            start_yaw = self.current_yaw
            cmd = Twist()
            start_time = self.get_clock().now()
            timeout = 60.0
            target_yaw = start_yaw + angle
            kp = 1.0  # Proportional gain (adjust for faster/slower response)
            max_angular_vel = 0.7  # Maximum angular velocity in rad/s
            min_angular_vel = 0.6  # Minimum angular velocity in rad/s
            last_log_error = float('inf')
            while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
                error = self.shortest_angular_distance(self.current_yaw, target_yaw)
                turned_angle = self.shortest_angular_distance(start_yaw, self.current_yaw)
                if abs(error) < 0.1:
                    break
                # Apply proportional control with minimum and maximum angular velocity
                proportional_vel = kp * error
                if abs(proportional_vel) < min_angular_vel:
                    cmd.angular.z = min_angular_vel * (1 if error > 0 else -1)
                else:
                    cmd.angular.z = max(min(proportional_vel, max_angular_vel), -max_angular_vel)
                current_time = self.get_clock().now().nanoseconds / 1e9
                if abs(error - last_log_error) >= 0.01 and current_time - self.last_log_time >= 1.0:
                    self.get_logger().info(f"Turned: {turned_angle:.3f}/{angle}rad, Error: {error:.3f}, Angular Vel: {cmd.angular.z:.3f}")
                    self.last_log_time = current_time
                    last_log_error = error
                self.pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            self.get_logger().info(f"Finished rotate ({angle}rad), Final angle: {turned_angle:.3f}")
            time.sleep(3.0)
            self.get_logger().info("Waiting 3 seconds at corner")

        def calculate_position_error(self):
            if self.initial_pose is None:
                self.get_logger().warn("Initial pose not set, cannot calculate error")
                return None
            final_x, final_y, final_yaw = self.current_x, self.current_y, self.current_yaw
            init_x, init_y, init_yaw = self.initial_pose
            x_error = final_x - init_x
            y_error = final_y - init_y
            position_error = math.sqrt(x_error**2 + y_error**2)
            angular_error = abs(self.shortest_angular_distance(final_yaw, init_yaw))
            self.get_logger().info(
                f"Position Error: {position_error:.3f}m (x_error: {x_error:.3f}m, y_error: {y_error:.3f}m), "
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

            for i, (action, value) in enumerate(moves):
                self.get_logger().info(f"Starting {action} ({value}), Step {i+1}/{len(moves)}")
                try:
                    if action == "forward":
                        self.move_forward(value)
                    elif action == "turn":
                        self.rotate(value)
                    self.get_logger().info(f"Completed {action} ({value}), Step {i+1}/{len(moves)}")
                except Exception as e:
                    self.get_logger().error(f"Error during {action} ({value}): {e}")
                    break

            self.get_logger().info("Trajectory execution completed")
            self.calculate_position_error()

    def main():
        rclpy.init()
        node = MoveSequence()
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