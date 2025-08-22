import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import tf2_ros
import math
import csv
from datetime import datetime

class PoseReader(Node):
    def __init__(self):
        super().__init__('pose_reader')
        self.odom_pose = None
        self.filtered_odom_pose = None
        self.amcl_pose = None
        self.tf_pose = None
        self.step_count = 0

        # Subskrypcje
        self.odom_sub = None
        self.filtered_odom_sub = None
        self.amcl_sub = None
        self.tf_sub = None
        self.create_subscriptions()

        # Bufor TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Plik CSV
        self.csv_filename = f'robot_poses_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Step', 'Temat', 'x', 'y', 'yaw'])

    def create_subscriptions(self):
        # Tworzenie nowych subskrypcji dla każdego kroku
        self.odom_pose = None
        self.filtered_odom_pose = None
        self.amcl_pose = None
        self.tf_pose = None
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.filtered_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_odom_callback, 10)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (x, y, yaw)
        self.odom_sub.destroy()  # Zatrzymaj subskrypcję po otrzymaniu danych

    def filtered_odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.filtered_odom_pose = (x, y, yaw)
        self.filtered_odom_sub.destroy()

    def amcl_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.amcl_pose = (x, y, yaw)
        self.amcl_sub.destroy()

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'base_link':
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                q = transform.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.tf_pose = (x, y, yaw)
                self.tf_sub.destroy()
                break

    def read_poses(self):
        self.step_count += 1
        self.get_logger().info(f"--- Step {self.step_count} ---")
        self.create_subscriptions()  # Reset subskrypcji dla nowego odczytu

        start_time = self.get_clock().now()
        timeout = 5.0  # Maksymalny czas oczekiwania na dane (sekundy)

        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            # Sprawdź, czy wszystkie dane zostały zebrane
            if (self.odom_pose is not None and 
                self.filtered_odom_pose is not None and 
                self.amcl_pose is not None and 
                self.tf_pose is not None):
                break

        # Zapisz dane do pliku CSV
        with open(self.csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([self.step_count, '/odom', 
                            self.odom_pose[0] if self.odom_pose else 'N/A', 
                            self.odom_pose[1] if self.odom_pose else 'N/A', 
                            self.odom_pose[2] if self.odom_pose else 'N/A'])
            writer.writerow([self.step_count, '/odometry/filtered', 
                            self.filtered_odom_pose[0] if self.filtered_odom_pose else 'N/A', 
                            self.filtered_odom_pose[1] if self.filtered_odom_pose else 'N/A', 
                            self.filtered_odom_pose[2] if self.filtered_odom_pose else 'N/A'])
            writer.writerow([self.step_count, '/amcl_pose', 
                            self.amcl_pose[0] if self.amcl_pose else 'N/A', 
                            self.amcl_pose[1] if self.amcl_pose else 'N/A', 
                            self.amcl_pose[2] if self.amcl_pose else 'N/A'])
            writer.writerow([self.step_count, 'TF (map -> base_link)', 
                            self.tf_pose[0] if self.tf_pose else 'N/A', 
                            self.tf_pose[1] if self.tf_pose else 'N/A', 
                            self.tf_pose[2] if self.tf_pose else 'N/A'])

        # Wyświetl wyniki w konsoli
        self.get_logger().info(f"Pozycje dla kroku {self.step_count}:")
        if self.odom_pose:
            self.get_logger().info(f"/odom: x={self.odom_pose[0]:.2f}, y={self.odom_pose[1]:.2f}, yaw={self.odom_pose[2]:.2f}")
        else:
            self.get_logger().info("/odom: Brak danych")
        if self.filtered_odom_pose:
            self.get_logger().info(f"/odometry/filtered: x={self.filtered_odom_pose[0]:.2f}, y={self.filtered_odom_pose[1]:.2f}, yaw={self.filtered_odom_pose[2]:.2f}")
        else:
            self.get_logger().info("/odometry/filtered: Brak danych")
        if self.amcl_pose:
            self.get_logger().info(f"/amcl_pose: x={self.amcl_pose[0]:.2f}, y={self.amcl_pose[1]:.2f}, yaw={self.amcl_pose[2]:.2f}")
        else:
            self.get_logger().info("/amcl_pose: Brak danych")
        if self.tf_pose:
            self.get_logger().info(f"TF (map -> base_link): x={self.tf_pose[0]:.2f}, y={self.tf_pose[1]:.2f}, yaw={self.tf_pose[2]:.2f}")
        else:
            self.get_logger().info("TF (map -> base_link): Brak danych")

def main():
    rclpy.init()
    node = PoseReader()
    try:
        while True:
            input("Naciśnij Enter, aby zebrać pozycje (lub 'q' aby zakończyć)...")
            user_input = input().strip().lower()
            if user_input == 'q':
                break
            node.read_poses()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gracefully")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()