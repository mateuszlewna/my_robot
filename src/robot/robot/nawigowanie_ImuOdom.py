#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MoveSequenceOdometry(Node):
    def __init__(self):
        super().__init__('move_sequence_odometry')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # Konwersja kwaternionu na yaw
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def move_forward(self, distance, speed=0.5):
        start_x, start_y = self.current_x, self.current_y
        cmd = Twist()
        cmd.linear.x = speed
        while math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2) < distance:
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)

    def rotate(self, angle, angular_speed=0.8):
        start_yaw = self.current_yaw
        cmd = Twist()
        cmd.angular.z = angular_speed if angle > 0 else -angular_speed
        while abs(self.current_yaw - start_yaw) < abs(angle):
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def execute_sequence(self):
        moves = [
            (3.0, 0),   # 3m prosto
          
        ]
        for distance, angle in moves:
            self.move_forward(distance)
            self.rotate(angle)

def main(args=None):
    rclpy.init(args=args)
    node = MoveSequenceOdometry()
    try:
        node.execute_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()