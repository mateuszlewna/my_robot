import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math


class MoveSequence(Node):
    def __init__(self):
        super().__init__('move_sequence')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.subscription = self.create_subscription(
            PoseStamped, '/pose', self.pose_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        # Konwersja kwaternionu na yaw (dla płaskiego ruchu)
        q = msg.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z),
            1.0 - 2.0 * (q.z * q.z)
        )

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
        cmd.linear.x = 0.4  # Stała prędkość liniowa
        while math.sqrt((self.current_x - start_x) ** 2 +
                        (self.current_y - start_y) ** 2) < distance:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
        cmd.linear.x = 0.0
        self.pub.publish(cmd)

    def rotate(self, angle):
        start_yaw = self.current_yaw
        cmd = Twist()
        cmd.angular.z = 0.7 if angle > 0 else -0.7  # Stała prędkość kątowa

        turned_angle = 0.0
        while abs(turned_angle) < abs(angle):
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
            turned_angle = self.shortest_angular_distance(start_yaw, self.current_yaw)

        cmd.angular.z = 0.0
        self.pub.publish(cmd)

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
            if action == "forward":
                self.move_forward(value)
            elif action == "turn":
                self.rotate(value)


def main():
    rclpy.init()
    node = MoveSequence()
    node.execute_sequence()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
