import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import TransformListener, Buffer
import math

class MoveSequence(Node):
    def __init__(self):
        super().__init__('move_sequence')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            PoseStamped, '/pose', self.pose_callback, 10)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        # Konwersja kwaternionu na yaw
        q = msg.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))

    def move_forward(self, distance):
        start_x, start_y = self.current_x, self.current_y
        cmd = Twist()
        cmd.linear.x = 0.2  # Stała prędkość liniowa
        while math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2) < distance:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
        cmd.linear.x = 0.0
        self.pub.publish(cmd)

    def rotate(self, angle):
        start_yaw = self.current_yaw
        cmd = Twist()
        cmd.angular.z = 0.3 if angle > 0 else -0.3  # Stała prędkość kątowa
        while abs(self.current_yaw - start_yaw) < abs(angle):
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
        cmd.angular.z = 0.0
        self.pub.publish(cmd)

    def execute_sequence(self):
        moves = [
            (1.0, math.pi/2),   # 1m, skręt 90°
            (0.5, math.pi/2),   # 0.5m, skręt 90°
            (1.0, math.pi/2),   # 1m, skręt 90°
            (0.5, math.pi/2)    # 0.5m, skręt 90°
        ]
        for distance, angle in moves:
            self.move_forward(distance)
            self.rotate(angle)

def main():
    rclpy.init()
    node = MoveSequence()
    node.execute_sequence()
    rclpy.shutdown()

if __name__ == '__main__':
    main()