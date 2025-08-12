import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
import math
import tf_transformations
import time


def create_pose(x, y, yaw_deg, frame_id="map"):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    yaw_rad = math.radians(yaw_deg)
    q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


class NavThroughPosesClient(Node):
    def __init__(self):
        super().__init__('nav_through_poses_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._initpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self._amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        self._current_pose = None

    def amcl_pose_callback(self, msg):
        self._current_pose = msg

    def set_initial_pose_from_amcl(self):
        while rclpy.ok() and self._current_pose is None:
            self.get_logger().info("Czekam na /amcl_pose...")
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._current_pose:
            self.get_logger().info("Ustawiam initial pose na obecną pozycję robota")
            self._initpose_pub.publish(self._current_pose)
            time.sleep(1)

    def send_goal(self, poses_list):
        self._action_client.wait_for_server()
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses_list
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NavThroughPosesClient()

    # Czekamy na aktualną pozycję po zlokalizowaniu
    node.set_initial_pose_from_amcl()

    # Pozycja startowa z AMCL
    start_pose = node._current_pose.pose.pose
    start_x = start_pose.position.x
    start_y = start_pose.position.y
    q = start_pose.orientation
    start_yaw = math.degrees(math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z)))

    # Definicja ruchów względem pozycji startowej
    moves = [
        ("forward", 1.0),
        ("turn", 90),
        ("forward", 0.5),
        ("turn", 90),
        ("forward", 1.0),
        ("turn", 90),
        ("forward", 0.5),
        ("turn", 90)
    ]

    # Generujemy listę pozycji globalnych
    poses = []
    x, y, yaw_deg = start_x, start_y, start_yaw
    for action, value in moves:
        if action == "forward":
            # Przemieszczenie w aktualnym kierunku
            yaw_rad = math.radians(yaw_deg)
            x += value * math.cos(yaw_rad)
            y += value * math.sin(yaw_rad)
        elif action == "turn":
            # Zmiana orientacji
            yaw_deg = (yaw_deg + value) % 360

        poses.append(create_pose(x, y, yaw_deg))

    # Wysyłamy całą trasę
    node.send_goal(poses)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
