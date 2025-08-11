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

    def set_initial_pose(self, x=0.0, y=0.0, yaw_deg=0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        yaw_rad = math.radians(yaw_deg)
        q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        # covariance - przykładowa macierz zerowa (możesz zmienić, jeśli chcesz)
        msg.pose.covariance = [0.0]*36
        self.get_logger().info(f'Setting initial pose to x:{x}, y:{y}, yaw:{yaw_deg} deg')
        self._initpose_pub.publish(msg)
        # Daj chwilę na opublikowanie i odebranie
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

    # Ustawiamy pozycję startową na 0,0,0 stopni
    node.set_initial_pose(0.0, 0.0, 0.0)

    # Definiujemy punkty trasy
    poses = [
        create_pose(1.0, 0.0, 0),
        create_pose(1.0, 0.5, 90),
        create_pose(0.5, 0.5, 180),
        create_pose(0.0, 0.0, 270),
    ]

    node.send_goal(poses)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
