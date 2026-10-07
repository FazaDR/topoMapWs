import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node


class ManualWaypointFollower(Node):
    def __init__(self):
        super().__init__("manual_waypoint_follower")
        self.declare_parameter(
            "waypoints",
            [
                16.0, 0.0,
                16.0, -4.0,
                0.0, -4.0,
                0.0, -8.0,
                16.0, -8.0,
                16.0, -12.0,
                0.0, -12.0,
                0.0, -16.0,
                16.0, -16.0,
                16.0, -21.0,
                -4.0, -21.0,
                -4.0, 0.0,
                0.0, 0.0,
            ],
        )
        self.declare_parameter(
            "yaws",
            [
                0.0,
                -math.pi / 2.0,
                math.pi,
                -math.pi / 2.0,
                0.0,
                -math.pi / 2.0,
                math.pi,
                -math.pi / 2.0,
                0.0,
                -math.pi / 2.0,
                math.pi,
                math.pi / 2.0,
                0.0,
            ],
        )
        self.client = ActionClient(
            self, NavigateThroughPoses, "/navigate_through_poses"
        )

    def send_waypoints(self):
        values = self.get_parameter("waypoints").value
        yaws = self.get_parameter("yaws").value
        if len(values) < 2 or len(values) % 2 != 0:
            raise ValueError("waypoints must be [x0, y0, x1, y1, ...]")
        if len(yaws) != len(values) // 2:
            raise ValueError("yaws must contain one yaw per waypoint")

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "NavigateThroughPoses action server is not available. "
                "Start Nav2 first with: ros2 launch bringup nav2.launch.py"
            )
            return False

        goal = NavigateThroughPoses.Goal()
        for index in range(0, len(values), 2):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = values[index]
            pose.pose.position.y = values[index + 1]
            pose.pose.orientation.z = math.sin(yaws[index // 2] / 2.0)
            pose.pose.orientation.w = math.cos(yaws[index // 2] / 2.0)
            goal.poses.append(pose)

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Waypoint goal was rejected")
            rclpy.shutdown()
            return
        self.get_logger().info("Waypoint goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        self.get_logger().info(
            f"Waypoint route finished with status {status}"
        )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ManualWaypointFollower()
    if node.send_waypoints():
        rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
