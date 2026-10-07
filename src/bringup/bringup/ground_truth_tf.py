import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GroundTruthTf(Node):
    def __init__(self):
        super().__init__("ground_truth_tf")
        self.broadcaster = TransformBroadcaster(self)
        self.initialized = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0
        self.subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

    def odom_callback(self, msg):
        stamp = msg.header.stamp
        orientation = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(orientation.z, orientation.w)

        if not self.initialized:
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.y
            self.initial_yaw = yaw
            self.initialized = True
            self.get_logger().info(
                f"Map frame initialized at odom pose "
                f"({self.initial_x:.2f}, {self.initial_y:.2f})"
            )

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.translation.x = (
            -math.cos(self.initial_yaw) * self.initial_x
            - math.sin(self.initial_yaw) * self.initial_y
        )
        map_to_odom.transform.translation.y = (
            math.sin(self.initial_yaw) * self.initial_x
            - math.cos(self.initial_yaw) * self.initial_y
        )
        map_to_odom.transform.rotation.z = -math.sin(self.initial_yaw / 2.0)
        map_to_odom.transform.rotation.w = math.cos(self.initial_yaw / 2.0)

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_link"
        odom_to_base.transform.translation.x = msg.pose.pose.position.x
        odom_to_base.transform.translation.y = msg.pose.pose.position.y
        odom_to_base.transform.translation.z = msg.pose.pose.position.z
        odom_to_base.transform.rotation = orientation

        self.broadcaster.sendTransform([map_to_odom, odom_to_base])


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
