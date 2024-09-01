#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class DualOdomToTF(Node):
    def __init__(self):
        super().__init__('dual_odom_to_tf_broadcaster_node')

        self.robot_names = ["rick", "morty"]
        self.transforms = {}
        self.broadcasters = {}

        # Using reliable QoS for critical data
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        # Create a static transform broadcaster for the static transforms
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        for robot_name in self.robot_names:

            # Single broadcaster for dynamic transforms
            self.broadcasters[robot_name] = tf2_ros.TransformBroadcaster(self)

            # Static transform from world to odom
            transform_world_to_odom = TransformStamped()
            transform_world_to_odom.header.stamp = self.get_clock().now().to_msg()
            transform_world_to_odom.header.frame_id = "world"
            transform_world_to_odom.child_frame_id = f"{robot_name}/odom"
            transform_world_to_odom.transform.translation.x = 0.0
            transform_world_to_odom.transform.translation.y = 0.0
            transform_world_to_odom.transform.translation.z = 0.0
            transform_world_to_odom.transform.rotation.x = 0.0
            transform_world_to_odom.transform.rotation.y = 0.0
            transform_world_to_odom.transform.rotation.z = 0.0
            transform_world_to_odom.transform.rotation.w = 1.0

            # Broadcast the static transform
            self.static_broadcaster.sendTransform(transform_world_to_odom)

            # Dynamic transform from odom to base_link
            transform_base = TransformStamped()
            transform_base.header.frame_id = f"{robot_name}/odom"
            transform_base.child_frame_id = f"{robot_name}/base_link"
            self.transforms[f"{robot_name}/base_link"] = transform_base

            # Subscriber for each robot's odometry
            self.create_subscription(
                Odometry,
                f"{robot_name}/odom",
                lambda msg, rn=robot_name: self.odom_callback(msg, rn),
                qos_reliable
            )

        # Timer set to call broadcast_transforms at a 0.1-second interval
        self.timer = self.create_timer(0.05, self.broadcast_transforms)

        self.get_logger().info("Dual robot odom to TF broadcaster node is ready!")

    def broadcast_transforms(self):
        for robot_name in self.robot_names:
            broadcaster = self.broadcasters[robot_name]
            broadcaster.sendTransform(self.transforms[f"{robot_name}/base_link"])


    def odom_callback(self, odom_msg, robot_name):
        self.update_transforms(odom_msg, robot_name)

    def update_transforms(self, odom_msg, robot_name):

        # Update base_link transform
        base_transform = self.transforms[f"{robot_name}/base_link"]
        base_transform.header.stamp = odom_msg.header.stamp
        base_transform.transform.translation.x = odom_msg.pose.pose.position.x
        base_transform.transform.translation.y = odom_msg.pose.pose.position.y
        base_transform.transform.translation.z = odom_msg.pose.pose.position.z
        base_transform.transform.rotation = odom_msg.pose.pose.orientation

        # Log transform updates for debugging
        self.get_logger().debug(f'Updated transform for {robot_name} at time {odom_msg.header.stamp.sec}.{odom_msg.header.stamp.nanosec}')


def main(args=None):
    rclpy.init(args=args)
    node = DualOdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
