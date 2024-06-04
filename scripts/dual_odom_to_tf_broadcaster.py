#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class DualOdomToTF(Node):
    def __init__(self):
        super().__init__('dual_odom_to_tf_broadcaster_node')

        self.robot_names = ["rick", "morty"]
        self.transforms = {}
        self.broadcasters = {}

        # Using reliable QoS for critical data
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        for robot_name in self.robot_names:
            # Transform from world to odom
            transform_odom = TransformStamped()
            transform_odom.header.frame_id = "world"
            transform_odom.child_frame_id = f"{robot_name}/odom"
            self.transforms[f"{robot_name}/odom"] = transform_odom

            # Transform from odom to base_link
            transform_base = TransformStamped()
            transform_base.header.frame_id = f"{robot_name}/odom"
            transform_base.child_frame_id = f"{robot_name}/base_link"
            self.transforms[f"{robot_name}/base_link"] = transform_base

            # Single broadcaster for all transforms
            self.broadcasters[robot_name] = tf2_ros.TransformBroadcaster(self)

            # Subscriber for each robot's odometry
            self.create_subscription(
                Odometry,
                f"{robot_name}/odom",
                lambda msg, rn=robot_name: self.odom_callback(msg, rn),
                qos_reliable
            )

            # Subscribe to the laser scan topic
            self.create_subscription(
                LaserScan,
                f"{robot_name}/scan",
                lambda msg, rn=robot_name: self.scan_callback(msg, rn),
                qos_reliable
            )

        # Timer set to call broadcast_transforms at a 0.1-second interval
        self.timer = self.create_timer(0.1, self.broadcast_transforms)

        self.get_logger().info("Dual robot odom to TF broadcaster node is ready!")

    def broadcast_transforms(self):
        for robot_name in self.robot_names:
            broadcaster = self.broadcasters[robot_name]
            broadcaster.sendTransform(self.transforms[f"{robot_name}/odom"])
            broadcaster.sendTransform(self.transforms[f"{robot_name}/base_link"])

    def scan_callback(self, scan_msg, robot_name):
        # Log or process the incoming laser scan data
        self.get_logger().info(f'Received scan from {robot_name}: {len(scan_msg.ranges)} ranges')

    def odom_callback(self, odom_msg, robot_name):
        self.update_transforms(odom_msg, robot_name)

    def update_transforms(self, odom_msg, robot_name):
        # Update odom transform
        odom_transform = self.transforms[f"{robot_name}/odom"]
        odom_transform.header.stamp = odom_msg.header.stamp
        odom_transform.transform.translation.x = 0.0
        odom_transform.transform.translation.y = 0.0
        odom_transform.transform.translation.z = 0.0
        odom_transform.transform.rotation = odom_msg.pose.pose.orientation

        # Update base_link transform
        base_transform = self.transforms[f"{robot_name}/base_link"]
        base_transform.header.stamp = odom_msg.header.stamp
        base_transform.transform.translation.x = odom_msg.pose.pose.position.x
        base_transform.transform.translation.y = odom_msg.pose.pose.position.y
        base_transform.transform.translation.z = odom_msg.pose.pose.position.z
        base_transform.transform.rotation = odom_msg.pose.pose.orientation

 #   def update_transforms(self, odom_msg, robot_name):
  #      # Update odom transform
   #     odom_transform = self.transforms[f"{robot_name}/odom"]
    #    odom_transform.header.stamp = odom_msg.header.stamp
    #    odom_transform.transform.translation.x = odom_msg.pose.pose.position.x
    #    odom_transform.transform.translation.y = odom_msg.pose.pose.position.y
    #    odom_transform.transform.translation.z = odom_msg.pose.pose.position.z
    #    odom_transform.transform.rotation = odom_msg.pose.pose.orientation

        # Update base_link transform
    # Since odom and base_link are coincident, we reset base_link's relative pose
      #  base_transform = self.transforms[f"{robot_name}/base_link"]
     #   base_transform.header.stamp = odom_msg.header.stamp
       # base_transform.transform.translation.x = 0.0
        #base_transform.transform.translation.y = 0.0
        #base_transform.transform.translation.z = 0.0
        #base_transform.transform.rotation = odom_msg.pose.pose.orientation 

def main(args=None):
    rclpy.init(args=args)
    node = DualOdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
