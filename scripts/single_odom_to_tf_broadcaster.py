#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster_node')
        robot_name_1 = "rick"
        robot_name_2 = "morty"

        # Robot 1 Transform
        self.transform_stamped_robot1 = TransformStamped()
        self.transform_stamped_robot1.header.frame_id = "world"
        self.transform_stamped_robot1.child_frame_id = robot_name_1 + "/odom"

        # Subscriber for Robot 1's odometry
        self.subscriber_robot1 = self.create_subscription(
            Odometry,
            robot_name_1 + '/odom',
            self.odom_callback_robot1,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Transform broadcaster for Robot 1
        self.br_robot1 = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Robot odom to TF broadcaster node is ready for Robot 1!")

    def odom_callback_robot1(self, msg):
        self.broadcast_new_tf(msg, self.transform_stamped_robot1, self.br_robot1)

    def broadcast_new_tf(self, odom_msg, transform_stamped, broadcaster):
        """
        This function broadcasts a new TF message to the TF network based on received odometry data for Robot 1.
        """

        # Set the timestamp of the TF message from the odometry message.
        transform_stamped.header.stamp = odom_msg.header.stamp

        # Set the translation of the TF message from the odometry position.
        transform_stamped.transform.translation.x = odom_msg.pose.pose.position.x
        transform_stamped.transform.translation.y = odom_msg.pose.pose.position.y
        transform_stamped.transform.translation.z = odom_msg.pose.pose.position.z

        # Set the rotation of the TF message from the odometry orientation.
        transform_stamped.transform.rotation = odom_msg.pose.pose.orientation

        # Broadcast the TF message using the broadcaster passed as a parameter.
        broadcaster.sendTransform(transform_stamped)


def main(args=None):
    rclpy.init(args=args)
    odom_to_tf_node = OdomToTF()
    rclpy.spin(odom_to_tf_node)
    odom_to_tf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
