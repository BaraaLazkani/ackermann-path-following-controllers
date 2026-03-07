#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdometryToTF(Node):
    def __init__(self):
        super().__init__('odometry_to_tf')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry,
            '/model/ackermann_steering_vehicle/odometry',
            self.odom_callback,
            10)

        self.get_logger().info('Odometry to TF broadcaster started')
        self.get_logger().info('Subscribing to: /model/ackermann_steering_vehicle/odometry')
        self.get_logger().info('Publishing TF: odom -> body_link')
        self.first_msg = True

    def odom_callback(self, msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'body_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

        if self.first_msg:
            self.get_logger().info(f'Publishing TF! Position: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})')
            self.first_msg = False

def main(args=None):
    rclpy.init(args=args)
    node = OdometryToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
