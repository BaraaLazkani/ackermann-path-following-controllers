#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class CircleMotion(Node):
    def __init__(self):
        super().__init__('circle_motion')

        # Publishers for velocity and steering angle
        self.velocity_pub = self.create_publisher(Float64, '/velocity', 10)
        self.steering_pub = self.create_publisher(Float64, '/steering_angle', 10)

        # Parameters for circle motion
        self.declare_parameter('velocity', 0.5)  # m/s
        self.declare_parameter('steering_angle', 0.3)  # radians (positive = left turn)

        self.velocity = self.get_parameter('velocity').value
        self.steering_angle = self.get_parameter('steering_angle').value

        # Timer to publish commands at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_commands)

        self.get_logger().info(f'Circle motion node started')
        self.get_logger().info(f'Velocity: {self.velocity} m/s, Steering angle: {self.steering_angle} rad')
        self.get_logger().info('Publishing to /velocity and /steering_angle topics')

    def publish_commands(self):
        # Create and publish velocity message
        vel_msg = Float64()
        vel_msg.data = self.velocity
        self.velocity_pub.publish(vel_msg)

        # Create and publish steering angle message
        steer_msg = Float64()
        steer_msg.data = self.steering_angle
        self.steering_pub.publish(steer_msg)


def main(args=None):
    rclpy.init(args=args)
    circle_motion = CircleMotion()

    try:
        rclpy.spin(circle_motion)
    except KeyboardInterrupt:
        pass
    finally:
        circle_motion.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
