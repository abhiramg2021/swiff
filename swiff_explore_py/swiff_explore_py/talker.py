#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__("talker")
        self.publisher_ = self.create_publisher(String, "chatter", 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Hello, world: %d" % self.get_clock().now().to_msg().sec
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
