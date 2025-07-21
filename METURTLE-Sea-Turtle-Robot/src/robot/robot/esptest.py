#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ESP32Subscriber(Node):
    def __init__(self):
        super().__init__('esp32_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'distance',  # Topic name from your ESP32 code
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f"Received from ESP32: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()