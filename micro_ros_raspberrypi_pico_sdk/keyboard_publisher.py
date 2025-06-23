#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import keyboard  # much better for real-time input

speed = 5
timerhertz = 100  # 20 Hz for smooth input
timerperiod = 1.0 / timerhertz  # period in seconds

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher = self.create_publisher(Int32, 'rp2040_listener_topic', 10)
        self.angle = 90

        self.timer = self.create_timer(timerperiod, self.check_keys)  # 20 Hz = smooth input
        self.get_logger().info("Use LEFT and RIGHT arrows to control. ESC to quit.")

    def check_keys(self):
        if keyboard.is_pressed('left'):
            self.angle = max(0, self.angle - speed)
            self.send_angle()
        elif keyboard.is_pressed('right'):
            self.angle = min(180, self.angle + speed)
            self.send_angle()
        elif keyboard.is_pressed('esc'):
            self.get_logger().info("Exiting...")
            rclpy.shutdown()

    def send_angle(self):
        msg = Int32()
        msg.data = self.angle
        self.publisher.publish(msg)
        self.get_logger().info(f"Published angle: {self.angle}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
