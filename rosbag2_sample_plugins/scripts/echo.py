#!/usr/bin/env python3

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub_chat = self.create_subscription(String, '/chat', self.chat_cb, 1)
        self.sub_geom = self.create_subscription(Point, '/geom', self.geom_cb, 1)

    def chat_cb(self, msg):
        print(f'Got chat {msg.data}')

    def geom_cb(self, msg):
        print(f'Got geom {msg.x},{msg.y},{msg.z}')


def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
