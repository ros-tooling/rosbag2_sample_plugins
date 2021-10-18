#!/usr/bin/env python3

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, '/chat', 1)
        self.pub2 = self.create_publisher(Point, '/geom', 1)
        self.timer = self.create_timer(1.0, self.pub_cb)
        self.count = 0

    def pub_cb(self):
        msg = String(data=f'hello {self.count}')
        print(f'published {msg.data}')
        self.pub.publish(msg)

        msg = Point(x=self.count * 0.1, y=self.count * 1.0, z=self.count * 5.0)
        self.pub2.publish(msg)

        self.count += 1


def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
