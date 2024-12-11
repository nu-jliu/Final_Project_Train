import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class JointStatePublisher(Node):

    def __init__(self):
        super().__init__("command_publisher")

        self.pub_command = self.create_publisher(JointState, "arm/js_command", 10)

        msg = JointState()

        msg.position = [-math.pi / 4.0, 0.1, 0.1, -0.2]

        self.pub_command.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    # rclpy.spin_once(node=node)

    node.destroy_node()
    rclpy.shutdown()
