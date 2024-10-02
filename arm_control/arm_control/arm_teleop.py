import math

import rclpy
from rclpy.node import Node


from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState


class ArmTeleop(Node):

    def __init__(self):
        super().__init__("arm_teleop")

        self.sub_joy = self.create_subscription(Joy, "joy", self.sub_joy_callback, 10)
        self.pub_js_command = self.create_publisher(JointState, "arm/js_command", 10)

    def sub_joy_callback(self, msg: Joy):
        j0 = msg.axes[5] * (-math.pi / 4.0) + math.pi / 4.0
        j1 = msg.axes[3] * math.pi / 2.0
        j2 = msg.axes[4] * (-math.pi / 2.0) if msg.axes[4] < 0 else 0.0

        msg_js = JointState()
        msg_js.position = [j0, j1, j2, 0]
        self.pub_js_command.publish(msg_js)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()

    try:
        rclpy.spin(node)
    except:
        node.destroy_node()
        rclpy.try_shutdown()
