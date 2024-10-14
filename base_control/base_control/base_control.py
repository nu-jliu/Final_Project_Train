from gpiozero import Motor

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class BaseControl(Node):

    def __init__(self):
        super().__init__("base_control")

        self.front_enable = 18
        self.front_forward = 24
        self.front_backward = 23

        self.rear_enable = 13
        self.rear_forward = 5
        self.rear_backward = 6

        self.motor_front = Motor(
            forward=self.front_forward,
            backward=self.front_backward,
            enable=self.front_enable,
        )

        self.motor_rear = Motor(
            forward=self.rear_forward,
            backward=self.rear_backward,
            enable=self.rear_enable,
        )

        self.get_logger().info("Motor Driver initialized")

        # self.timer = self.create_timer(0.1, self.timer_callback)
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            "cmd_vel",
            self.sub_cmd_vel_callback,
            10,
        )

    # def timer_callback(self):
    #     self.duty_cycle = (self.duty_cycle + 1) % 101

    def sub_cmd_vel_callback(self, msg: Twist):
        speed = msg.linear.x / 1.5
        speed = min(speed, 1.0)
        speed = max(speed, -1.0)

        # if speed > 0:
        #     self.motor.forward(speed=speed / 1.5)

        # elif speed < 0:
        #     self.motor.backward
        #     self.motor.backward()

        # else:
        #     self.motor.stop()
        self.motor_front.value = speed
        self.motor_rear.value = speed

        self.get_logger().info(f"Setting speed to {speed}")


def main(args=None):
    rclpy.init(args=args)
    node = BaseControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.try_shutdown()
