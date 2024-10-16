from adafruit_servokit import ServoKit
import math
import copy

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState

from std_srvs.srv import Trigger, Trigger_Request, Trigger_Response


class ServoControl(Node):

    def __init__(self):
        super().__init__("servo_control")
        self.callback_group = ReentrantCallbackGroup()
        self.get_logger().info("Initializing Servo")

        self.servo_kit = ServoKit(channels=16)

        self.get_logger().info("Servo initialized")

        self.home_pos = [180.0, 100.0, 35.0, 0.0]
        self.leg_home_pos = [100]

        self.current_pos: list[float] = [None] * 4
        self.desired_pos: list[float] = copy.deepcopy(self.home_pos)

        self.timer = self.create_timer(
            0.01,
            self.timer_callback,
            callback_group=self.callback_group,
        )

        self.srv_leg_up = self.create_service(
            Trigger,
            "leg/up",
            self.srv_leg_up_callback,
            callback_group=self.callback_group,
        )

        self.srv_leg_down = self.create_service(
            Trigger,
            "leg/down",
            self.srv_leg_down_callback,
            callback_group=self.callback_group,
        )

        self.sub_js_command = self.create_subscription(
            JointState,
            "arm/js_command",
            self.sub_js_command_callback,
            10,
            callback_group=self.callback_group,
        )

        self.leg_up()

    def __del__(self):
        for i in range(16):
            self.servo_kit.servo[i].set_pulse_width_range(0, 0)
            self.get_logger().info(f"Detaching servo {i}")

        self.get_logger().info(f"Shutting down node")
        rclpy.try_shutdown()

    def timer_callback(self):
        # self.get_logger().info("Timer")

        for i, (curr, desired) in enumerate(zip(self.current_pos, self.desired_pos)):
            desired = min(desired, 180)
            desired = max(desired, 0)

            if curr is None:
                target = desired
            elif curr < desired:
                target = min(curr + 0.5, desired)
            elif curr > desired:
                target = max(curr - 0.5, desired)
            else:
                continue

            self.servo_kit.servo[i].angle = target
            self.current_pos[i] = target

            self.get_logger().info(f"Set servo {i} to position {target}")

    def sub_js_command_callback(self, msg: JointState):
        # self.get_logger().info("joint command")
        if len(msg.position) < 4:
            self.get_logger().error("Position command in valid")

        # self.desired_pos[0] = self.home_pos[0] + msg.position[0] * 180 / math.pi
        # self.desired_pos[1] = self.home_pos[1] + msg.position[1] * 180 / math.pi
        # self.desired_pos[2] = self.home_pos[2] + msg.position[2] * 180 / math.pi
        # self.desired_pos[3] = self.home_pos[3] + msg.position[3] * 180 / math.pi

        for i in range(4):
            self.desired_pos[i] = self.home_pos[i] + msg.position[i] * 160 / math.pi
            # self.get_logger().info(
            #     f"Received joint {i} with position {msg.position[i]}"
            # )

    def srv_leg_down_callback(
        self,
        request: Trigger_Request,
        response: Trigger_Response,
    ):
        self.leg_down()

        response.success = True
        return response

    def srv_leg_up_callback(
        self,
        request: Trigger_Request,
        response: Trigger_Response,
    ):
        self.leg_up()

        response.success = True
        return response

    def leg_down(self):
        self.get_logger().info("Putting leg down")
        self.servo_kit.servo[4].angle = 100
        self.servo_kit.servo[5].angle = 105

    def leg_up(self):
        self.get_logger().info("Putting leg up")
        self.servo_kit.servo[4].angle = 160
        self.servo_kit.servo[5].angle = 40


def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()

    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.try_shutdown()
