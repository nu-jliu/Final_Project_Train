import math

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

from std_srvs.srv import Trigger


class ArmTeleop(Node):

    def __init__(self):
        super().__init__("arm_teleop")
        self.callback_group = ReentrantCallbackGroup()

        self.sub_joy = self.create_subscription(
            Joy,
            "joy",
            self.sub_joy_callback,
            10,
            callback_group=self.callback_group,
        )
        self.pub_js_command = self.create_publisher(
            JointState,
            "arm/js_command",
            10,
            callback_group=self.callback_group,
        )

        self.left_down = False
        self.right_down = False

        self.cli_left_leg_down = self.create_client(
            Trigger,
            "leg/left/down",
            callback_group=self.callback_group,
        )
        self.cli_left_leg_up = self.create_client(
            Trigger,
            "leg/left/up",
            callback_group=self.callback_group,
        )
        self.cli_right_leg_down = self.create_client(
            Trigger,
            "leg/right/down",
            callback_group=self.callback_group,
        )
        self.cli_right_leg_up = self.create_client(
            Trigger,
            "leg/right/up",
            callback_group=self.callback_group,
        )

        self.cli_pump_on = self.create_client(
            Trigger,
            "pump/on",
            callback_group=self.callback_group,
        )

        self.cli_pump_off = self.create_client(
            Trigger,
            "pump/off",
            callback_group=self.callback_group,
        )

        self.cli_valve_on = self.create_client(
            Trigger,
            "valve/on",
            callback_group=self.callback_group,
        )

        self.cli_valve_off = self.create_client(
            Trigger,
            "valve/off",
            callback_group=self.callback_group,
        )

    async def sub_joy_callback(self, msg: Joy):
        j0 = msg.axes[5] * (-math.pi / 2.0) + math.pi / 2.0
        j1 = msg.axes[3] * math.pi / 2.0
        j2 = msg.axes[4] * (-math.pi / 2.0) if msg.axes[4] < 0 else 0.0

        # self.get_logger().info(f"{len(msg.buttons)}")
        left_down = bool(msg.buttons[4])
        right_down = bool(msg.buttons[5])

        print(f"{left_down, right_down}")

        future: Future = None

        if left_down and not self.left_down:
            future = self.cli_pump_on.call_async(Trigger.Request())

        elif not left_down and self.left_down:
            future = self.cli_pump_off.call_async(Trigger.Request())

        if right_down and not self.right_down:
            future = self.cli_valve_on.call_async(Trigger.Request())

        elif not right_down and self.right_down:
            future = self.cli_valve_off.call_async(Trigger.Request())

        if future is not None:
            await future

        self.left_down = left_down
        self.right_down = right_down

        msg_js = JointState()
        msg_js.position = [j0, j1, j2, 0]
        self.pub_js_command.publish(msg_js)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.try_shutdown()
