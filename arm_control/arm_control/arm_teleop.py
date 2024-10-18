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

        self.pump_on = False
        self.valve_on = False
        self.leg_down = False

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

        self.cli_leg_up = self.create_client(
            Trigger,
            "leg/up",
            callback_group=self.callback_group,
        )

        self.cli_leg_down = self.create_client(
            Trigger,
            "leg/down",
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

        while not self.cli_leg_up.wait_for_service(timeout_sec=2.0):
            if rclpy.ok():
                self.get_logger().info(
                    f"Service {self.cli_leg_up.srv_name} not available, waiting again"
                )

        while not self.cli_leg_down.wait_for_service(timeout_sec=2.0):
            if rclpy.ok():
                self.get_logger().info(
                    f"Service {self.cli_leg_down.srv_name} not available, waiting again"
                )

        while not self.cli_pump_on.wait_for_service(timeout_sec=2.0):
            if rclpy.ok():
                self.get_logger().info(
                    f"Service {self.cli_pump_on.srv_name} not available, waiting again"
                )

        while not self.cli_pump_off.wait_for_service(timeout_sec=2.0):
            if rclpy.ok():
                self.get_logger().info(
                    f"Service {self.cli_pump_off.srv_name} not available, waiting again"
                )

        while not self.cli_valve_on.wait_for_service(timeout_sec=2.0):
            if rclpy.ok():
                self.get_logger().info(
                    f"Service {self.cli_valve_on.srv_name} not available, waiting again"
                )

        while not self.cli_valve_off.wait_for_service(timeout_sec=2.0):
            if rclpy.ok():
                self.get_logger().info(
                    f"Service {self.cli_valve_off.srv_name} not available, waiting again"
                )

    async def sub_joy_callback(self, msg: Joy):
        j0 = msg.axes[5] * math.pi / 2.0 - math.pi / 2.0
        j1 = msg.axes[3] * math.pi / 2.0
        j2 = msg.axes[4] * (-math.pi / 2.0) if msg.axes[4] < 0 else 0.0

        # self.get_logger().info(f"{len(msg.buttons)}")
        pump_on = bool(msg.buttons[4])
        valve_on = bool(msg.buttons[5])
        leg_down = bool(msg.buttons[0])
        leg_up = bool(msg.buttons[1])

        print(f"{pump_on, valve_on}")

        future: Future = None
        request = Trigger.Request()

        if leg_down and not self.leg_down:
            future = self.cli_leg_down.call_async(request)
            self.leg_down = True

        elif leg_up and self.leg_down:
            future = self.cli_leg_up.call_async(request)
            self.leg_down = False

        if pump_on and not self.pump_on:
            future = self.cli_pump_on.call_async(request)

        elif not pump_on and self.pump_on:
            future = self.cli_pump_off.call_async(request)

        if valve_on and not self.valve_on:
            future = self.cli_valve_on.call_async(request)

        elif not valve_on and self.valve_on:
            future = self.cli_valve_off.call_async(request)

        if future is not None:
            await future

        self.pump_on = pump_on
        self.valve_on = valve_on

        msg_js = JointState()
        msg_js.position = [j0, j1, j2, 0]
        self.pub_js_command.publish(msg_js)

        # self.get_logger().info(f"Published joint state {msg_js.position}")


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.try_shutdown()
