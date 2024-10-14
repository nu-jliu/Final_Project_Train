from gpiozero import LED

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger, Trigger_Request, Trigger_Response


class PumpControl(Node):

    def __init__(self):
        super().__init__("pump_control")
        self.callback_group = ReentrantCallbackGroup()

        self.pin_pump = 25
        self.pin_valve = 26

        self.pump = LED(self.pin_pump)
        self.valve = LED(self.pin_valve)

        self.pump.on()
        self.valve.on()

        self.srv_pump_on = self.create_service(
            Trigger,
            "pump/on",
            self.srv_pump_on_callback,
            callback_group=self.callback_group,
        )

        self.srv_pump_off = self.create_service(
            Trigger,
            "pump/off",
            self.srv_pump_off_callback,
            callback_group=self.callback_group,
        )

        self.srv_valve_on = self.create_service(
            Trigger,
            "valve/on",
            self.srv_valve_on_callback,
            callback_group=self.callback_group,
        )

        self.srv_valve_off = self.create_service(
            Trigger,
            "valve/off",
            self.srv_valve_off_callback,
            callback_group=self.callback_group,
        )

    def srv_pump_on_callback(
        self, request: Trigger_Request, response: Trigger_Response
    ):
        self.get_logger().info("Pump on")
        self.pump.off()
        response.success = True
        return response

    def srv_pump_off_callback(
        self, request: Trigger_Request, response: Trigger_Response
    ):
        self.get_logger().info("Pump off")
        self.pump.on()
        response.success = True
        return response

    def srv_valve_on_callback(
        self, request: Trigger_Request, response: Trigger_Response
    ):
        self.get_logger().info("Valve on")
        self.valve.off()
        response.success = True
        return response

    def srv_valve_off_callback(
        self, request: Trigger_Request, response: Trigger_Response
    ):
        self.get_logger().info("Valve off")
        self.valve.on()
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PumpControl()

    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.try_shutdown()
