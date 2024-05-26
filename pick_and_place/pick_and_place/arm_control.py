import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger, Trigger_Request, Trigger_Response

from pick_and_place import MaxArm_ctl
from train_interfaces.srv import (
    MoveArm,
    MoveArm_Request,
    MoveArm_Response,
    GetPosition,
    GetPosition_Request,
    GetPosition_Response,
)

# import MaxArm_ctl
import time
import serial
import scanf


class ArmControl(Node):

    def __init__(self):
        super().__init__("arm_con")
        self.callback_group = ReentrantCallbackGroup()
        self.arm_ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200)
        self.get_logger().info(f"Serial Device connected: {self.arm_ser}")

        self.srv_open_suction = self.create_service(
            Trigger,
            "grab",
            self.srv_open_suction_callback,
        )
        self.srv_close_suction = self.create_service(
            Trigger,
            "release",
            self.srv_close_suction_callback,
        )
        self.srv_get_xyz = self.create_service(
            GetPosition,
            "read_xyz",
            self.srv_get_xyz_callback,
        )
        self.srv_go_to_postion = self.create_service(
            MoveArm,
            "move_arm",
            self.srv_move_arm_callback,
        )
        self.srv_go_home = self.create_service(
            Trigger,
            "go_home",
            self.srv_go_home_callback,
        )

    def wait_until_data(self):
        while True:
            if self.arm_ser.in_waiting > 0:
                break

            self.get_logger().warn(f"Data unavailable, continue waiting ...")

        time.sleep(0.1)

    def srv_open_suction_callback(
        self, request: Trigger_Request, response: Trigger_Response
    ):
        # self.arm.set_SuctioNnozzle(3)
        self.arm_ser.reset_input_buffer()
        self.arm_ser.write("nozzle.on()\r\n".encode())
        self.wait_until_data()

        self.get_logger().info(f"Received response: {self.arm_ser.read_all().decode()}")
        # time.sleep(2)

        response.success = True
        response.message = "Suction cup enabled"
        return response

    def srv_close_suction_callback(
        self, request: Trigger_Request, response: Trigger_Response
    ):
        # self.arm.set_SuctioNnozzle(0)
        self.arm_ser.reset_input_buffer()
        self.arm_ser.write("nozzle.off()\r\n".encode())
        self.wait_until_data()

        self.get_logger().info(f"Received response: {self.arm_ser.read_all().decode()}")
        # time.sleep(2)

        response.success = True
        response.message = "Suction cup disabled"
        return response

    def srv_get_xyz_callback(
        self, request: GetPosition_Request, response: GetPosition_Response
    ):
        # xyz = self.arm.read_xyz()
        self.arm_ser.reset_input_buffer()
        self.arm_ser.write("arm.position\r\n".encode())
        self.wait_until_data()

        xyz = self.arm_ser.read_all().decode()
        # self.get_logger().info(f"Received response: {xyz}")

        lines = xyz.splitlines()
        x, y, z = scanf.scanf("(%f, %f, %f)", lines[1])
        self.get_logger().info(f"x: {x}, y: {y}, z: {z}")

        response.position.x = x
        response.position.y = y
        response.position.z = z

        return response

    def srv_move_arm_callback(
        self, request: MoveArm_Request, response: MoveArm_Response
    ):
        x = request.position.x
        y = request.position.y
        z = request.position.z

        self.arm_ser.write(f"arm.set_position(({x}, {y}, {z}), 3000)\r\n".encode())
        self.wait_until_data()

        self.get_logger().info(f"Received response: {self.arm_ser.read_all().decode()}")

        response.success = True
        return response

    def srv_go_home_callback(
        self, request: Trigger_Request, response: Trigger_Response
    ):
        self.arm_ser.write(f"arm.set_position((163.0, 0.0, 212.0), 3000)\r\n".encode())
        self.wait_until_data()

        self.get_logger().info(f"Received response: {self.arm_ser.read_all().decode()}")

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node_arm_control = ArmControl()
    rclpy.spin(node=node_arm_control)

    node_arm_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    am = MaxArm_ctl.MaxArm_ctl(device="/dev/ttyUSB0", baudrate=9600)
    am.set_SuctioNnozzle(1)
    time.sleep(2)
