from adafruit_servokit import ServoKit
import math
import copy

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from std_srvs.srv import Trigger, Trigger_Request, Trigger_Response


class ServoControl(Node):

    def __init__(self):
        super().__init__("servo_control")
        self.callback_group = ReentrantCallbackGroup()
        self.get_logger().info("Initializing Servo")

        self.servo_kit = ServoKit(channels=16)

        self.get_logger().info("Servo initialized")

        self.forward_speed = 0.5
        self.backward_speed = -0.3

        self.home_pos = [180.0, 100.0, 20.0, 80.0]
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

        self.srv_gripper_open = self.create_service(
            Trigger,
            "gripper/open",
            self.srv_gripper_open_callback,
            callback_group=self.callback_group,
        )

        self.srv_gripper_close = self.create_service(
            Trigger,
            "gripper/close",
            self.srv_gripper_close_callback,
            callback_group=self.callback_group,
        )

        self.sub_js_command = self.create_subscription(
            JointState,
            "arm/js_command",
            self.sub_js_command_callback,
            10,
            callback_group=self.callback_group,
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            "cmd_vel",
            self.sub_cmd_vel_callback,
            10,
            callback_group=self.callback_group,
        )

        self.pub_joint_states = self.create_publisher(JointState, "joint_states", 10)

        self.open_gripper()
        self.leg_up()

    def __del__(self):
        for i in range(12):
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

            self.servo_kit.servo[i + 8].angle = target
            self.current_pos[i] = target

            self.get_logger().info(f"Set servo {i} to position {target}")

        self.publish_joint_angles()

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

    def sub_cmd_vel_callback(self, msg: Twist):
        speed = msg.linear.x

        if speed > 0.2:
            self.forward()
        elif speed < -0.2:
            self.backward()
        else:
            self.stop()

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

    def srv_gripper_open_callback(
        self,
        request: Trigger_Request,
        response: Trigger_Response,
    ):
        self.open_gripper()

        response.success = True
        return response

    def srv_gripper_close_callback(
        self,
        request: Trigger_Request,
        response: Trigger_Response,
    ):
        self.close_gripper()

        response.success = True
        return response

    def open_gripper(self):
        self.get_logger().info("Opening gripper")
        self.servo_kit.servo[4].angle = 0

    def close_gripper(self):
        self.get_logger().info("Closing gripper")
        self.servo_kit.servo[4].angle = 120

    def leg_down(self):
        self.get_logger().info("Putting leg down")
        self.servo_kit.servo[8].angle = 100
        self.servo_kit.servo[9].angle = 105

    def leg_up(self):
        self.get_logger().info("Putting leg up")
        self.servo_kit.servo[8].angle = 160
        self.servo_kit.servo[9].angle = 40

    def publish_joint_angles(self):
        if self.check_position():
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ["base", "shoulder", "elbow", "wrist"]
            msg.position = [0.0] * 4

            for i in range(4):
                msg.position[i] = (
                    (self.current_pos[i] - self.home_pos[i]) / 160.0 * math.pi
                )

            self.pub_joint_states.publish(msg)

        else:
            self.get_logger().info("Returned a position")

    def check_position(self):
        for pos in self.current_pos:
            if pos is None:
                return False

        return True

    def forward(self):
        self.get_logger().info("Moving forward")
        self.servo_kit.continuous_servo[0].throttle = self.forward_speed
        self.servo_kit.continuous_servo[1].throttle = self.backward_speed
        self.servo_kit.continuous_servo[2].throttle = self.forward_speed
        self.servo_kit.continuous_servo[3].throttle = self.backward_speed

    def backward(self):
        self.get_logger().info("Moving backward")
        self.servo_kit.continuous_servo[0].throttle = self.backward_speed
        self.servo_kit.continuous_servo[1].throttle = self.forward_speed
        self.servo_kit.continuous_servo[2].throttle = self.backward_speed
        self.servo_kit.continuous_servo[3].throttle = self.forward_speed

    def stop(self):
        self.servo_kit.continuous_servo[0].throttle = 0.1
        self.servo_kit.continuous_servo[1].throttle = 0.1
        self.servo_kit.continuous_servo[2].throttle = 0.1
        self.servo_kit.continuous_servo[3].throttle = 0.1


def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()

    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.try_shutdown()
