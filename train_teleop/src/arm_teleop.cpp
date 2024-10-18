#include <memory>
#include "train_teleop/arm_teleop.hpp"

namespace train_teleop
{
ArmTeleop::ArmTeleop() : rclcpp::Node("arm_teleop")
{
    sub_joy_ = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&ArmTeleop::sub_joy_callback_, this, std::placeholders::_1));
    pub_js_command_ = create_publisher<sensor_msgs::msg::JointState>("arm/js_command", 10);
}

void ArmTeleop::sub_joy_callback_(const sensor_msgs::msg::Joy::SharedPtr msg) {
    const double x = msg->axes.at(3);
    const double y = msg->axes.at(4);
    const double z = msg->axes.at(5);

    RCLCPP_INFO_STREAM(get_logger(), "Get joy states: " << x << ", " << y << ", " << z);
}
}  // namespace train_teleop


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<train_teleop::ArmTeleop>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

