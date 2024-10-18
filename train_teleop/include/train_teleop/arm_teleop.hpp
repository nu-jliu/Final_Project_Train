#ifndef TRAIN_TELEOP__ARM_TELEOP_HPP___
#define TRAIN_TELEOP__ARM_TELEOP_HPP___

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <std_srvs/srv/trigger.hpp>

namespace train_teleop
{
class ArmTeleop : public rclcpp::Node
{
public:
    ArmTeleop();
    // ~ArmTeleop();

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_leg_up_;
    // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_leg_down_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_pump_on_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_pump_off_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_valve_on_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_valve_off_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_command_;

    bool pump_on_;
    bool valve_on_;
    bool leg_up_;

    void sub_joy_callback_(const sensor_msgs::msg::Joy::SharedPtr msg);
};
}  // namespace train_teleop
#endif  // TRAIN_TELEOP__ARM_TELEOP_HPP___
