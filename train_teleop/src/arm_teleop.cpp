#include <memory>
#include <chrono>
#include "train_teleop/arm_teleop.hpp"

#include <std_srvs/srv/trigger.hpp>
#include "arm_kinematics/kinematics.hpp"

namespace train_teleop
{
using namespace std::chrono_literals;

ArmTeleop::ArmTeleop()
: rclcpp::Node("arm_teleop"), js_ready_(false)
{
  js_command_.position = {0.0, 0.0, 0.0, 0.0};
  RCLCPP_INFO_STREAM(get_logger(), static_cast<int>(js_current_.position.size()));
  // js_command_.name = {"joint0"}

  timer_ = create_wall_timer(0.01s, std::bind(&ArmTeleop::timer_callback_, this));

  srv_move_arm_ = create_service<train_interfaces::srv::MoveArm>(
    "move_arm",
    std::bind(
      &ArmTeleop::srv_move_arm_callback_,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );

  cli_pump_on_ = create_client<std_srvs::srv::Trigger>("pump/on");
  cli_pump_off_ = create_client<std_srvs::srv::Trigger>("pump/off");
  cli_valve_on_ = create_client<std_srvs::srv::Trigger>("valve/on");
  cli_valve_off_ = create_client<std_srvs::srv::Trigger>("valve/off");

//   while (!cli_pump_on_->wait_for_service(2.0s)) {
//     RCLCPP_WARN_STREAM(
//       get_logger(),
//       "Service " << cli_pump_on_->get_service_name() << "not available, waiting again");
//   }

//   while (!cli_pump_off_->wait_for_service(2.0s)) {
//     RCLCPP_WARN_STREAM(
//       get_logger(),
//       "Service " << cli_pump_off_->get_service_name() << "not available, waiting again");
//   }

//   while (!cli_valve_on_->wait_for_service(2.0s)) {
//     RCLCPP_WARN_STREAM(
//       get_logger(),
//       "Service " << cli_valve_on_->get_service_name() << "not available, waiting again");
//   }

//   while (!cli_valve_off_->wait_for_service(2.0s)) {
//     RCLCPP_WARN_STREAM(
//       get_logger(),
//       "Service " << cli_valve_off_->get_service_name() << "not available, waiting again");
//   }

  sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy",
    10,
    std::bind(&ArmTeleop::sub_joy_callback_, this, std::placeholders::_1)
  );
  sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    10,
    std::bind(&ArmTeleop::sub_js_callback_, this, std::placeholders::_1)
  );

  pub_js_command_ = create_publisher<sensor_msgs::msg::JointState>("arm/js_command", 10);

  RCLCPP_INFO(get_logger(), "Arm teleop node initialized");
}

void ArmTeleop::timer_callback_()
{
  pub_js_command_->publish(js_command_);
}

void ArmTeleop::sub_joy_callback_(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  const double x = msg->axes.at(3);
  const double y = msg->axes.at(4);
  const double z = msg->axes.at(5);

  RCLCPP_INFO_STREAM(get_logger(), "Get joy states: " << x << ", " << y << ", " << z);
}

void ArmTeleop::sub_js_callback_(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  js_current_ = *msg;

  if (!js_ready_) {
    js_ready_ = true;
  }
}

void ArmTeleop::srv_move_arm_callback_(
  const train_interfaces::srv::MoveArm::Request::SharedPtr request,
  const train_interfaces::srv::MoveArm::Response::SharedPtr response
)
{
  if (!js_ready_) {
    RCLCPP_WARN(get_logger(), "Joint state not ready yet");
    response->success = false;
    // return;
  }

  const auto x = request->position.x;
  const auto y = request->position.y;
  const auto z = request->position.z;

  RCLCPP_INFO_STREAM(get_logger(), static_cast<int>(js_current_.position.size()));

  const auto j0 = js_current_.position.at(0);
  const auto j1 = js_current_.position.at(1);
  const auto j2 = js_current_.position.at(2);

  const auto result = arm::compute_ik({x, y, z}, {j0, j1, j2});

  const auto joints = std::get<0>(result);
  const auto success = std::get<1>(result);

  if (success) {
    RCLCPP_INFO_STREAM(get_logger(), "Calculated joint angles: " << joints);

    js_command_.position.at(0) = arm::normalize_angle(joints.j0);
    js_command_.position.at(1) = arm::normalize_angle(joints.j1);
    js_command_.position.at(2) = arm::normalize_angle(joints.j2);
    // RCLCPP_INFO_STREAM(get_logger(), success);
  } else {
    RCLCPP_WARN(get_logger(), "Failed to find the solution");
  }

  response->success = success;
}
}  // namespace train_teleop


/// \brief Main function of the arm teleop node
/// \param argc number of arguments
/// \param argv arguments
/// \return result code
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<train_teleop::ArmTeleop>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
