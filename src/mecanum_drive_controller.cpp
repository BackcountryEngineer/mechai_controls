#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "mechai_controls/mecanum_drive_controller.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // local constants

namespace mecanum_drive_controller {
  using namespace std::chrono_literals;
  using controller_interface::interface_configuration_type;
  using controller_interface::InterfaceConfiguration;
  using hardware_interface::HW_IF_POSITION;
  using hardware_interface::HW_IF_VELOCITY;
  using lifecycle_msgs::msg::State;

  MecanumDriveController::MecanumDriveController() : controller_interface::ControllerInterface() {}

  auto MecanumDriveController::feedback_type() const {
    return odom_params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
  }

  void MecanumDriveController::halt() {
    for (const auto & wheel_handle : registered_wheel_handles_) {
      wheel_handle.velocity.get().set_value(0.0);
    }
  }

  CallbackReturn MecanumDriveController::on_init() {
    try {
      auto_declare<std::vector<std::string>>("wheel_names", std::vector<std::string>());
      auto_declare<double>("wheel_separation", wheel_params_.separation);
      auto_declare<double>("wheel_radius", wheel_params_.radius);
      auto_declare<double>("wheel_separation_multiplier", wheel_params_.separation_multiplier);

      auto_declare<bool>("position_feedback", odom_params_.position_feedback);

      auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    } catch (const std::exception &e) {
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  InterfaceConfiguration MecanumDriveController::command_interface_configuration() const {
    std::vector<std::string> conf_names;
    for (const auto & joint_name : wheel_names_) {
      conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
    }
    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  InterfaceConfiguration MecanumDriveController::state_interface_configuration() const {
    std::vector<std::string> conf_names;
    for (const auto & joint_name : wheel_names_) {
      conf_names.push_back(joint_name + "/" + feedback_type());
    }

    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  controller_interface::return_type MecanumDriveController::update(const rclcpp::Time & time, const rclcpp::Duration & period) {
    auto logger = node_->get_logger();
    if (get_state().id() == State::PRIMARY_STATE_INACTIVE) {
      if (!is_halted) {
        halt();
        is_halted = true;
      }

      return controller_interface::return_type::OK;
    }

    const auto current_time = time;

    std::shared_ptr<Twist> last_command_msg;
    received_velocity_msg_ptr_.get(last_command_msg);

    if (last_command_msg == nullptr) {
      RCLCPP_WARN(logger, "Last recieved command is nullptr");
      return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = current_time - last_command_msg->header.stamp;
    if (age_of_last_command > cmd_vel_timeout_) {
      last_command_msg->twist.linear.x = 0.0;
      last_command_msg->twist.angular.z =0.0;
    }

    Twist command = *last_command_msg;
    double & linear_command = command.twist.linear.x;
    double & angular_command = command.twist.angular.z;

    for (const auto & wheel_handle : registered_wheel_handles_) {
      //Wheel velocity placeholder calculations
      wheel_handle.velocity.get().set_value(linear_command + angular_command);
    }

    return controller_interface::return_type::OK;
  }

  // CallbackReturn MecanumDriveController::on_configure(const rclcpp_lifecycle::State&) {
  //   auto logger = node_->get_logger();
  // }
}