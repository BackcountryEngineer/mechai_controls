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
      auto_declare<double>("wheel_radius_multiplier", wheel_params_.radius_multiplier);

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

  CallbackReturn MecanumDriveController::on_configure(const rclcpp_lifecycle::State &) {
    auto logger = node_->get_logger();
    wheel_names_ = node_->get_parameter("wheel_names").as_string_array();
    if (wheel_names_.empty()) {
      RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
      return CallbackReturn::ERROR;
    }

    wheel_params_.separation = node_->get_parameter("wheel_separation").as_double();
    wheel_params_.radius = node_->get_parameter("wheel_radius").as_double();
    wheel_params_.separation_multiplier = node_->get_parameter("wheel_separation_multiplier").as_double();
    wheel_params_.radius_multiplier = node_->get_parameter("wheel_radius_multiplier").as_double();

    const double wheel_separation = wheel_params_.separation_multiplier * wheel_params_.separation;
    const double wheel_radius = wheel_params_.radius_multiplier * wheel_params_.radius;

    odom_params_.open_loop = node_->get_parameter("open_loop").as_bool();
    odom_params_.position_feedback = node_->get_parameter("position_feedback").as_bool();

    cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(node_->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};

    const Twist empty_twist;
    received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

    previous_commands_.emplace(empty_twist);
    previous_commands_.emplace(empty_twist);

    if (use_stamped_vel_) {
      velocity_command_subscriber_ = node_->create_subscription<Twist>(DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<Twist> msg) -> void {
        if (!subscriber_is_active_) {
          RCLCPP_WARN(node_->get_logger(), "Command subscriber inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
          RCLCPP_WARN_ONCE(node_->get_logger(), "Received command with zero timestamp, setting it to current time");
          msg->header.stamp = node_->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
    } else {
      velocity_command_unstamped_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
        if (!subscriber_is_active_) {
          RCLCPP_WARN(node_->get_logger(), "Command subscriber inactive");
          return;
        }

        std::shared_ptr<Twist> twist_stamped;
        received_velocity_msg_ptr_.get(twist_stamped);
        twist_stamped->twist = *msg;
        twist_stamped->header.stamp = node_->get_clock()->now();
      });
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_activate(const rclcpp_lifecycle::State &) {
    if (!configure_wheel_handles(wheel_names_, registered_wheel_handles_)) {
      return CallbackReturn::ERROR;
    }

    if (registered_wheel_handles_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Can't find wheel interfaces");
      return CallbackReturn::ERROR;
    }

    is_halted = false;
    subscriber_is_active_ = true;
    RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
  }

  bool MecanumDriveController::configure_wheel_handles(const std::vector<std::string> & wheel_names, std::vector<WheelHandle> & registered_handles) {
    auto logger = node_->get_logger();

    if (wheel_names.empty()) {
      RCLCPP_ERROR(logger, "No wheel names specified");
      return false;
    }

    registered_handles.reserve(wheel_names.size());
    for (const auto & wheel_name : wheel_names) {
      const auto interface_name = feedback_type();
      const auto state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_name, &interface_name](const auto & interface) {
        return interface.get_name() == wheel_name &&
               interface.get_interface_name() == interface_name;
      });

      if (state_handle == state_interfaces_.cend()) {
        RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
        return false;
      }

      const auto command_handle = std::find_if(command_interfaces_.cbegin(), command_interfaces_.cend(), [&wheel_name](const auto & interface) {
        return interface.get_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

      if (command_handle == command_interfaces_.cend()) {
        RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
        return false;
      }

      registered_handles.emplace_back(
        WheelHandle{
          std::ref(*state_handle),
          std::ref(*command_handle),
        }
      );
    }

    return true;
  }
}