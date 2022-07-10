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
      auto_declare<double>("wheel_separation_w", wheel_params_.separation_w);
      auto_declare<double>("wheel_separation_l", wheel_params_.separation_l);
      auto_declare<double>("wheel_radius", wheel_params_.radius);

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

  controller_interface::return_type MecanumDriveController::update(const rclcpp::Time & time, const rclcpp::Duration &) {
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
      last_command_msg->twist.linear.y = 0.0;
      last_command_msg->twist.angular.z = 0.0;
    }

    Twist command = *last_command_msg;
    double & linear_x = command.twist.linear.x;
    double & linear_y = command.twist.linear.y;
    double & angular_z = command.twist.angular.z;

    update_wheel_velocities(linear_x, linear_y, angular_z);

    return controller_interface::return_type::OK;
  }

  void MecanumDriveController::update_wheel_velocities(double vx, double vy, double va) {
    const double wheel_w_separation = wheel_params_.separation_w;
    const double wheel_l_separation = wheel_params_.separation_l;
    const double wheel_diameter = wheel_params_.radius;
    
    auto v_front_left = (vx - vy - va * (wheel_w_separation + wheel_l_separation) / 2.0) / wheel_diameter * 2;
    auto v_front_right = (vx + vy + va * (wheel_w_separation + wheel_l_separation) / 2.0) / wheel_diameter * 2;
    auto v_back_left = (vx + vy - va * (wheel_w_separation + wheel_l_separation) / 2.0) / wheel_diameter * 2;
    auto v_back_right = (vx - vy + va * (wheel_w_separation + wheel_l_separation) / 2.0) / wheel_diameter * 2;

    registered_wheel_handles_[FRONT_LEFT].velocity.get().set_value(v_front_left);
    registered_wheel_handles_[FRONT_RIGHT].velocity.get().set_value(v_front_right);
    registered_wheel_handles_[BACK_LEFT].velocity.get().set_value(v_back_left);
    registered_wheel_handles_[BACK_RIGHT].velocity.get().set_value(v_back_right);
  }

  CallbackReturn MecanumDriveController::on_configure(const rclcpp_lifecycle::State &) {
    auto logger = node_->get_logger();
    wheel_names_ = node_->get_parameter("wheel_names").as_string_array();
    if (wheel_names_.empty()) {
      RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
      return CallbackReturn::ERROR;
    }

    wheel_params_.separation_w = node_->get_parameter("wheel_separation_w").as_double();
    wheel_params_.separation_l = node_->get_parameter("wheel_separation_l").as_double();
    wheel_params_.radius = node_->get_parameter("wheel_radius").as_double();

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

      const auto command_handle = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_name](const auto & interface) {
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

  CallbackReturn MecanumDriveController::on_deactivate(const rclcpp_lifecycle::State &) {
    subscriber_is_active_ = false;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_cleanup(const rclcpp_lifecycle::State &) {
    if (!reset()) {
      return CallbackReturn::ERROR;
    }
    received_velocity_msg_ptr_.set(std::make_shared<Twist>());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_error(const rclcpp_lifecycle::State &) {
    RCLCPP_WARN(node_->get_logger(), "Error deteced resetting internal");
    if (!reset()) {
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  bool MecanumDriveController::reset() {
    std::queue<Twist> empty;
    std::swap(previous_commands_, empty);

    registered_wheel_handles_.clear();

    subscriber_is_active_ = false;
    velocity_command_subscriber_.reset();
    velocity_command_unstamped_subscriber_.reset();

    received_velocity_msg_ptr_.set(nullptr);

    is_halted = false;
    return true;
  }

  CallbackReturn MecanumDriveController::on_shutdown(const rclcpp_lifecycle::State &) {
    return CallbackReturn::SUCCESS;
  }
}