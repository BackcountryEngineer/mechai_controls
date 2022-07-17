#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <tf2/LinearMath/Quaternion.h>

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

      auto_declare<double>("publish_rate", publish_rate_);

      auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
      auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);
      
      auto_declare<bool>("open_loop", odom_params_.open_loop);
      auto_declare<bool>("position_feedback", odom_params_.position_feedback);

      auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
      auto_declare<bool>("publish_limited_velocity", publish_limited_velocity_);
      auto_declare<bool>("use_stamped_vel", use_stamped_vel_);
      auto_declare<int>("velocity_rolling_window_size", 10);
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
      RCLCPP_WARN(logger, "No latest command, recieved nullptr");
      return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = current_time - last_command_msg->header.stamp;
    if (age_of_last_command > cmd_vel_timeout_) {
      //Stop if lastest command is stale
      last_command_msg->twist.linear.x = 0.0;
      last_command_msg->twist.linear.y = 0.0;
      last_command_msg->twist.angular.z = 0.0;
    }

    Twist command = *last_command_msg;

    bound_velocity(current_time, command);

    publish_odometry(current_time, command);

    publish_velocity(current_time, command);

    update_wheel_velocities(command);

    previous_update_timestamp_ = current_time;

    return controller_interface::return_type::OK;
  }

  void MecanumDriveController::publish_odometry(const rclcpp::Time & current_time, const Twist & command) {
    //initial implementation open loop odom only
    const double dt = current_time.seconds() - previous_update_timestamp_.seconds();

    const double linear_x = command.twist.linear.x * dt;
    const double linear_y = command.twist.linear.y * dt;
    const double angular_z = command.twist.angular.z * dt;

    const double heading_old = heading_;
    const double rx = linear_x / angular_z;
    const double ry = linear_y / angular_z;
    heading_ += angular_z;
    px_ += (rx * sin(heading_) - sin(heading_old)) + (ry * cos(heading_) - cos(heading_old));
    py_ = (-rx * cos(heading_) - cos(heading_old)) + (-ry * sin(heading_) - sin(heading_old));

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, heading_);

    if (previous_publish_timestamp_ + publish_period_ < current_time) {
      previous_publish_timestamp_ += publish_period_;
      if (realtime_odometry_publisher_->trylock()) {
        auto & odom_msg = realtime_odometry_publisher_->msg_;
        odom_msg.header.stamp = current_time;
        odom_msg.pose.pose.position.x = px_;
        odom_msg.pose.pose.position.y = py_;
        odom_msg.pose.pose.orientation.x = orientation.x();
        odom_msg.pose.pose.orientation.y = orientation.y();
        odom_msg.pose.pose.orientation.z = orientation.z();
        odom_msg.pose.pose.orientation.w = orientation.w();
        odom_msg.twist.twist = command.twist;
        realtime_odometry_publisher_->unlockAndPublish();
      }
    }
  }

  void MecanumDriveController::bound_velocity(const rclcpp::Time & current_time, Twist & command) {
    //note this function will modify the pass in command to bound the command velocities
    const auto update_dt = current_time - previous_update_timestamp_;

    // auto & last_command = previous_commands_.back().twist;
    // auto & second_to_last_command = previous_commands_.front().twist;
    // limiter_linear_.limit(
    //   command.twist.linear.x, last_command.linear.x, second_to_last_command.linear.x, update_dt.seconds());
    // limiter_angular_.limit(
    //   command.twist.angular.z, last_command.angular.z, second_to_last_command.angular.z, update_dt.seconds());

    previous_commands_.pop();
    previous_commands_.emplace(command);
  }

  void MecanumDriveController::publish_bounded_velocity(const rclcpp::Time & current_time, const Twist & command) {
    if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock()) {
      auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
      limited_velocity_command.header.stamp = current_time;
      limited_velocity_command.twist = command.twist;
      realtime_limited_velocity_publisher_->unlockAndPublish();
    }
  }

  void MecanumDriveController::update_wheel_velocities(const Twist & command) {
    const double vx = command.twist.linear.x;
    const double vy = command.twist.linear.y;
    const double va = command.twist.angular.z;
    
    auto v_front_left = (vx - vy - va * (wheel_params_.separation_w + wheel_params_.separation_l) / 2.0) / wheel_params_.radius;
    auto v_front_right = (vx + vy + va * (wheel_params_.separation_w + wheel_params_.separation_l) / 2.0) / wheel_params_.radius;
    auto v_back_left = (vx + vy - va * (wheel_params_.separation_w + wheel_params_.separation_l) / 2.0) / wheel_params_.radius;
    auto v_back_right = (vx - vy + va * (wheel_params_.separation_w + wheel_params_.separation_l) / 2.0) / wheel_params_.radius;

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

    odom_params_.odom_frame_id = node_->get_parameter("odom_frame_id").as_string();
    odom_params_.base_frame_id = node_->get_parameter("base_frame_id").as_string();

    cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(node_->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};

    publish_limited_velocity_ = node_->get_parameter("publish_limited_velocity").as_bool();
    use_stamped_vel_ = node_->get_parameter("use_stamped_vel").as_bool();

    if (!reset()) {
      return CallbackReturn::ERROR;
    }

    if (publish_limited_velocity_) {
      limited_velocity_publisher_ = node_->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
      realtime_limited_velocity_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
    }

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

    //initialize odometry feedback
    odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);

    auto & odom_msg = realtime_odometry_publisher_->msg_;
    odom_msg.header.frame_id = odom_params_.odom_frame_id;
    odom_msg.child_frame_id = odom_params_.base_frame_id;

    publish_rate_ = node_->get_parameter("publish_rate").as_double();
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
    previous_publish_timestamp_ = node_->get_clock()->now();

    //fills in zeros for message values
    odom_msg.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL); 

    for (size_t idx = 0; idx < NUM_DIMENSIONS; ++idx) {
      auto diagonal_idx = NUM_DIMENSIONS * idx + idx;
      odom_msg.pose.covariance[diagonal_idx] = odom_params_.pose_covariance_diagonal[diagonal_idx];
      odom_msg.twist.covariance[diagonal_idx] = odom_params_.twist_covariance_diagonal[diagonal_idx];
    }

    previous_update_timestamp_ = node_->get_clock()->now();

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
    px_ = 0;
    py_ = 0;
    heading_ = 0;

    return true;
  }

  CallbackReturn MecanumDriveController::on_shutdown(const rclcpp_lifecycle::State &) {
    return CallbackReturn::SUCCESS;
  }
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerInterface)