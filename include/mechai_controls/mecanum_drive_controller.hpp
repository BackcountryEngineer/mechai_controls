#ifndef MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_
#define MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <controller_interface/controller_interface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace mecanum_drive_controller {
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // ROS 2 integrated controller for mecanum wheel base chasises. A mechanum wheeled robot uses wheels that are composed of slanted rollers, affording the ability have the full degree of freedom in a plane, including in place rotations. This controller assumes a X configuration of the wheels looking from top down, as this configuration allows for much better rotation control.
  class MecanumDriveController : public controller_interface::ControllerInterface {
    using Twist = geometry_msgs::msg::TwistStamped;

    public:
      MecanumDriveController();

      controller_interface::InterfaceConfiguration command_interface_configuration() const override;

      controller_interface::InterfaceConfiguration state_interface_configuration() const override;

      controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

      CallbackReturn on_init() override;

      CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

      CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

      CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

      CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

      CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

      CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

    protected:
      struct WheelHandle {
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
      };

      std::vector<std::string> wheel_names_;
      std::vector<WheelHandle> registered_wheel_handles_;

      struct WheelParams {
        double separation = 0.0;  // w.r.t. the midpoint of the wheel width
        double radius = 0.0;      // Assumed to be the same for both wheels
        double separation_multiplier = 1.0;
      } wheel_params_;

      struct OdometryParams {
        bool open_loop = false;
        bool position_feedback = true;
        bool enable_odom_tf = true;
        std::string base_frame_id = "base_link";
        std::string odom_frame_id = "odom";
      } odom_params_;

      bool subscriber_is_active_ = false;
      rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        velocity_command_unstamped_subscriber_ = nullptr;

      realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

      auto feedback_type() const;

      bool is_halted;
      std::chrono::milliseconds cmd_vel_timeout_{500};

      bool reset();
      void halt();

  };
} // namespace mecanum_drive_controller
#endif  // MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_