#ifndef MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_
#define MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_

#include <queue>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <memory>

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
  constexpr size_t NUM_DIMENSIONS = 6;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /*
    ROS 2 integrated controller for mecanum wheel base chasises. A mechanum wheeled robot uses wheels that are composed of slanted rollers, affording the ability for full degree of freedom in a plane, including in place rotations. This controller assumes a X configuration of the wheels looking from top down, as this configuration allows for much better rotation control.

    Make use of the kinematic model derived here:
    https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
  */
  class MecanumDriveController : public controller_interface::ControllerInterface {
    using Twist = geometry_msgs::msg::TwistStamped;

    public:
      enum wheels {
        FRONT_LEFT = 0,
        FRONT_RIGHT = 1,
        BACK_LEFT = 2,
        BACK_RIGHT = 3,
      };

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
        double separation_w = 0.0;  // separation side to side
        double separation_l = 0.0; //separation front and back
        double radius = 0.0;      // same for all wheels
      } wheel_params_;

      struct OdometryParams {
        bool open_loop = false;
        bool position_feedback = true;
        std::string base_frame_id = "base_link";
        std::string odom_frame_id = "odom";
        std::array<double, NUM_DIMENSIONS> pose_covariance_diagonal;
        std::array<double, NUM_DIMENSIONS> twist_covariance_diagonal;
      } odom_params_;

      std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
      std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_ = nullptr;

      bool subscriber_is_active_ = false;
      rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        velocity_command_unstamped_subscriber_ = nullptr;

      std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
      std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ = nullptr;

      realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

      std::queue<Twist> previous_commands_;  // last two commands

      auto feedback_type() const;
      void bound_velocity(const rclcpp::Time & current_time, Twist & command);
      void publish_odometry(const rclcpp::Time & current_time, const Twist & command);
      void publish_velocity(const rclcpp::Time & current_time, const Twist & command);
      void update_wheel_velocities(const Twist & command);

      std::chrono::milliseconds cmd_vel_timeout_{500};

      double px_ = 0;
      double py_ = 0;
      double heading_ = 0;

      bool is_halted = false;

      double publish_rate_ = 50.0;
      bool publish_limited_velocity_ = false;
      bool use_stamped_vel_ = true;

      rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);

      rclcpp::Time previous_update_timestamp_{0};
      rclcpp::Time previous_publish_timestamp_{0};

      bool reset();
      void halt();
      bool configure_wheel_handles(const std::vector<std::string> & wheel_names, std::vector<WheelHandle> & registered_handles);

  };
} // namespace mecanum_drive_controller
#endif  // MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_