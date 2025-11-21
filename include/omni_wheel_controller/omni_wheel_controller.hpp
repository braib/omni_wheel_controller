#ifndef OMNI_WHEEL_CONTROLLER__OMNI_WHEEL_CONTROLLER_HPP_
#define OMNI_WHEEL_CONTROLLER__OMNI_WHEEL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace omni_wheel_controller
{

class OmniWheelController : public controller_interface::ControllerInterface
{
public:
  OmniWheelController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // Parameters
  std::vector<std::string> wheel_joints_;
  double wheel_radius_;           // 'a' in your formula
  std::vector<double> theta_B_;   // theta_Bi for each wheel (radians)
  std::vector<double> alpha_;     // alpha_i for each wheel (radians)
  double wheel_base_radius_;      // 'l' in your formula (distance from center to wheel)
  std::string base_frame_id_;
  std::string odom_frame_id_;
  bool publish_odom_;
  bool publish_tf_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  double cmd_vel_timeout_;        // Timeout for velocity commands
  
  // Smoothing parameters
  double linear_acceleration_limit_;   // m/s²
  double angular_acceleration_limit_;  // rad/s²
  double velocity_smoothing_alpha_;    // Exponential smoothing factor (0-1)

  // Command velocity subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> cmd_vel_buffer_;
  rclcpp::Time last_cmd_vel_time_;

  // Odometry publisher
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_pub_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Command interfaces (wheel velocity commands)
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> 
    wheel_velocity_command_interfaces_;

  // State interfaces (wheel position feedback for odometry)
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
    wheel_position_state_interfaces_;

  // Odometry state
  double x_;
  double y_;
  double theta_;
  double vx_;     // Current robot velocity for odometry message
  double vy_;
  double omega_z_;
  std::vector<double> prev_wheel_positions_;
  rclcpp::Time prev_time_;
  
  // Velocity smoothing state
  double target_vx_;      // Target velocity from cmd_vel
  double target_vy_;
  double target_omega_z_;
  double current_vx_;     // Smoothed current velocity sent to wheels
  double current_vy_;
  double current_omega_z_;

  // Helper methods
  void cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
  std::vector<double> compute_wheel_velocities(double vx, double vy, double omega_z);
  void update_odometry(const rclcpp::Time & time);
  void publish_odometry(const rclcpp::Time & time);
  void publish_transform(const rclcpp::Time & time);
  bool is_cmd_vel_timeout(const rclcpp::Time & current_time);
  
  // Smoothing methods
  void apply_acceleration_limits(double dt);
  void apply_velocity_smoothing();
};

}  // namespace omni_wheel_controller

#endif  // OMNI_WHEEL_CONTROLLER__OMNI_WHEEL_CONTROLLER_HPP_