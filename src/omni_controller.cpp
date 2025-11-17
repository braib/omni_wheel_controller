#include "omni_wheel_controller/omni_controller.hpp"
#include "omni_wheel_controller/kinematics.hpp"
#include "omni_wheel_controller/odom_integrator.hpp"

#include <cmath>
#include <algorithm>
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

namespace omni_wheel_controller
{

OmniController::OmniController()
: ControllerInterface(),
  wheel_radius_(0.05), center_to_wheel_(0.15), max_wheel_rad_s_(50.0),
  cmd_topic_("/cmd_vel"), cmd_timeout_ms_(500),
  N_(3), base_rotation_deg_(0.0), wheel_spacing_deg_(120.0),
  odom_frame_("odom"), base_frame_("base_link"),
  cov_x_(0.001), cov_y_(0.001), cov_yaw_(0.001)
{
  // default wheel names
  wheels_ = {"wheel1","wheel2","wheel3"};
}

controller_interface::CallbackReturn OmniController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  
  // Try to get parameters - they may already be declared from YAML
  // Use get_parameter_or to provide defaults if not in YAML
  try {
    wheels_ = node->get_parameter("wheel_joints").as_string_array();
  } catch (const std::exception&) {
    node->declare_parameter<std::vector<std::string>>("wheel_joints", wheels_);
    wheels_ = node->get_parameter("wheel_joints").as_string_array();
  }
  
  try {
    wheel_radius_ = node->get_parameter("wheel_radius").as_double();
  } catch (const std::exception&) {
    node->declare_parameter<double>("wheel_radius", wheel_radius_);
    wheel_radius_ = node->get_parameter("wheel_radius").as_double();
  }
  
  try {
    center_to_wheel_ = node->get_parameter("center_to_wheel").as_double();
  } catch (const std::exception&) {
    node->declare_parameter<double>("center_to_wheel", center_to_wheel_);
    center_to_wheel_ = node->get_parameter("center_to_wheel").as_double();
  }
  
  try {
    max_wheel_rad_s_ = node->get_parameter("max_wheel_rad_s").as_double();
  } catch (const std::exception&) {
    node->declare_parameter<double>("max_wheel_rad_s", max_wheel_rad_s_);
    max_wheel_rad_s_ = node->get_parameter("max_wheel_rad_s").as_double();
  }
  
  try {
    cmd_topic_ = node->get_parameter("cmd_topic").as_string();
  } catch (const std::exception&) {
    node->declare_parameter<std::string>("cmd_topic", cmd_topic_);
    cmd_topic_ = node->get_parameter("cmd_topic").as_string();
  }
  
  try {
    cmd_timeout_ms_ = node->get_parameter("cmd_timeout_ms").as_int();
  } catch (const std::exception&) {
    node->declare_parameter<int>("cmd_timeout_ms", cmd_timeout_ms_);
    cmd_timeout_ms_ = node->get_parameter("cmd_timeout_ms").as_int();
  }
  
  try {
    base_rotation_deg_ = node->get_parameter("base_rotation_deg").as_double();
  } catch (const std::exception&) {
    node->declare_parameter<double>("base_rotation_deg", base_rotation_deg_);
    base_rotation_deg_ = node->get_parameter("base_rotation_deg").as_double();
  }
  
  try {
    wheel_spacing_deg_ = node->get_parameter("wheel_spacing_deg").as_double();
  } catch (const std::exception&) {
    node->declare_parameter<double>("wheel_spacing_deg", wheel_spacing_deg_);
    wheel_spacing_deg_ = node->get_parameter("wheel_spacing_deg").as_double();
  }
  
  try {
    odom_frame_ = node->get_parameter("odom_frame").as_string();
  } catch (const std::exception&) {
    node->declare_parameter<std::string>("odom_frame", odom_frame_);
    odom_frame_ = node->get_parameter("odom_frame").as_string();
  }
  
  try {
    base_frame_ = node->get_parameter("base_frame").as_string();
  } catch (const std::exception&) {
    node->declare_parameter<std::string>("base_frame", base_frame_);
    base_frame_ = node->get_parameter("base_frame").as_string();
  }
  
  try {
    cov_x_ = node->get_parameter("cov_x").as_double();
  } catch (const std::exception&) {
    node->declare_parameter<double>("cov_x", cov_x_);
    cov_x_ = node->get_parameter("cov_x").as_double();
  }
  
  try {
    cov_y_ = node->get_parameter("cov_y").as_double();
  } catch (const std::exception&) {
    node->declare_parameter<double>("cov_y", cov_y_);
    cov_y_ = node->get_parameter("cov_y").as_double();
  }
  
  try {
    cov_yaw_ = node->get_parameter("cov_yaw").as_double();
  } catch (const std::exception&) {
    node->declare_parameter<double>("cov_yaw", cov_yaw_);
    cov_yaw_ = node->get_parameter("cov_yaw").as_double();
  }

  // basic checks
  N_ = static_cast<int>(wheels_.size());
  if (N_ != 3) {
    RCLCPP_ERROR(node->get_logger(), "Expected 3 wheels, got %d", N_);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (wheel_radius_ <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Invalid wheel_radius");
    return controller_interface::CallbackReturn::ERROR;
  }

  // compute theta
  theta_.resize(N_);
  for (int i=0;i<N_;++i) theta_[i] = (base_rotation_deg_ + i * wheel_spacing_deg_) * M_PI / 180.0;

  // subscribe
  cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    cmd_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&OmniController::cmd_cb, this, std::placeholders::_1));
  last_cmd_time_ = node->now();

  // odom pub + tf
  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  odom_.reset_pose();

  RCLCPP_INFO(node->get_logger(), "OmniController configured");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniController::on_activate(const rclcpp_lifecycle::State &)
{
  last_cmd_time_ = get_node()->now();
  odom_.reset_pose();
  RCLCPP_INFO(get_node()->get_logger(), "OmniController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OmniController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.clear();
  for (const auto & j : wheels_) cfg.names.push_back(j + "/velocity");
  return cfg;
}

controller_interface::InterfaceConfiguration OmniController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.clear();
  for (const auto & j : wheels_) cfg.names.push_back(j + "/velocity");
  return cfg;
}

controller_interface::return_type OmniController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto node = get_node();
  double dt = period.seconds();
  if (dt <= 0.0) return controller_interface::return_type::OK;

  // watchdog
  if ((time - last_cmd_time_).nanoseconds() / 1e6 > cmd_timeout_ms_) {
    last_vx_ = 0.0;
    last_vy_ = 0.0;
    last_omega_ = 0.0;
  }

  // compute wheel commands from last cmd
  std::array<double,3> robot_vel{last_vx_.load(), last_vy_.load(), last_omega_.load()};
  auto wheel_rates = robot_vel_to_wheel_rates(robot_vel, theta_, wheel_radius_, center_to_wheel_);

  // write commands to command_interfaces_ (protected member, ordered as requested)
  if (command_interfaces_.size() < static_cast<size_t>(N_)) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "command interfaces not ready");
    return controller_interface::return_type::OK;
  }
  for (size_t i=0;i<static_cast<size_t>(N_);++i) {
    double v = wheel_rates[i];
    // clamp
    if (v > max_wheel_rad_s_) v = max_wheel_rad_s_;
    if (v < -max_wheel_rad_s_) v = -max_wheel_rad_s_;
    command_interfaces_[i].set_value(v);
  }

  // read wheel velocities from state_interfaces_ and compute odom
  if (state_interfaces_.size() < static_cast<size_t>(N_)) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "state interfaces not ready; odom skipped");
    return controller_interface::return_type::OK;
  }

  std::array<double,3> wheel_vel_lin{};
  for (size_t i=0;i<static_cast<size_t>(N_);++i) {
    double w_ang = state_interfaces_[i].get_value(); // rad/s
    wheel_vel_lin[i] = w_ang * wheel_radius_;
  }

  auto robot_vel_from_wheels = wheel_rates_to_robot_vel(wheel_vel_lin, theta_, center_to_wheel_);
  double vx = robot_vel_from_wheels[0];
  double vy = robot_vel_from_wheels[1];
  double omega = robot_vel_from_wheels[2];

  // integrate
  odom_.integrate_body(vx, vy, omega, dt);

  // publish odom and tf
  auto odom_msg = odom_.make_odom_msg(time, odom_frame_, base_frame_, cov_x_, cov_y_, cov_yaw_);
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = omega;
  odom_pub_->publish(odom_msg);

  auto tf_msg = odom_.make_tf(time, odom_frame_, base_frame_);
  tf_broadcaster_->sendTransform(tf_msg);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn OmniController::on_deactivate(const rclcpp_lifecycle::State &)
{
  cmd_sub_.reset();
  odom_pub_.reset();
  tf_broadcaster_.reset();
  RCLCPP_INFO(get_node()->get_logger(), "OmniController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniController::on_cleanup(const rclcpp_lifecycle::State &)
{
  // nothing extra
  return controller_interface::CallbackReturn::SUCCESS;
}

void OmniController::cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_vx_ = msg->linear.x;
  last_vy_ = msg->linear.y;
  last_omega_ = msg->angular.z;
  last_cmd_time_ = get_node()->now();
}

} // namespace

PLUGINLIB_EXPORT_CLASS(omni_wheel_controller::OmniController, controller_interface::ControllerInterface)