#include "omni_wheel_controller/omni_wheel_controller.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace omni_wheel_controller
{

OmniWheelController::OmniWheelController()
: controller_interface::ControllerInterface(),
  wheel_radius_(0.05),
  wheel_base_radius_(0.2),
  base_frame_id_("base_link"),
  odom_frame_id_("odom"),
  publish_odom_(true),
  publish_tf_(true),
  cmd_vel_topic_("~/cmd_vel"),
  odom_topic_("~/odom"),
  cmd_vel_timeout_(0.5),
  x_(0.0),
  y_(0.0),
  theta_(0.0),
  vx_(0.0),
  vy_(0.0),
  omega_z_(0.0)
{
}

controller_interface::CallbackReturn OmniWheelController::on_init()
{
  try
  {
    // Declare parameters
    auto_declare<std::vector<std::string>>("wheel_joints", std::vector<std::string>());
    auto_declare<double>("wheel_radius", 0.05);
    auto_declare<std::vector<double>>("theta_B", std::vector<double>());
    auto_declare<std::vector<double>>("alpha", std::vector<double>());
    auto_declare<double>("wheel_base_radius", 0.2);
    auto_declare<std::string>("base_frame_id", "base_link");
    auto_declare<std::string>("odom_frame_id", "odom");
    auto_declare<bool>("publish_odom", true);
    auto_declare<bool>("publish_tf", true);
    auto_declare<std::string>("cmd_vel_topic", "~/cmd_vel");
    auto_declare<std::string>("odom_topic", "~/odom");
    auto_declare<double>("cmd_vel_timeout", 0.5);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniWheelController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Read parameters
  wheel_joints_ = get_node()->get_parameter("wheel_joints").as_string_array();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  
  auto theta_B_deg = get_node()->get_parameter("theta_B").as_double_array();
  auto alpha_deg = get_node()->get_parameter("alpha").as_double_array();
  
  wheel_base_radius_ = get_node()->get_parameter("wheel_base_radius").as_double();
  base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();
  odom_frame_id_ = get_node()->get_parameter("odom_frame_id").as_string();
  publish_odom_ = get_node()->get_parameter("publish_odom").as_bool();
  publish_tf_ = get_node()->get_parameter("publish_tf").as_bool();
  cmd_vel_topic_ = get_node()->get_parameter("cmd_vel_topic").as_string();
  odom_topic_ = get_node()->get_parameter("odom_topic").as_string();
  cmd_vel_timeout_ = get_node()->get_parameter("cmd_vel_timeout").as_double();

  // Validate parameters
  if (wheel_joints_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "wheel_joints parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (theta_B_deg.size() != wheel_joints_.size() || alpha_deg.size() != wheel_joints_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "theta_B and alpha must have the same size as wheel_joints (%zu wheels)",
      wheel_joints_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Convert degrees to radians
  theta_B_.clear();
  alpha_.clear();
  for (size_t i = 0; i < wheel_joints_.size(); ++i)
  {
    theta_B_.push_back(theta_B_deg[i] * M_PI / 180.0);
    alpha_.push_back(alpha_deg[i] * M_PI / 180.0);
  }

  // Initialize previous wheel positions
  prev_wheel_positions_.resize(wheel_joints_.size(), 0.0);

  // Setup cmd_vel subscriber
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
      cmd_vel_callback(msg);
    });

  // Setup odometry publisher
  if (publish_odom_)
  {
    odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      get_node()->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SystemDefaultsQoS()));
  }

  // Setup TF broadcaster
  if (publish_tf_)
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());
  }

  RCLCPP_INFO(get_node()->get_logger(), "OmniWheelController configured successfully");
  RCLCPP_INFO(get_node()->get_logger(), "  wheel_joints: %zu", wheel_joints_.size());
  RCLCPP_INFO(get_node()->get_logger(), "  wheel_radius: %.3f m", wheel_radius_);
  RCLCPP_INFO(get_node()->get_logger(), "  wheel_base_radius: %.3f m", wheel_base_radius_);
  RCLCPP_INFO(get_node()->get_logger(), "  cmd_vel_topic: %s", cmd_vel_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "  odom_topic: %s", odom_topic_.c_str());
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
OmniWheelController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & wheel_joint : wheel_joints_)
  {
    config.names.push_back(wheel_joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  
  return config;
}

controller_interface::InterfaceConfiguration 
OmniWheelController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & wheel_joint : wheel_joints_)
  {
    config.names.push_back(wheel_joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  
  return config;
}

controller_interface::CallbackReturn OmniWheelController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Assign command interfaces
  wheel_velocity_command_interfaces_.clear();
  for (const auto & interface_name : command_interface_configuration().names)
  {
    auto it = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&interface_name](const auto & interface) { return interface.get_name() == interface_name; });
    
    if (it == command_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Command interface '%s' not found!", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    
    wheel_velocity_command_interfaces_.emplace_back(*it);
  }

  // Assign state interfaces
  wheel_position_state_interfaces_.clear();
  for (const auto & interface_name : state_interface_configuration().names)
  {
    auto it = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&interface_name](const auto & interface) { return interface.get_name() == interface_name; });
    
    if (it == state_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "State interface '%s' not found!", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    
    wheel_position_state_interfaces_.emplace_back(*it);
  }

  // Initialize odometry state
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  vx_ = 0.0;
  vy_ = 0.0;
  omega_z_ = 0.0;
  
  for (size_t i = 0; i < wheel_position_state_interfaces_.size(); ++i)
  {
    prev_wheel_positions_[i] = wheel_position_state_interfaces_[i].get().get_value();
  }
  
  prev_time_ = get_node()->now();
  last_cmd_vel_time_ = get_node()->now();

  RCLCPP_INFO(get_node()->get_logger(), "OmniWheelController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniWheelController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop all wheels
  for (auto & cmd_interface : wheel_velocity_command_interfaces_)
  {
    cmd_interface.get().set_value(0.0);
  }
  
  wheel_velocity_command_interfaces_.clear();
  wheel_position_state_interfaces_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "OmniWheelController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type OmniWheelController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  // Get latest cmd_vel
  auto cmd_vel = *cmd_vel_buffer_.readFromRT();
  
  double vx = 0.0, vy = 0.0, omega_z = 0.0;
  
  // Check if we have a valid command and it hasn't timed out
  if (cmd_vel && !is_cmd_vel_timeout(time))
  {
    vx = cmd_vel->linear.x;
    vy = cmd_vel->linear.y;
    omega_z = cmd_vel->angular.z;
  }
  else
  {
    // If timeout or no command, stop the robot
    vx = 0.0;
    vy = 0.0;
    omega_z = 0.0;
  }

  // Compute wheel velocities using the kinematic model
  auto wheel_velocities = compute_wheel_velocities(vx, vy, omega_z);

  // Send commands to wheels
  for (size_t i = 0; i < wheel_velocity_command_interfaces_.size(); ++i)
  {
    wheel_velocity_command_interfaces_[i].get().set_value(wheel_velocities[i]);
  }

  // Update and publish odometry
  update_odometry(time);
  
  if (publish_odom_)
  {
    publish_odometry(time);
  }
  
  if (publish_tf_)
  {
    publish_transform(time);
  }

  return controller_interface::return_type::OK;
}

void OmniWheelController::cmd_vel_callback(
  const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  cmd_vel_buffer_.writeFromNonRT(msg);
  last_cmd_vel_time_ = get_node()->now();
}

bool OmniWheelController::is_cmd_vel_timeout(const rclcpp::Time & current_time)
{
  double time_since_last_cmd = (current_time - last_cmd_vel_time_).seconds();
  return time_since_last_cmd > cmd_vel_timeout_;
}

std::vector<double> OmniWheelController::compute_wheel_velocities(
  double vx, double vy, double omega_z)
{
  std::vector<double> wheel_vels(wheel_joints_.size());
  
  // Apply the kinematic formula for each wheel:
  // omega_i = (1/a) * (u*cos(theta_Bi) + v*sin(theta_Bi) + r*l*sin(theta_Bi - alpha_i))
  // where: u = vx, v = vy, r = omega_z, a = wheel_radius, l = wheel_base_radius
  
  for (size_t i = 0; i < wheel_joints_.size(); ++i)
  {
    double cos_theta = std::cos(theta_B_[i]);
    double sin_theta = std::sin(theta_B_[i]);
    double sin_diff = std::sin(theta_B_[i] - alpha_[i]);
    
    wheel_vels[i] = (1.0 / wheel_radius_) * 
                    (vx * cos_theta + vy * sin_theta + omega_z * wheel_base_radius_ * sin_diff);
  }
  
  return wheel_vels;
}

void OmniWheelController::update_odometry(const rclcpp::Time & time)
{
  // Compute dt
  double dt = (time - prev_time_).seconds();
  if (dt <= 0.0)
  {
    return;
  }

  // Read current wheel positions and compute wheel angular velocities
  std::vector<double> wheel_ang_vels(wheel_joints_.size());
  
  for (size_t i = 0; i < wheel_position_state_interfaces_.size(); ++i)
  {
    double curr_pos = wheel_position_state_interfaces_[i].get().get_value();
    double delta_pos = curr_pos - prev_wheel_positions_[i];
    wheel_ang_vels[i] = delta_pos / dt;  // Angular velocity (rad/s)
    prev_wheel_positions_[i] = curr_pos;
  }

  // Inverse kinematics: compute robot velocities from wheel angular velocities
  // ω = J · V / a  =>  V = a · J⁺ · ω
  // where J is the Jacobian matrix and J⁺ is its pseudo-inverse
  
  // Build Jacobian matrix J (3x3 for 3 wheels)
  // J[i][0] = cos(θ_Bi)
  // J[i][1] = sin(θ_Bi)  
  // J[i][2] = l·sin(θ_Bi - α_i)
  
  double J[3][3];
  for (size_t i = 0; i < 3; ++i)
  {
    J[i][0] = std::cos(theta_B_[i]);
    J[i][1] = std::sin(theta_B_[i]);
    J[i][2] = wheel_base_radius_ * std::sin(theta_B_[i] - alpha_[i]);
  }
  
  // Compute J^T · J
  double JtJ[3][3] = {{0}};
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      for (int k = 0; k < 3; ++k)
      {
        JtJ[i][j] += J[k][i] * J[k][j];
      }
    }
  }
  
  // Compute inverse of (J^T · J) using 3x3 matrix inversion
  double det = JtJ[0][0] * (JtJ[1][1] * JtJ[2][2] - JtJ[1][2] * JtJ[2][1]) -
               JtJ[0][1] * (JtJ[1][0] * JtJ[2][2] - JtJ[1][2] * JtJ[2][0]) +
               JtJ[0][2] * (JtJ[1][0] * JtJ[2][1] - JtJ[1][1] * JtJ[2][0]);
  
  if (std::abs(det) < 1e-6)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Jacobian matrix is near-singular, skipping odometry update");
    return;
  }
  
  double inv_JtJ[3][3];
  inv_JtJ[0][0] = (JtJ[1][1] * JtJ[2][2] - JtJ[1][2] * JtJ[2][1]) / det;
  inv_JtJ[0][1] = (JtJ[0][2] * JtJ[2][1] - JtJ[0][1] * JtJ[2][2]) / det;
  inv_JtJ[0][2] = (JtJ[0][1] * JtJ[1][2] - JtJ[0][2] * JtJ[1][1]) / det;
  inv_JtJ[1][0] = (JtJ[1][2] * JtJ[2][0] - JtJ[1][0] * JtJ[2][2]) / det;
  inv_JtJ[1][1] = (JtJ[0][0] * JtJ[2][2] - JtJ[0][2] * JtJ[2][0]) / det;
  inv_JtJ[1][2] = (JtJ[0][2] * JtJ[1][0] - JtJ[0][0] * JtJ[1][2]) / det;
  inv_JtJ[2][0] = (JtJ[1][0] * JtJ[2][1] - JtJ[1][1] * JtJ[2][0]) / det;
  inv_JtJ[2][1] = (JtJ[0][1] * JtJ[2][0] - JtJ[0][0] * JtJ[2][1]) / det;
  inv_JtJ[2][2] = (JtJ[0][0] * JtJ[1][1] - JtJ[0][1] * JtJ[1][0]) / det;
  
  // Compute J⁺ = (J^T · J)^(-1) · J^T
  double J_pinv[3][3];
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      J_pinv[i][j] = 0.0;
      for (int k = 0; k < 3; ++k)
      {
        J_pinv[i][j] += inv_JtJ[i][k] * J[j][k];
      }
    }
  }
  
  // Compute robot velocities: V = a · J⁺ · ω
  vx_ = 0.0;
  vy_ = 0.0;
  omega_z_ = 0.0;
  for (int i = 0; i < 3; ++i)
  {
    vx_ += J_pinv[0][i] * wheel_ang_vels[i];
    vy_ += J_pinv[1][i] * wheel_ang_vels[i];
    omega_z_ += J_pinv[2][i] * wheel_ang_vels[i];
  }
  vx_ *= wheel_radius_;
  vy_ *= wheel_radius_;
  omega_z_ *= wheel_radius_;

  // Update robot pose in odometry frame
  double delta_x = (vx_ * std::cos(theta_) - vy_ * std::sin(theta_)) * dt;
  double delta_y = (vx_ * std::sin(theta_) + vy_ * std::cos(theta_)) * dt;
  double delta_theta = omega_z_ * dt;

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;

  // Normalize theta to [-pi, pi]
  while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
  while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

  prev_time_ = time;
}

void OmniWheelController::publish_odometry(const rclcpp::Time & time)
{
  if (odom_pub_ && odom_pub_->trylock())
  {
    auto & odom_msg = odom_pub_->msg_;
    
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    // Position
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation (quaternion from theta)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Velocity (in base_link frame)
    odom_msg.twist.twist.linear.x = vx_;
    odom_msg.twist.twist.linear.y = vy_;
    odom_msg.twist.twist.angular.z = omega_z_;

    odom_pub_->unlockAndPublish();
  }
}

void OmniWheelController::publish_transform(const rclcpp::Time & time)
{
  if (tf_broadcaster_)
  {
    geometry_msgs::msg::TransformStamped transform;
    
    transform.header.stamp = time;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;

    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform);
  }
}

}  // namespace omni_wheel_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  omni_wheel_controller::OmniWheelController,
  controller_interface::ControllerInterface)