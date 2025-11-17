#ifndef OMNI_WHEEL_CONTROLLER_OMNI_CONTROLLER_HPP_
#define OMNI_WHEEL_CONTROLLER_OMNI_CONTROLLER_HPP_

#include <vector>
#include <string>
#include <atomic>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

// include our helper types
#include "omni_wheel_controller/odom_integrator.hpp"

namespace omni_wheel_controller
{

class OmniController : public controller_interface::ControllerInterface
{
public:
  OmniController();

  // lifecycle hooks
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  // declare interfaces required
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

  // params
  std::vector<std::string> wheels_;
  double wheel_radius_;
  double center_to_wheel_;
  double max_wheel_rad_s_;
  std::string cmd_topic_;
  int cmd_timeout_ms_;
  int N_;
  double base_rotation_deg_;
  double wheel_spacing_deg_;

  // kinematics angles
  std::vector<double> theta_;

  // runtime cmd storage
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::atomic<double> last_vx_{0.0}, last_vy_{0.0}, last_omega_{0.0};
  rclcpp::Time last_cmd_time_;

  // odom + tf
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // odom integrator (type defined in odom_integrator.hpp)
  OdomIntegrator odom_;

  // odom params
  std::string odom_frame_;
  std::string base_frame_;
  double cov_x_, cov_y_, cov_yaw_;
};

} // namespace

#endif // OMNI_WHEEL_CONTROLLER_OMNI_CONTROLLER_HPP_
