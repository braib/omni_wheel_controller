#ifndef OMNI_WHEEL_CONTROLLER_ODOM_INTEGRATOR_HPP_
#define OMNI_WHEEL_CONTROLLER_ODOM_INTEGRATOR_HPP_

#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace omni_wheel_controller
{

class OdomIntegrator
{
public:
  OdomIntegrator();
  void reset_pose();
  void integrate_body(double vx, double vy, double omega, double dt);
  nav_msgs::msg::Odometry make_odom_msg(const rclcpp::Time &stamp,
                                        const std::string &odom_frame,
                                        const std::string &base_frame,
                                        double cov_x, double cov_y, double cov_yaw);
  geometry_msgs::msg::TransformStamped make_tf(const rclcpp::Time &stamp,
                                               const std::string &odom_frame,
                                               const std::string &base_frame);

private:
  std::mutex mutex_;
  double x_; double y_; double yaw_;
};

} // namespace

#endif // OMNI_WHEEL_CONTROLLER_ODOM_INTEGRATOR_HPP_
