#include "omni_wheel_controller/odom_integrator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

namespace omni_wheel_controller
{

OdomIntegrator::OdomIntegrator() : x_(0.0), y_(0.0), yaw_(0.0) {}

void OdomIntegrator::reset_pose()
{
  std::lock_guard<std::mutex> lk(mutex_);
  x_ = 0.0; y_ = 0.0; yaw_ = 0.0;
}

void OdomIntegrator::integrate_body(double vx, double vy, double omega, double dt)
{
  if (dt <= 0.0) return;
  std::lock_guard<std::mutex> lk(mutex_);
  double c = std::cos(yaw_), s = std::sin(yaw_);
  double dx = (vx * c - vy * s) * dt;
  double dy = (vx * s + vy * c) * dt;
  double dyaw = omega * dt;
  x_ += dx; y_ += dy; yaw_ += dyaw;
}

nav_msgs::msg::Odometry OdomIntegrator::make_odom_msg(const rclcpp::Time &stamp,
                                                     const std::string &odom_frame,
                                                     const std::string &base_frame,
                                                     double cov_x, double cov_y, double cov_yaw)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = base_frame;

  {
    std::lock_guard<std::mutex> lk(mutex_);
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q; q.setRPY(0,0,yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
  }

  // zero all then set the main three cov entries
  for (size_t i=0;i<odom.pose.covariance.size();++i) odom.pose.covariance[i]=0.0;
  odom.pose.covariance[0] = cov_x;
  odom.pose.covariance[7] = cov_y;
  odom.pose.covariance[35] = cov_yaw;

  // twist left zero; controller could fill later
  return odom;
}

geometry_msgs::msg::TransformStamped OdomIntegrator::make_tf(const rclcpp::Time &stamp,
                                                             const std::string &odom_frame,
                                                             const std::string &base_frame)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = odom_frame;
  t.child_frame_id = base_frame;

  double px, py, pyaw;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    px = x_; py = y_; pyaw = yaw_;
  }

  t.transform.translation.x = px;
  t.transform.translation.y = py;
  t.transform.translation.z = 0.0;
  tf2::Quaternion q; q.setRPY(0,0,pyaw);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  return t;
}

} // namespace
