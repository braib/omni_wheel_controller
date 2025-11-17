#ifndef OMNI_WHEEL_CONTROLLER_KINEMATICS_HPP_
#define OMNI_WHEEL_CONTROLLER_KINEMATICS_HPP_

#include <array>
#include <vector>

/*
 * Small kinematics helpers for a 3-wheel omni robot.
 * theta: wheel steering directions (radians), same order as joints.
 */

namespace omni_wheel_controller
{

// robot_vel = [vx, vy, omega] -> returns wheel angular speeds (rad/s)
std::array<double,3> robot_vel_to_wheel_rates(const std::array<double,3> &robot_vel,
                                              const std::vector<double> &theta,
                                              double wheel_radius, double L);

// v_w: wheel linear velocities along driving directions (m/s) -> [vx, vy, omega]
std::array<double,3> wheel_rates_to_robot_vel(const std::array<double,3> &v_w,
                                              const std::vector<double> &theta,
                                              double L);

} // namespace

#endif // OMNI_WHEEL_CONTROLLER_KINEMATICS_HPP_
