#include "omni_wheel_controller/kinematics.hpp"
#include <cmath>

namespace omni_wheel_controller
{

std::array<double,3> robot_vel_to_wheel_rates(const std::array<double,3> &robot_vel,
                                              const std::vector<double> &theta,
                                              double wheel_radius, double L)
{
  double vx = robot_vel[0];
  double vy = robot_vel[1];
  double omega = robot_vel[2];

  std::array<double,3> out{};
  double inv_r = 1.0 / wheel_radius;
  for (int i = 0; i < 3; ++i) {
    double proj = vx * std::cos(theta[i]) + vy * std::sin(theta[i]);
    double rot = L * omega;
    out[i] = inv_r * (proj + rot);
  }
  return out;
}

std::array<double,3> wheel_rates_to_robot_vel(const std::array<double,3> &v_w,
                                              const std::vector<double> &theta,
                                              double L)
{
  // Build A and solve A * x = b using closed-form 3x3 solve (Cramer's rule)
  double A00 = std::cos(theta[0]), A01 = std::sin(theta[0]), A02 = L;
  double A10 = std::cos(theta[1]), A11 = std::sin(theta[1]), A12 = L;
  double A20 = std::cos(theta[2]), A21 = std::sin(theta[2]), A22 = L;

  double det =
    A00*(A11*A22 - A12*A21) -
    A01*(A10*A22 - A12*A20) +
    A02*(A10*A21 - A11*A20);

  std::array<double,3> sol{0.0, 0.0, 0.0};
  if (std::abs(det) < 1e-12) return sol;

  auto det_replace = [&](int col)->double {
    double M00=A00, M01=A01, M02=A02;
    double M10=A10, M11=A11, M12=A12;
    double M20=A20, M21=A21, M22=A22;
    if (col==0) { M00 = v_w[0]; M10 = v_w[1]; M20 = v_w[2]; }
    else if (col==1) { M01 = v_w[0]; M11 = v_w[1]; M21 = v_w[2]; }
    else { M02 = v_w[0]; M12 = v_w[1]; M22 = v_w[2]; }
    double d = M00*(M11*M22 - M12*M21) - M01*(M10*M22 - M12*M20) + M02*(M10*M21 - M11*M20);
    return d;
  };

  sol[0] = det_replace(0) / det;
  sol[1] = det_replace(1) / det;
  sol[2] = det_replace(2) / det;
  return sol;
}

} // namespace
