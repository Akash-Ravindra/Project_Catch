#include <iostream>

#include "../include/estimator/kf.hpp"

/// @brief Generate a parabola given inital velocity and angle
/// @return std::vector<Eigen::Vector3d> of points of the parabola
void generateParabola(std::vector<Eigen::Vector3d> &parabola, const double &v0,
                      const double &theta, const double &g, const double &dt,
                      const double &t_max) {
  parabola.clear();
  for (double t = 0.0; t < t_max; t += dt) {
    Eigen::Vector3d point;
    point << v0 * cos(theta) * t, 0.0, v0 * sin(theta) * t - 0.5 * g * t * t;
    parabola.push_back(point);
    // If the point is below the ground, stop
    if (point(2) < 0.0) {
      break;
    }
  }
}

int main() {
  // Create a parabola
  std::vector<Eigen::Vector3d> parabola;
  double v0 = 10.0;
  double theta = M_PI / 4.0;
  double g = 9.81;
  double dt = 0.01;
  double t_max = 2.0;
  generateParabola(parabola, v0, theta, g, dt, t_max);

  // Create a kalman filter
  linearKF::KF kf = {};
  // Step through the parabola
  auto counter = 0;
  for (auto &point : parabola) {
    kf.step(dt, point);
    Eigen::Matrix<double, 6, 1> state;
    Eigen::Matrix<double, 6, 6> cov;
    kf.getStates(state, cov);
    std::cout << "State: " << state.transpose() << "|" << point.transpose() << std::endl;
    counter++;
    if (counter > 100) {
    std::cout << "P" << std::endl << cov << std::endl;
      break;
    }
  }
}