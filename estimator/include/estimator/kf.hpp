#ifndef KF_HPP
#define KF_HPP
// Import libraries
#include <eigen3/Eigen/Dense>
#include <opencv4/opencv2/video/tracking.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <vector>

// Define the namespace
namespace linearKF {
using namespace Eigen;

class KF {
private:
  // State vector
  Matrix<double, 6, 1> x_;
  // Predicted state vector
  Matrix<double, 6, 1> x_hat_;
  // Covariance matrix
  Matrix<double, 6, 6> P_;
  // Predicted covariance matrix
  Matrix<double, 6, 6> P_hat_;
  // Process noise covariance matrix
  Matrix<double, 6, 6> Q_;
  // Measurement noise covariance matrix
  Matrix<double, 3, 3> R_;
  // Kalman gain
  Matrix<double, 6, 3> K_;
  // Measurement matrix
  Matrix<double, 3, 6> H_;
  // Control input vector
  Matrix<double, 3, 1> u_;
  // State transition matrix
  Matrix<double, 6, 6> A_;
  // Control input matrix
  Matrix<double, 6, 3> B_;
  // Accumulated mesurement
  std::vector<Vector3d> history_;
  // Average timestep
  std::vector<double> dt_;


  /// @brief Update the average of the timestep
  /// @param dt
  void updatedt_(const double &dt);

  /// @brief Update the state transition matrix and control input matrix
  /// @param dt
  void updateParams_(const double &dt);
  void updateParams_();

public:
  static Matrix<double, 6, 6> get_Q_(double dt) {
    Matrix<double, 6, 6> Q;
    RowVector4d diff{0.25 * pow(dt, 4.0), 0, 0, 0.5 * pow(dt, 3.0)};
    Q.row(0) << diff, 0, 0;
    Q.row(1) << 0, diff, 0;
    Q.row(2) << 0, 0, diff;
    diff = RowVector4d{0.5 * pow(dt, 3.0), 0, 0, pow(dt, 2.0)};
    Q.row(3) << diff, 0, 0;
    Q.row(4) << 0, diff, 0;
    Q.row(5) << 0, 0, diff;
    return Q;
  };
  static Matrix<double, 6, 6> get_A_(const double &dt) {
    return (MatrixXd(6, 6) << 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0,
            0, dt, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1)
        .finished();
  }
  static Matrix<double, 6, 3> get_B_(const double &dt) {
    return (MatrixXd(6, 3) << 0.5 * pow(dt, 2.0), 0, 0, 0, 0.5 * pow(dt, 2.0),
            0, 0, 0, 0.5 * pow(dt, 2.0), dt, 0, 0, 0, dt, 0, 0, 0, dt)
        .finished();
  }
  /// @brief The main constructor
  /// @param P
  /// @param R
  /// @param H
  /// @param gravity
  KF(const MatrixXd &P, const MatrixXd &R, const MatrixXd &H,
     const double &gravity = -9.81);

  /// @brief The constructor with gravity
  /// @param gravity
  KF(const double &gravity);
  KF();
  // Destructor
  ~KF() = default;

  /// @brief The main step function run at every control step to predict the
  /// states and update the covariance matrix
  /// @param z
  void step(const double &dt, const Matrix<double, 3, 1> &z);
  void step(const double &dt);

  /// @brief Update the state vector and covariance matrix given the measurement
  /// @param dt
  void update(const Matrix<double, 3, 1> &z);

  /// @brief Update the state vector and covariance matrix given the measurement
  void predict(const double &dt);
  void predict();

  /// @brief Get the state vector and covariance matrix
  /// @param x
  /// @param P
  void getStates(Matrix<double, 6, 1> &x, Matrix<double, 6, 6> &P) const {
    x = this->x_;
    P = this->P_;
  };
};

} // namespace linearKF

namespace cvKF {
class KF {
  cv::KalmanFilter kf_;
  cv::Mat_<double> u_;
  std::vector<Eigen::Vector3d> history_;
  std::vector<double> dt_;
public:
  KF();
  ~KF();
  void step(const double &dt, const Eigen::Matrix<double, 3, 1> &z);
  void step(const double &dt);
  void predict(const double &dt);
  void updatedt_(const double &dt);
  void getStates(Eigen::Matrix<double, 6, 1> &x,
                 Eigen::Matrix<double, 6, 6> &P) const;
};
} // namespace cvKF

#endif // KF_HPP
