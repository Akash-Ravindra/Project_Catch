#include "../include/estimator/kf.hpp"
using namespace Eigen;
using namespace linearKF;

const Matrix<double, 6, 6> defaultP =
    (MatrixXd(6, 6) << 0.01, 0, 0, 0.001, 0, 0, 0, 0.01, 0, 0, 0.001, 0, 0, 0,
     0.01, 0, 0, 0.001, 0.001, 0, 0, 500, 0, 0, 0, 0.001, 0, 0, 500, 0, 0, 0,
     0.001, 0, 0, 500)
        .finished();

const Matrix3d defaultR = MatrixXd::Identity(3, 3) * 0.005;

const Matrix<double, 3, 6> defaultH =
    (MatrixXd(3, 6) << MatrixXd::Identity(3, 3), MatrixXd::Zero(3, 3))
        .finished();

KF::KF(const MatrixXd &P, const MatrixXd &R, const MatrixXd &H,
       const double &gravity)
    : P_{P}, R_{R}, H_{H} {
  this->x_ = Matrix<double, 6, 1>::Zero();
  this->x_hat_ = Matrix<double, 6, 1>::Zero();
  this->P_hat_ = Matrix<double, 6, 6>::Zero();
  this->u_ = Matrix<double, 3, 1>::Zero();
  this->u_.row(2) << gravity;
};

// The default constructor
KF::KF() : KF{defaultP, defaultR, defaultH} {};

KF::KF(const double &gravity) : KF{defaultP, defaultR, defaultH, gravity} {};

void KF::updatedt_(const double &dt) {
  this->dt_.push_back(dt);
  if (this->dt_.size() > 10) {
    this->dt_.erase(this->dt_.begin());
  }
}

void KF::updateParams_(const double &dt) {
  this->Q_ = this->get_Q_(dt);
  this->A_ = this->get_A_(dt);
  this->B_ = this->get_B_(dt);
}
void KF::updateParams_() { this->updateParams_(this->dt_.back()); }

void KF::step(const double &dt, const Matrix<double, 3, 1> &z) {
  if (this->history_.size() < 2) {
    // Initialize the history and use as seed for states
    this->history_.push_back(z);
    if (this->history_.size() == 2) {
      this->x_ << this->history_[1],
          (this->history_[1] - this->history_[0]) / dt;
    }
    return;
  }
  this->updatedt_(dt);
  this->updateParams_();
  // Predict
  this->predict(dt);
  // Update
  this->update(z);
};

void KF::step(const double &dt) {
  this->updateParams_(dt);
  // Predict
  this->predict(dt);
  // Update
  this->x_ = this->x_hat_;
  this->P_ = this->P_hat_;
};

void KF::update(const Matrix<double, 3, 1> &z) {
  // Calculate Kalman gain
  this->K_.noalias() =
      (this->P_hat_ * this->H_.transpose()) *
      ((this->H_ * this->P_hat_ * this->H_.transpose())+ this->R_).inverse();
  // Update state estimate
  this->x_.noalias() = this->x_hat_ + (this->K_ * (z - this->H_ * this->x_hat_));
  // Update covariance estimate
  this->P_.noalias() =
      ((this->I_ - (this->K_ * this->H_)) * this->P_hat_ *
          (this->I_ - (this->K_ * this->H_)).transpose()) +
      (this->K_ * this->R_ * this->K_.transpose());
};

void KF::predict(const double &dt) {
  // Update state estimate
  this->x_hat_.noalias() = (this->A_ * this->x_) + (this->B_ * this->u_);
  // Update covariance estimate
  this->P_hat_.noalias() = (this->A_ * this->P_ * this->A_.transpose()) + this->Q_;
}