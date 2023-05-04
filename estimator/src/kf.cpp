#include "../include/estimator/kf.hpp"

using namespace Eigen;
using namespace linearKF;

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

void KF::updatedt_(const double &dt){
  this->dt_.push_back(dt);
  if (this->dt_.size() > 10) {
    this->dt_.erase(this->dt_.begin());
  }
}


void KF::updateParams_(const double &dt){
    this->Q_ = this->get_Q_(dt);
  this->A_ = this->get_A_(dt);
  this->B_ = this->get_B_(dt);
}
void KF::updateParams_(){
    this->updateParams_(this->dt_.back());
}


void KF::step(const double &dt, const Matrix<double, 3, 1> &z) {
    this->updatedt_(dt);
    this->updateParams_();
  // Predict
  this->predict(dt);
  // Update
  this->update(z);
};

void KF::step(){
    this->updateParams_(0.0001);
    // Predict
    this->predict(0.0001);
    // Update
    this->x_ = this->x_hat_;
    this->P_ = this->P_hat_;
};

void KF::update(const Matrix<double, 3, 1> &z) {
  // Calculate Kalman gain
  this->K_ = this->P_hat_ * this->H_.transpose() *
             (this->H_ * this->P_hat_ * this->H_.transpose() + this->R_).inverse();
    // Update state estimate
    this->x_ = this->x_hat_ + this->K_ * (z - this->H_ * this->x_hat_);
    // Update covariance estimate
    this->P_ = (Matrix<double, 6, 6>::Identity() - this->K_ * this->H_) * this->P_hat_ *
               (Matrix<double, 6, 6>::Identity() - this->K_ * this->H_).transpose() +
               this->K_ * this->R_ * this->K_.transpose();
};

void KF::predict(const double &dt) {
    // Update state estimate
    this->x_hat_ = this->A_ * this->x_ + this->B_ * this->u_;
    // Update covariance estimate
    this->P_hat_ = this->A_ * this->P_ * this->A_.transpose() + this->Q_;
}