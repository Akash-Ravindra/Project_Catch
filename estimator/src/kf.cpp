#include "../include/estimator/kf.hpp"
using namespace Eigen;

const Matrix<double, 6, 6> defaultP =
    (MatrixXd(6, 6) << 0.01, 0, 0, 0.001, 0, 0, 0, 0.01, 0, 0, 0.001, 0, 0, 0,
     0.01, 0, 0, 0.001, 0.001, 0, 0, 500, 0, 0, 0, 0.001, 0, 0, 500, 0, 0, 0,
     0.001, 0, 0, 500)
        .finished();

const Matrix3d defaultR = MatrixXd::Identity(3, 3) * 0.005;

const Matrix<double, 3, 6> defaultH =
    (MatrixXd(3, 6) << MatrixXd::Identity(3, 3), MatrixXd::Zero(3, 3))
        .finished();

linearKF::KF::KF(const MatrixXd &P, const MatrixXd &R, const MatrixXd &H,
                 const double &gravity)
    : P_{P}, R_{R}, H_{H} {
  this->x_ = Matrix<double, 6, 1>::Zero();
  this->x_hat_ = Matrix<double, 6, 1>::Zero();
  this->P_hat_ = Matrix<double, 6, 6>::Zero();
  this->u_ = Matrix<double, 3, 1>::Zero();
  this->u_.row(2) << gravity;
};

// The default constructor
linearKF::KF::KF() : KF{defaultP, defaultR, defaultH} {};

linearKF::KF::KF(const double &gravity)
    : KF{defaultP, defaultR, defaultH, gravity} {};

void linearKF::KF::updatedt_(const double &dt) {
  this->dt_.push_back(dt);
  if (this->dt_.size() > 10) {
    this->dt_.erase(this->dt_.begin());
  }
}

void linearKF::KF::updateParams_(const double &dt) {
  this->Q_ = this->get_Q_(dt);
  this->A_ = this->get_A_(dt);
  this->B_ = this->get_B_(dt);
}
void linearKF::KF::updateParams_() { this->updateParams_(this->dt_.back()); }

void linearKF::KF::step(const double &dt, const Matrix<double, 3, 1> &z) {
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

void linearKF::KF::step(const double &dt) {
  this->updateParams_(dt);
  // Predict
  this->predict(dt);
  // Update
  this->x_ = this->x_hat_;
  this->P_ = this->P_hat_;
};

void linearKF::KF::update(const Matrix<double, 3, 1> &z) {
  // Calculate Kalman gain
  this->K_ =
      this->P_hat_ * this->H_.transpose() *
      (this->H_ * this->P_hat_ * this->H_.transpose() + this->R_).inverse();
  // Update state estimate
  this->x_ = this->x_hat_ + this->K_ * (z - this->H_ * this->x_hat_);
  // Update covariance estimate
  this->P_ =
      (Matrix<double, 6, 6>::Identity() - this->K_ * this->H_) * this->P_hat_ *
          (Matrix<double, 6, 6>::Identity() - this->K_ * this->H_).transpose() +
      this->K_ * this->R_ * this->K_.transpose();
};

void linearKF::KF::predict(const double &dt) {
  // Update state estimate
  this->x_hat_ = this->A_ * this->x_ + this->B_ * this->u_;
  // Update covariance estimate
  this->P_hat_ = this->A_ * this->P_ * this->A_.transpose() + this->Q_;
}

cvKF::KF::KF() {
  this->kf_ = new cv::KalmanFilter(6, 3, 0);
  this->kf_->measurementNoiseCov = cv::Mat::eye(3, 3, CV_64F) * 0.005;
  this->kf_->errorCovPre =
      (cv::Mat_<double>(6, 6) << 0.01, 0, 0, 0.001, 0, 0, 0, 0.01, 0, 0, 0.001,
       0, 0, 0, 0.01, 0, 0, 0.001, 0.001, 0, 0, 500, 0, 0, 0, 0.001, 0, 0, 500,
       0, 0, 0, 0.001, 0, 0, 500);
  this->kf_->measurementMatrix = (cv::Mat_<double>(3, 6) << 1, 0, 0, 0, 0, 0, 0,
                                  1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0);
  this->u_ = cv::Mat_<double>(3, 1) << 0, 0, -9.81;
}
cvKF::KF::~KF() { delete this->kf_; }

void cvKF::KF::updatedt_(const double &dt) {
  this->dt_.push_back(dt);
  if (this->dt_.size() > 10) {
    this->dt_.erase(this->dt_.begin());
  }
  cv::eigen2cv(linearKF::KF::get_A_(dt), this->kf_->transitionMatrix);
  cv::eigen2cv(linearKF::KF::get_B_(dt), this->kf_->controlMatrix);
  cv::eigen2cv(linearKF::KF::get_Q_(dt), this->kf_->processNoiseCov);
}

void cvKF::KF::step(const double &dt, const Eigen::Matrix<double, 3, 1> &z) {
  if (this->history_.size() < 2) {
    // Initialize the history and use as seed for states
    this->history_.push_back(z);
    if (this->history_.size() == 2) {
      this->kf_->statePre =
          (cv::Mat_<double>(6, 1) << this->history_[1](0), this->history_[1](1),
           this->history_[1](2),
           (this->history_[1](0) - this->history_[0](0)) / dt,
           (this->history_[1](1) - this->history_[0](1)) / dt,
           (this->history_[1](2) - this->history_[0](2)) / dt);
    }
    return;
  }
  this->updatedt_(dt);
  // Predict
  this->kf_->predict();
  // Update
  this->kf_->correct((cv::Mat_<double>(3, 1) << z(0), z(1), z(2)));
}
void cvKF::KF::step(const double &dt){
  
  cv::eigen2cv(linearKF::KF::get_A_(dt), this->kf_->transitionMatrix);
  cv::eigen2cv(linearKF::KF::get_B_(dt), this->kf_->controlMatrix);
  cv::eigen2cv(linearKF::KF::get_Q_(dt), this->kf_->processNoiseCov);
  // Predict
  this->kf_->predict();
  this->kf_->statePre.copyTo(this->kf_->statePost);
  this->kf_->errorCovPre.copyTo(this->kf_->errorCovPost);
}

void cvKF::KF::getStates(Eigen::Matrix<double, 6, 1> &x,
                 Eigen::Matrix<double, 6, 6> &P)const{
  cv::Mat_<double> x_ = this->kf_->statePost;
  cv::Mat_<double> P_ = this->kf_->errorCovPost;
  x << x_(0), x_(1), x_(2), x_(3), x_(4), x_(5);
  P << P_(0,0), P_(0,1), P_(0,2), P_(0,3), P_(0,4), P_(0,5),
        P_(1,0), P_(1,1), P_(1,2), P_(1,3), P_(1,4), P_(1,5),
        P_(2,0), P_(2,1), P_(2,2), P_(2,3), P_(2,4), P_(2,5),
        P_(3,0), P_(3,1), P_(3,2), P_(3,3), P_(3,4), P_(3,5),
        P_(4,0), P_(4,1), P_(4,2), P_(4,3), P_(4,4), P_(4,5),
        P_(5,0), P_(5,1), P_(5,2), P_(5,3), P_(5,4), P_(5,5);
                 }