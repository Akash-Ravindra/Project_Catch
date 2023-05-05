#include "../include/estimator/estimator.hpp"

using namespace et;
std::string defaultViconTopic = "/vicon/markers";

Estimator::Estimator(std::string name)
    : action_server_(nh_, name, false), name_(name) {
  this->action_server_.registerGoalCallback(
      boost::bind(&Estimator::goalCallback, this));
  this->action_server_.start();
};

Estimator::~Estimator(){

};

void Estimator::preemptCallback() {
  ROS_INFO("%s: Preempted", this->name_.c_str());
  this->action_server_.setPreempted();
}

void Estimator::goalCallback() {
  auto goal = this->action_server_.acceptNewGoal();
  if (goal.get()->startTracker) {
    // Clear the feedhback
    ROS_INFO("%s: Received the goal from client to track '%s' on '%s'",
             this->name_.c_str(), goal.get()->objectName.c_str(),
             goal.get()->viconTopic.empty() ? defaultViconTopic.c_str()
                                            : goal.get()->viconTopic.c_str());
    this->clearFeedback();
    // subscribe to the vicon topic
    this->vicon_sub_ =
        nh_.subscribe(goal.get()->viconTopic.empty() ? defaultViconTopic
                                                     : goal.get()->viconTopic,
                      1000, &Estimator::markersCallback, this);
    // create a action timer to close the
    this->action_timeout_ = nh_.createTimer(
        ros::Duration(600), &Estimator::actiontimerCallback, this, true, false);
    this->action_timeout_.start();
    // Get the name of the object that we want to track
    objectName_ = goal.get()->objectName;
  } else {
    ROS_INFO_NAMED(this->name_, "Received goal but start tracker not set");
  }
}

void Estimator::actiontimerCallback(const ros::TimerEvent &__event) {}
void Estimator::flighttimerCallback(const ros::TimerEvent &event) {
  // Reset the kalman filter and history so that
  ROS_INFO_NAMED(this->name_,
                 "Flight timer expired, reseting the filter and the pub hist");
  // Reset the filter
  this->filter_ = linearKF::KF();
  // Reset the history
  this->msg_hist_.clear();
}
void Estimator::markersCallback(const vicon_bridge::MarkersConstPtr &markers) {
  if (!ros::ok() || this->action_server_.isPreemptRequested() ||
      !this->action_timeout_.hasStarted() || !this->action_server_.isActive()) {
    // handle exit from the publish node and close the action server request
    // nicely
    ROS_WARN_NAMED(this->name_,
                   "The publisher was closed from preempt, timer or ctrl+C");
    this->action_server_.setPreempted();
    this->clearFeedback();
    this->vicon_sub_.shutdown();
    return;
  }
  // Check if there are any markers detected
  if (markers.get()->markers.size()) {
    auto mkrs = markers.get()->markers;
    auto name_to_find = objectName_;
    // Find the marker with the name that we want to track
    auto it = std::find_if(mkrs.begin(), mkrs.end(),
                           [name_to_find](const vicon_bridge::Marker &m) {
                             return m.marker_name == name_to_find;
                           });
    // If we found the marker
    if (it != mkrs.end()) {
      auto cur = *it;
      this->clearFeedback();
      // Convert the units to meters
      cur.translation.x = cur.translation.x / 1000.0;
      cur.translation.y = cur.translation.y / 1000.0;
      cur.translation.z = cur.translation.z / 1000.0;

      /////////////////////////////
      //// Bound box test here ////
      /////////////////////////////

      // if it passes the bounding box test
      this->feedback_.isValid = true;
      this->feedback_.objectName = this->objectName_;
      if (this->msg_hist_.empty()) {
        // This would be the first time its seeing the ball
        this->flight_timeout_ =
            nh_.createTimer(ros::Duration(5), &Estimator::flighttimerCallback,
                            this, true, false);
        ROS_INFO_NAMED(this->name_,
                       "Found the marker and starting flight timer");
      }

      /////////////////////////////
      /////// Kalman filter ///////
      /////////////////////////////

      // Calculate the time difference between the current and the last marker
      double dt =
          markers.get()->header.stamp.toSec() -
          (this->msg_hist_.empty() ? 0.0 : (this->msg_hist_.back().second));
      // save the msg and the timestamp
      this->msg_hist_.push_back(std::pair<geometry_msgs::Point, double>(
          cur.translation, markers.get()->header.stamp.toSec()));
      // Step the kalman filter
      this->filter_.step(dt, (Eigen::VectorXd(3) << cur.translation.x,
                              cur.translation.y, cur.translation.z)
                                 .finished());
      // Get the filtered states
      Eigen::Matrix<double, 6, 1> state;
      Eigen::Matrix<double, 6, 6> cov;
      this->filter_.getStates(state, cov);
      // Store the filtered states
      geometry_msgs::Point pred;
      eigenToPoint(state, &pred);
      this->filtered_hist_.push_back(pred);

      this->feedback_.currentPosition = pred;

      // After some time, we can start predicting the path of the ball and
      if (this->msg_hist_.size() > 20) {
        std::vector<geometry_msgs::Point> prediction;
        this->simulateFlight_(&prediction);
        // publish feedback
        this->feedback_.predictedTrajectory = prediction;
        this->feedback_.interceptTime = prediction.size() * 0.001;
      }

      // Publish the feedback
      this->feedback_.deltaAltitude = this->filtered_hist_.size()>1?(this->filtered_hist_.back().z - (this->filtered_hist_.at(this->filtered_hist_.size()-2)).z)>0?1.0:-1.0:0.0;
      this->action_server_.publishFeedback(this->feedback_);
      // If the ball is on the ground
      if (this->filtered_hist_.back().z < 0.5) {
        // Stop the flight timer
        this->flight_timeout_.stop();
        // Stop the vicon subscriber
        this->vicon_sub_.shutdown();
        // Stop the action server
        this->action_server_.setSucceeded();
        ROS_INFO_NAMED(this->name_, "Ball has landed, shutting down");
      }
    }
  }
}

void Estimator::simulateFlight_(std::vector<geometry_msgs::Point> *prediction) {
  // Simulate the flight of the ball
  auto cpy_filter = this->filter_;
  auto cur = this->msg_hist_.back().first;
  prediction->push_back(cur);
  while (prediction->back().z > 0.5 || prediction->size() < 1.5 / 0.001) {
    cpy_filter.step();
    Eigen::Matrix<double, 6, 1> state;
    Eigen::Matrix<double, 6, 6> cov;
    cpy_filter.getStates(state, cov);
    geometry_msgs::Point pred;
    eigenToPoint(state, &pred);
    prediction->push_back(pred);
  }
}