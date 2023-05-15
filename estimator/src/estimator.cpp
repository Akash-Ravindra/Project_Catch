#include "../include/estimator/estimator.hpp"

using namespace et;

const std::string defaultViconTopic = "/vicon/markers";
const double defaultPredictionTimestep = 0.001;
const double defaultFlightTimeout = 10.0;

// Bounding box for the net area
const double netXMin = -2000;
const double netXMax = 2000;// NEED TO CHANGE BACK TO 2000
const double netYMin = -3000;
const double netYMax = 2500;
const double netZMin = 350.0;
const double netZMax = 4000.0;

const double startPredictionTime = 1.0;
const double startPredictionAltitude = 3000.0;

Estimator::Estimator(std::string name)
    : action_server_(nh_, name, false), name_(name) {
  this->action_server_.registerGoalCallback(
      boost::bind(&Estimator::goalCallback, this));
  this->action_server_.start();
  ROS_INFO_NAMED(this->name_, "Started the estimator action server");
  // Get the parameters
  this->startPredictionAltitude_ = nh_.param("start_prediction_altitude", startPredictionAltitude);
  this->startPredictionTime_ = nh_.param("start_prediction_time", startPredictionTime);
  // Print the parameters
  ROS_INFO_NAMED(this->name_, "start_prediction_altitude: %f", this->startPredictionAltitude_);
  ROS_INFO_NAMED(this->name_, "start_prediction_time: %f", this->startPredictionTime_);
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
    this->resetAccumulators_();
    ROS_INFO_NAMED(this->name_, "Started tracking '%s'", objectName_.c_str());
    // subscribe to the vicon topic
    this->vicon_sub_ =
        nh_.subscribe(goal.get()->viconTopic.empty() ? defaultViconTopic
                                                     : goal.get()->viconTopic,
                      1000, &Estimator::markersCallback, this);
    // create a action timer to close the
    this->action_timer_ =
        nh_.createTimer(ros::Duration(60.0, 0.0),
                        &Estimator::actiontimerCallback, this, true, false);
    this->action_timer_.start();
    // Get the name of the object that we want to track
    objectName_ = goal.get()->objectName;
  } else {
    ROS_INFO_NAMED(this->name_, "Received goal but start tracker not set");
  }
}

void Estimator::actiontimerCallback(const ros::TimerEvent &__event) {
  if(!this->action_server_.isActive())
    return;
  // Fail the action call and return failed result
  ROS_ERROR_NAMED(this->name_, "Action timer expired, closing the action server");
  this->action_server_.setAborted();
  this->resetAccumulators_();
  this->clearFeedback();
  this->vicon_sub_.shutdown();
  this->flight_timer_.stop();
  this->action_timer_.stop();
}
void Estimator::flighttimerCallback(const ros::TimerEvent &event) {
  // Reset the kalman filter and history so that
  ROS_WARN_NAMED(this->name_,
                 "Flight timer expired, reseting the filter and the pub hist");
  this->resetAccumulators_();
}
void Estimator::markersCallback(const vicon_bridge::MarkersConstPtr &markers) {
  if (!ros::ok() || this->action_server_.isPreemptRequested() ||
      !this->action_timer_.hasStarted() || !this->action_server_.isActive()) {
    // handle exit from the publish node and close the action server request
    // nicely
    ROS_WARN_NAMED(this->name_,
                   "The publisher was closed from preempt, timer or ctrl+C");
    this->action_server_.setAborted();
    this->resetAccumulators_();
    this->clearFeedback();
    this->vicon_sub_.shutdown();
    return;
  }
  // Check if there are any markers detected
  if (markers.get()->markers.size()) {
    auto mkrs = markers.get()->markers;
    auto name_to_find = objectName_;
    // Find the marker with the name that we want to track
                  /////////////////////////////
                  //// Bound box test here ////
                  /////////////////////////////
    auto it = std::find_if(
        mkrs.begin(), mkrs.end(),
        [name_to_find](const vicon_bridge::Marker &m) {
          return (m.marker_name == name_to_find) &&
                 (m.translation.x > netXMin && m.translation.x < netXMax) &&
                 (m.translation.y > netYMin && m.translation.y < netYMax) &&
                 (m.translation.z > netZMin && m.translation.z < netZMax);
        });
    // If we found the marker
    if (it != mkrs.end()) {
      auto cur = *it;
      this->clearFeedback();
      // Convert the units to meters
      cur.translation.x = cur.translation.x / 1000.0;
      cur.translation.y = cur.translation.y / 1000.0;
      cur.translation.z = cur.translation.z / 1000.0;
      // Add the current marker to the history
      this->feedback_.isValid = true;
      this->feedback_.objectName = this->objectName_;
      if (this->msg_hist_.empty()) {
        // This would be the first time its seeing the ball
        this->flight_timer_ = nh_.createTimer(
            ros::Duration(defaultFlightTimeout, 0),
            &Estimator::flighttimerCallback, this, true, false);
        this->flight_timer_.start();
        ROS_INFO_NAMED(this->name_,
                       "Found the marker and starting flight timer");
      }

      /////////////////////////////
      /////// Kalman filter ///////
      /////////////////////////////

      // Calculate the time difference between the current and the last marker
      double dt =
          (this->msg_hist_.empty() ? 0.0
                                   : markers.get()->header.stamp.toSec() -
                                         (this->msg_hist_.back().second));
      
      // Elapsed time
      this->flight_time_ += dt;
      // save the msg and the timestamp
      this->msg_hist_.push_back(std::pair<geometry_msgs::Point, double>(
          cur.translation, markers.get()->header.stamp.toSec()));
      // Step the kalman filter
      ROS_INFO_NAMED(this->name_,
                     "Found the marker '%s' at (%f, %f, %f) Stepping the "
                     "kalman filter with dt: %f",
                     cur.marker_name.c_str(), cur.translation.x,
                     cur.translation.y, cur.translation.z, dt);
      this->filter_.step(dt, (Eigen::VectorXd(3) << cur.translation.x,
                              cur.translation.y, cur.translation.z)
                                 .finished());
      // Get the filtered states
      Eigen::Matrix<double, 6, 1> state;
      Eigen::Matrix<double, 6, 6> cov;
      this->filter_.getStates(state, cov);
      ROS_INFO_NAMED(this->name_, "Filtered state: (%f, %f, %f)", state(0),
                     state(1), state(2));
      // Store the filtered states
      geometry_msgs::Point pred;
      eigenToPoint(state, &pred);
      this->filtered_hist_.push_back(pred);

      this->feedback_.currentPosition = pred;
      this->feedback_.deltaAltitude =
          this->filtered_hist_.size() > 1
              ? (this->filtered_hist_.back().z -
                 (this->filtered_hist_.at(this->filtered_hist_.size() - 2)).z) >
                        0
                    ? 1.0
                    : -1.0
              : 0.0;

      // After some time, we can start predicting the path of the ball and
      if (flight_time_ > this->startPredictionTime_ || this->filtered_hist_.back().z > this->startPredictionAltitude_) {
        std::vector<geometry_msgs::Point> prediction;
        this->simulateFlight_(&prediction);
        // publish feedback
        this->feedback_.predictedTrajectory = prediction;
        this->feedback_.interceptTime =
            prediction.size() * defaultPredictionTimestep;

      }

      // If the ball is on the ground NO CATCH CONDITION
      if (this->filtered_hist_.back().z < 0.5 && this->filtered_hist_.size() >
                                                      20) {
        // Stop the flight timer
        this->flight_timer_.stop();
        // Stop the vicon subscriber
        this->vicon_sub_.shutdown();
        // Stop the action server
        this->action_server_.setSucceeded();
        ROS_INFO_NAMED(this->name_, "Ball has landed, shutting down");
        
        return;
      }
      // Publish the feedback
      this->action_server_.publishFeedback(this->feedback_);
    }
  }
}

void Estimator::simulateFlight_(std::vector<geometry_msgs::Point> *prediction) {
  // Simulate the flight of the ball
  ROS_INFO_NAMED(this->name_, "Simulating the flight of the ball");
  auto cpy_filter = this->filter_;
  auto cur = this->msg_hist_.back().first;
  prediction->push_back(cur);
  // Simulate the flight of the ball for 1.5 seconds
  while (prediction->back().z > 0.5 && prediction->size() < ros::Duration(2.0,0).toSec() / defaultPredictionTimestep) {
    cpy_filter.step(defaultPredictionTimestep);
    Eigen::Matrix<double, 6, 1> state;
    Eigen::Matrix<double, 6, 6> cov;
    cpy_filter.getStates(state, cov);
    geometry_msgs::Point pred;
    eigenToPoint(state, &pred);
    prediction->push_back(pred);
  }
}