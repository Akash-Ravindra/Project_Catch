#include "../include/catcher/catcher.hpp"

using namespace catcher;

Catcher::Catcher(std::string name)
    : nh_{name},
      name_{name},
      trackerClient_{"estimator", true},
      flyerClient_{"flyer", true},
      spinner_{0} {
  ROS_INFO_NAMED(this->name_, "Catcher started");
  // Set up the tracker action client
  this->trackerState_.trackerClient = &this->trackerClient_;
  this->flyerState_.flyerClient = &this->flyerClient_;
  // Wait for the action servers to be ready
  ROS_INFO_NAMED(this->name_, "Waiting for the tracker action server");
  this->trackerState_.trackerClient->waitForResult();
  ROS_INFO_NAMED(this->name_, "Tracker action server ready");
  ROS_INFO_NAMED(this->name_, "Waiting for the flyer action server");
  this->flyerState_.flyerClient->waitForServer();
  ROS_INFO_NAMED(this->name_, "Flyer action server ready");

  // Wait for transform service
  ROS_INFO_NAMED(this->name_, "Waiting for the transform service");
  this->transformClient_ =
      nh_.serviceClient<reorient::TransformDtoW>("transform_service");
  this->transformClient_.waitForExistence();
  ROS_INFO_NAMED(this->name_, "Transform service ready");

  // Start the tick timer
  this->tickTimer_ = nh_.createTimer(ros::Rate(10), &Catcher::tickTimerCallback,
                                     this, false, false);
  this->state_ = STOPPED;

  // Start timer
  this->tickTimer_.start();
  // Start the async spinner
  this->spinner_.stop();
}
void Catcher::flyerDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const flyer::FlyerCommandResultConstPtr& result) {
  switch (this->state_) {
    case STOPPED:
      break;
    case GROUNDED:
      this->state_ = result->success ? IDLE : ERROR;
      break;
    case IDLE:
      break;
    case TRACKING:
      break;
    case CATCHING:
      break;
    case CAUGHT:
      break;
    case ERROR:
      break;
    default:
      break;
  }
  this->flyerState_.active = false;
}

void Catcher::trackerFeedbackCallback(
    const estimator::ParabolicTrackerFeedbackConstPtr& feedback) {
  switch (this->state_) {
    case IDLE:
      this->state_ = TRACKING;
      break;
    case TRACKING:
      // check the feedback for valid traj
      if (feedback->isValid) {
        auto in_range =
            std::find_if(feedback->predictedTrajectory.begin(),
                         feedback->predictedTrajectory.end(),
                         [](geometry_msgs::Point p) { return p.z < 3; });
      }
      break;
    case CATCHING:
      break;
    case CAUGHT:
      break;
    case STOPPED:
    case GROUNDED:
    case ERROR:
      break;
    default:
      break;
  }
}

void Catcher::tickTimerCallback(const ros::TimerEvent& event) {
  this->tickTimer_.stop();
  switch (this->state_) {
    case STOPPED:
      // Blocking state
      auto request = reorient::TransformDtoW();
      if (this->transformClient_.isValid() &&
          this->transformClient_.call(request)) {
        this->state_ = GROUNDED;
        this->transformClient_.shutdown();
      } else {
        this->state_ = ERROR;
      }
      break;
    case GROUNDED:
      // Take off and hover
      this->flyerState_.goal.cmdType = (int)FlyerCommand::TAKEOFF;
      if (!this->flyerState_.active)
        this->flyerClient_.sendGoal(
            this->flyerState_.goal,
            boost::bind(&Catcher::flyerDoneCallback, this, _1, _2),
            boost::bind(&Catcher::flyerActiveCallback, this),
            boost::bind(&Catcher::flyerFeedbackCallback, this, _1));
      break;
    case IDLE:
      // Hovering in air waiting for ball
      static double time = ros::Time::now().toSec();
      if (ros::Time::now().toSec() - time > 5 && !this->trackerState_.active) {
        this->trackerState_.goal.startTracker = true;
        // Make sure the action server has no goals
        this->trackerState_.trackerClient->cancelAllGoals();
        ros::Duration(5.0).sleep();
        // Send the goal to the action server
        this->trackerState_.trackerClient->sendGoal(
            this->trackerState_.goal,
            boost::bind(&Catcher::trackerDoneCallback, this, _1, _2),
            boost::bind(&Catcher::trackerActiveCallback, this),
            boost::bind(&Catcher::trackerFeedbackCallback, this, _1));
        this->trackerState_.active = true;
      }
      break;
    case TRACKING:
      // Check if feedback
      break;
    case CATCHING:
      break;
    case CAUGHT:
      break;
    case ERROR:
      break;
    default:
      break;
  }
  this->tickTimer_.start();
}
