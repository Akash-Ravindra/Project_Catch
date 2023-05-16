#include "../include/catcher/catcher.hpp"

using namespace catcher;

const double CatchingAltitude = 1.2;

Catcher::Catcher(std::string name)
    : nh_{name}, name_{name}, trackerClient_{"estimator_node", true},
      flyerClient_{"flyer_node", true}, spinner_{0} {
  ROS_INFO_NAMED(this->name_, "Catcher started");
  // Set up the tracker action client
  this->trackerState_.trackerClient = &this->trackerClient_;
  this->flyerState_.flyerClient = &this->flyerClient_;
  // Wait for the action servers to be ready
  ROS_INFO_NAMED(this->name_, "Waiting for the tracker action server");
  this->trackerState_.trackerClient->waitForServer();
  ROS_INFO_NAMED(this->name_, "Tracker action server ready");
  ROS_INFO_NAMED(this->name_, "Waiting for the flyer action server");
  this->flyerState_.flyerClient->waitForServer();
  ROS_INFO_NAMED(this->name_, "Flyer action server ready");

  // Wait for transform service
  ROS_INFO_NAMED(this->name_, "Waiting for the transform service");
  this->transformClient_ =
      nh_.serviceClient<reorient::TransformDtoW>("/transform_service");
  this->transformClient_.waitForExistence();
  ROS_INFO_NAMED(this->name_, "Transform service ready");
  this->trackerState_.goal.targetAltitude =
      nh_.param<double>("target_altitude", CatchingAltitude);
  // Start the tick timer
  this->tickTimer_ = nh_.createTimer(ros::Rate(10), &Catcher::tickTimerCallback,
                                     this, false, false);
  this->state_ = STOPPED;
  this->catcherTimer_ =
      nh_.createTimer(ros::Duration(5.0, 0.0), &Catcher::catcherTimerCallback,
                      this, true, false);
  this->catcherTimer_.stop();
  this->stateTimer_ = nh_.createTimer(
      ros::Duration(5.0, 0.0), &Catcher::stateTimerCallback, this, true, false);
  this->stateTimer_.stop();
  // Start timer
  this->tickTimer_.start();
  // Start the async spinner
  this->spinner_.stop();
}
void Catcher::catcherTimerCallback(const ros::TimerEvent &event) {
  this->catcherTimer_.stop();
  ROS_INFO_NAMED(this->name_, "Catching state expired");
  this->state_ = LAND;
}
void Catcher::shutdown() {
  this->tickTimer_.stop();
  this->stateTimer_.stop();
  this->catcherTimer_.stop();
  // this->trackerState_.trackerClient->cancelAllGoals();
  // this->flyerState_.flyerClient->cancelAllGoals();
  this->trackerState_.active = false;
  this->flyerState_.active = false;
  this->spinner_.stop();
  ROS_INFO_NAMED(this->name_, "Catcher shutdown");
}
void Catcher::flyerDoneCallback(
    const actionlib::SimpleClientGoalState &state,
    const flyer::FlyerCommandResultConstPtr &result) {
  switch (this->state_) {
  case STOPPED:
    break;
  case GROUNDED:
    this->state_ = result->success ? IDLE : ERROR;
    break;
  case LAND:
    this->shutdown();
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
    const estimator::ParabolicTrackerFeedbackConstPtr &feedback) {
  switch (this->state_) {
  case IDLE: {
    this->state_ = TRACKING;
  } /// NEEDS TO BE CHANGED BACK TO TRACKER
  break;
  case TRACKING: { // check the feedback for valid traj
    if (feedback->isValid) {
      if (!(feedback->targetPredictionIndex == -1)) {
        auto in_range = std::find_if(
            feedback->predictedTrajectory.begin() +
                feedback->targetPredictionIndex,
            feedback->predictedTrajectory.end(),
            [](geometry_msgs::Point p) { return p.z < 1.2 && p.z > 0.8; });
        if (in_range != feedback->predictedTrajectory.end()) {
          // validate the point
          this->flyerState_.target = *in_range;
          this->flyerState_.targetSet = true;
          // Check if the point is in the catcher
          ROS_INFO_NAMED(this->name_, "Target: %f, %f, %f",
                         this->flyerState_.target.x, this->flyerState_.target.y,
                         this->flyerState_.target.z);
          this->state_ = CATCHING;
        }
      }
    }
    break;
  case CATCHING: {
    this->stateTimer_.stop();
    if (feedback->isValid) {
      static int counter = 5;
      if (!(feedback->targetPredictionIndex == -1)) {
        auto target =
            feedback->predictedTrajectory[feedback->targetPredictionIndex];
        auto difference = sqrt(pow((target.x - this->flyerState_.target.x), 2) +
                               pow((target.y - this->flyerState_.target.y), 2) +
                               pow((target.z - this->flyerState_.target.z), 2));
        if (difference < 0.001 && counter-- < 0) { /// MAGIC NUMBER
          this->state_ = CAUGHT;
        }
        if (!difference > 0.7) { ////MAGIC NUMBER
          this->flyerState_.target = target;
          this->flyerState_.targetSet = true;
        }
      }
    }
  } break;
  case CAUGHT:
  case STOPPED:
  case GROUNDED:
  case ERROR:
    break;
  default:
    break;
  }
  }
}
void Catcher::tickTimerCallback(const ros::TimerEvent &event) {
  this->tickTimer_.stop();
  if(!this->trackerState_.trackerClient->isServerConnected() || !this->flyerState_.flyerClient->isServerConnected())
    {
      ROS_INFO_NAMED(this->name_, "Reconnecting to action servers");
      if(!this->trackerState_.trackerClient->waitForServer(ros::Duration(3.0,0.0)) || !this->flyerState_.flyerClient->waitForServer(ros::Duration(3.0,0.0)))
        {
          ROS_INFO_NAMED(this->name_, "Reconnection failed");
          this->state_ = ERROR;
        }
      else
        {
          ROS_INFO_NAMED(this->name_, "Reconnection successful");
        }
    }
  switch (this->state_) {
  case STOPPED: { // Blocking state
    ROS_INFO_ONCE_NAMED(this->name_, "STOPPED_STATE");
    reorient::TransformDtoW request;
    if (this->transformClient_.isValid() &&
        this->transformClient_.call(request)) {
      this->transformClient_.shutdown();
      // Transform Lookup
      tf2_ros::Buffer tfBuffer_;
      tf2_ros::TransformListener tfListener_(tfBuffer_);
      try {
        this->worldToDrone_ =
            tfBuffer_.lookupTransform("mavrik_inertial", "vicon_world",
                                      ros::Time::now(), ros::Duration(5.0));
      } catch (tf2::TransformException &ex) {
        ROS_ERROR("Could not get transform %s", ex.what());
        this->state_ = ERROR;
      }
      this->state_ = GROUNDED;
    } else {
      this->state_ = ERROR;
    }
  } break;
  case GROUNDED: { // Take off and hover
    ROS_INFO_ONCE_NAMED(this->name_, "GROUNDED_STATE");
    this->flyerState_.goal.cmdType = (int)FlyerCommand::TAKEOFF;
    this->flyerState_.goal.worldToDrone = this->worldToDrone_;
    if (!this->flyerState_.active)
      this->flyerState_.flyerClient->sendGoal(
          this->flyerState_.goal,
          boost::bind(&Catcher::flyerDoneCallback, this, _1, _2),
          boost::bind(&Catcher::flyerActiveCallback, this),
          boost::bind(&Catcher::flyerFeedbackCallback, this, _1));
  } break;
  case IDLE: { // Hovering in air waiting for ball
    ROS_INFO_ONCE_NAMED(this->name_, "IDLE_STATE");
    static double time = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - time > 2 && !this->trackerState_.active) {
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
  } break;
  case TRACKING:
    // Check if feedback
    this->stateTimer_.start();
    ROS_INFO_ONCE_NAMED(this->name_, "TRACKING_STATE");
    break;
  case CATCHING: {
    this->stateTimer_.stop();
    this->catcherTimer_.start();
    ROS_INFO_ONCE_NAMED(this->name_, "CATCHING_STATE");
    if (this->flyerState_.targetSet && !this->flyerState_.active) {
      this->flyerState_.targetSet = false;
      this->flyerState_.goal.cmdType = (int)FlyerCommand::MOVE;
      this->flyerState_.goal.worldToDrone = this->worldToDrone_;
      this->flyerState_.goal.goalPose.position = this->flyerState_.target;
      // Send the goal to the action server
      this->flyerState_.flyerClient->sendGoal(
          this->flyerState_.goal,
          boost::bind(&Catcher::flyerDoneCallback, this, _1, _2),
          boost::bind(&Catcher::flyerActiveCallback, this),
          boost::bind(&Catcher::flyerFeedbackCallback, this, _1));
    }
  } break;
  case CAUGHT:
    this->catcherTimer_.stop();
    this->stateTimer_.start();

    ROS_INFO_ONCE_NAMED(this->name_, "CAUGHT_STATE");
    break;
  case LAND:
    ROS_INFO_ONCE_NAMED(this->name_, "LAND_STATE");
    this->flyerState_.goal.cmdType = (int)FlyerCommand::LAND;
    this->flyerState_.goal.worldToDrone = this->worldToDrone_;
    this->flyerState_.flyerClient->sendGoal(
        this->flyerState_.goal,
        boost::bind(&Catcher::flyerDoneCallback, this, _1, _2),
        boost::bind(&Catcher::flyerActiveCallback, this),
        boost::bind(&Catcher::flyerFeedbackCallback, this, _1));
    break;
  case ERROR:
    ROS_INFO_ONCE_NAMED(this->name_, "ERROR_STATE");
    break;
  default:
    ROS_INFO_ONCE_NAMED(this->name_, "UNKNOWN_STATE");
    break;
  }
  this->tickTimer_.start();
}
