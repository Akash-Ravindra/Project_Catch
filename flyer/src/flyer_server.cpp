#include "../include/flyer/flyer_server.hpp"
#include <unordered_map>

using namespace flyer;



Flyer::Flyer(std::string name) : flight_action_server_(nh_, name, false), action_name_{ name }
{
  // Register the goal and preemt callbacks
  this->flight_action_server_.registerGoalCallback(boost::bind(&Flyer::actionGoalCB, this));
  this->flight_action_server_.registerPreemptCallback(boost::bind(&Flyer::actionPreemptCB, this));
  // Subscribe to the drone state
  this->state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &Flyer::stateCB, this);
  // Subscribe to the drone position
  this->set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  // Subscribe to the drone position
  this->arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  // Land client
  this->land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  // Subscribe to the drone position
  this->setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
  // Start the subscriber to the drone position
  // Start the hold timer
  this->hold_timer_ = nh_.createTimer(ros::Rate(cmdRate), &Flyer::holdCB, this, false, false);
  // Wait for the connection with the drone
  while (ros::ok() && !this->current_state_.connected)
  {
    ros::spinOnce();
    // Check for incoming messages from the drone
    ROS_INFO_NAMED(this->action_name_, "Waiting for the connection with the drone");
    ros::Duration(0.5).sleep();
  }
  // Start the action server
  ROS_INFO_NAMED(this->action_name_, "Action server started");
  this->flight_action_server_.start();
  ROS_INFO_NAMED(this->action_name_, "Drone connected");
}

Flyer::~Flyer()
{
}

void Flyer::actionGoalCB()
{
  auto goal = this->flight_action_server_.acceptNewGoal();
  this->is_goal_active_ = true;
  this->requestedTransform_ = goal->worldToDrone;
  switch ((FlyerCommand)goal->cmdType)
  {
    case FlyerCommand::TAKEOFF:
      ROS_INFO_NAMED(this->action_name_, "Takeoff command received");
      this->takeoff();
      break;
    case FlyerCommand::LAND:
      ROS_INFO_NAMED(this->action_name_, "Land command received");
      this->land();
      break;
    case FlyerCommand::HOME:
      ROS_INFO_NAMED(this->action_name_, "Home command received");
      this->move(Waypoints::HOME, true, false);
      break;
    case FlyerCommand::MOVE:
      ROS_INFO_NAMED(this->action_name_, "Move command received");
      // Convert geometry pose to mavros position target
      this->poseToPositionTarget(goal->goalPose, hold_target_);
      this->move(hold_target_, true);
      break;
    case FlyerCommand::SET_MODE:
      ROS_INFO_NAMED(this->action_name_, "Set mode command received");
      this->setMode(goal->setMode);
      break;
    case FlyerCommand::ARM:
      ROS_INFO_NAMED(this->action_name_, "Arm command received");
      this->arm();
      break;
    case FlyerCommand::DISARM:
      ROS_INFO_NAMED(this->action_name_, "Disarm command received");
      this->disarm();
      break;
    default:
      ROS_INFO_NAMED(this->action_name_, "Unknown command received");
      this->result_.success = false;
      break;
  }

  if(this->result_.success){
    this->flight_action_server_.setSucceeded(this->result_);
  }
  else{
    this->flight_action_server_.setAborted(this->result_);
  }
  this->result_.success = false;
  this->is_goal_active_ = false;
}

void Flyer::holdCB(const ros::TimerEvent& event)
{
  if(ros::ok() && this->current_state_.mode == "OFFBOARD" && this->current_state_.armed)
  {
    ROS_DEBUG_NAMED(this->action_name_, "Hold timer triggered");
    this->setpoint_pub_.publish(this->hold_target_);
  }
  else
  {
    ROS_INFO_NAMED(this->action_name_, "Hold timer stopped");
    this->hold_timer_.stop();
  }
}

void Flyer::dronePosCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_DEBUG_NAMED(this->action_name_, "Drone position received");
  this->current_pose_.is_valid = true;

  this->current_pose_.pos.x() = msg->pose.position.x;
  this->current_pose_.pos.y() = msg->pose.position.y;
  this->current_pose_.pos.z() = msg->pose.position.z;
  this->current_pose_.quat.x() = msg->pose.orientation.x;
  this->current_pose_.quat.y() = msg->pose.orientation.y;
  this->current_pose_.quat.z() = msg->pose.orientation.z;
  this->current_pose_.quat.w() = msg->pose.orientation.w;
  this->current_pose_.euler = this->current_pose_.quat.toRotationMatrix().eulerAngles(0, 1, 2);
  // Rad to deg
  this->current_pose_.euler *= 180.0 / M_PI;

  // Publish feedback
  this->feedback_.dronePose = msg->pose;
  this->flight_action_server_.publishFeedback(this->feedback_);
}
void Flyer::stateCB(const mavros_msgs::State::ConstPtr& msg)
{
  this->current_state_ = *msg;
}

const bool Flyer::setMode(const std::string& mode)
{
  ROS_DEBUG_NAMED(this->action_name_, "Set mode called");
  mavros_msgs::SetMode set_mode;
  set_mode.request.custom_mode = mode;
  // Call the server
  bool success = set_mode_client_.call(set_mode);
  ros::Duration(0.1).sleep();

  if (success && set_mode.response.mode_sent)
  {
    ROS_INFO_NAMED(this->action_name_, "Mode set to %s", mode.c_str());
    this->result_.success = true;
    return true;
  }
  else
  {
    ROS_ERROR_NAMED(this->action_name_, "Failed to set mode to %s", mode.c_str());
    this->result_.success = false;
    return false;
  }
}
const bool Flyer::arm(bool arm)
{
  ROS_DEBUG_NAMED(this->action_name_, "Arm called");
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  // Call the server
  bool success = arming_client_.call(arm_cmd);
  ros::Duration(0.5).sleep();

  if ( success && arm_cmd.response.success)
  {
    ROS_INFO_NAMED(this->action_name_, arm?"Armed":"Disarmed");
    this->result_.success = true;
    return true;
  }
  else
  {
    ROS_ERROR_NAMED(this->action_name_, arm?"Failed to arm":"Failed to disarm");
    this->result_.success = false;
    return false;
  }
}
const bool Flyer::disarm()
{
  return this->arm(false);
}

void Flyer::takeoff()
{
  ROS_DEBUG_NAMED(this->action_name_, "Takeoff called");
  // Set the takeoff height
  this->move(Waypoints::TAKEOFF, true,false);
}
void Flyer::land()
{
  ROS_DEBUG_NAMED(this->action_name_, "Land called");
  // Set the landing height
  this->move(Waypoints::HOME, true, false);
  ros::Duration(2.0).sleep();
  // Land
  mavros_msgs::CommandTOL land_cmd;
  land_cmd.request.altitude = Waypoints::LAND.first.z();
  land_cmd.request.latitude = 0;
  land_cmd.request.longitude = 0;
  land_cmd.request.min_pitch = 0;
  land_cmd.request.yaw = 0;
  // Call the server
  this->result_.success = land_client_.call(land_cmd);
  ros::Duration(0.5).sleep();
  this->disarm();
}
void Flyer::move(const std::pair<Eigen::Vector3d, Eigen::Vector3d>& pos_ori, const bool& hold,const bool& absolute)
{
  ROS_DEBUG_NAMED(this->action_name_, "Move called");
  mavros_msgs::PositionTarget pos_target;
  this->vector3dToPositionTarget(pos_ori, pos_target);
  this->move(pos_target, hold, absolute);
}
void Flyer::validatePositionTarget(mavros_msgs::PositionTarget &pos_target){
    
    pos_target.position.x = std::min(std::max(pos_target.position.x, Waypoints::minX), Waypoints::maxX);
    pos_target.position.y = std::min(std::max(pos_target.position.y, Waypoints::minY), Waypoints::maxY);
    pos_target.position.z = std::min(std::max(pos_target.position.z, Waypoints::minZ), Waypoints::maxZ);
}
void Flyer::move(mavros_msgs::PositionTarget& pos_target, const bool& hold, const bool& absolute){
    // Arm if not
    if (!this->current_state_.armed)
    {
      ROS_INFO_NAMED(this->action_name_, "Arming");
      if(!this->arm()) return;
    }
    // Validate the position target
    this->validatePositionTarget(pos_target);
    if(absolute){
      tf2::doTransform(pos_target.position, pos_target.position, this->requestedTransform_);
    }
    // Switch to off board if not
    this->setpoint_pub_.publish(pos_target);
    ros::spinOnce();
    if (this->current_state_.mode != "OFFBOARD")
    {
      ROS_INFO_NAMED(this->action_name_, "Setting mode to OFFBOARD");
      if(!this->setMode("OFFBOARD")) return;
    }
    this->poseSub(true);
    // Stop the hold timer so that new position target can be sent
    this->hold_timer_.stop();
    // Send the position target
    ROS_INFO_NAMED(this->action_name_, "Sending position target x:%f|y:%f|z:%f", pos_target.position.x, pos_target.position.y, pos_target.position.z);
    double time = ros::Time::now().toSec();
    while(ros::ok() && this->is_goal_active_){
      this->setpoint_pub_.publish(pos_target);
      ros::spinOnce();
      this->rate_.sleep();
      // Send until the drone is at the target position with some tolerance
      if(this->current_pose_.is_valid && this->distance(this->current_pose_.pos, pos_target.position) < Waypoints::tolerance) {this->result_.success=true;break;}
      if(ros::Time::now().toSec() - time > Waypoints::timeout) {this->result_.success=false;break;}
    }
    if(!this->is_goal_active_ || !this->result_.success){
      ROS_WARN_NAMED(this->action_name_, "Goal cancelled or not reached");
      ros::spinOnce();
      pos_target.position.x = this->current_pose_.pos.x();
      pos_target.position.y = this->current_pose_.pos.y();
      pos_target.position.z = this->current_pose_.pos.z();
    }
    // Stop the pose subscriber to avoid getting the same pose
    this->poseSub(false);
    this->current_pose_.is_valid = false;
    
    // Hold if needed
    if(hold && this->result_.success){
      this->hold_target_ = pos_target;
      this->hold_timer_.start();
    }
}