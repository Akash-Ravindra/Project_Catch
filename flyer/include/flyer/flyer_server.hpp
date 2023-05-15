#ifndef FLYER_HPP
#define FLYER_HPP
#include <ros/ros.h>
#include "./waypoint.hpp"
#include <flyer/FlyerCommandAction.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>

constexpr double cmdRate = 20.0;

namespace flyer
{
class Flyer
{
private:
    ros::NodeHandle nh_;
    // Action server
    actionlib::SimpleActionServer<flyer::FlyerCommandAction> flight_action_server_;
    // Action server name
    std::string action_name_;
    // Action server Feedback
    flyer::FlyerCommandFeedback feedback_;
    // Action server result
    flyer::FlyerCommandResult result_;
    // Action server goal
    bool is_goal_active_;
    
    // Setpoint publisher
    ros::Publisher setpoint_pub_;
    // Change drone mode service client
    ros::ServiceClient set_mode_client_;
    // Subscriber to drone position
    ros::Subscriber drone_pos_sub;
    // Server client to arm the drone
    ros::ServiceClient arming_client_;
    // Subscriber to drone state
    ros::Subscriber state_sub_;
    // Sleep rate
    ros::Rate rate_ = ros::Rate(cmdRate);
    // Hold timer
    ros::Timer hold_timer_;
    // Hold target
    mavros_msgs::PositionTarget hold_target_;
    // Current drone State
    mavros_msgs::State current_state_;
    // The world to drone transform from client
    geometry_msgs::TransformStamped requestedTransform_;

    // Hold current drone position
    void holdCB(const ros::TimerEvent &event);
    struct DronePose
    {
        bool is_valid;
        Eigen::Vector3d pos;
        Eigen::Quaterniond quat;
        Eigen::Vector3d euler;
    }current_pose_;
    /// @brief Start and stop the subscriber to the drone position
    /// @param start 
    inline void poseSub(bool start = true){
        if(start)
        this->drone_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &Flyer::dronePosCB, this);
        else this->drone_pos_sub.shutdown();
    }
    /// @brief Convert the drone position and orientation pair to mavros position target
    /// @param vec 
    /// @param pos_target 
    inline void vector3dToPositionTarget(const std::pair<Eigen::Vector3d, Eigen::Vector3d> &vec, mavros_msgs::PositionTarget &pos_target)
    {
        pos_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        pos_target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                               mavros_msgs::PositionTarget::IGNORE_VY |
                               mavros_msgs::PositionTarget::IGNORE_VZ |
                               mavros_msgs::PositionTarget::IGNORE_AFX |
                               mavros_msgs::PositionTarget::IGNORE_AFY |
                               mavros_msgs::PositionTarget::IGNORE_AFZ |
                               mavros_msgs::PositionTarget::FORCE |
                               mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        pos_target.position.x = vec.first.x();
        pos_target.position.y = vec.first.y();
        pos_target.position.z = vec.first.z();
        pos_target.velocity.x = 0;
        pos_target.velocity.y = 0;
        pos_target.velocity.z = 0;
        pos_target.acceleration_or_force.x = 0;
        pos_target.acceleration_or_force.y = 0;
        pos_target.acceleration_or_force.z = 0;
        pos_target.yaw = (vec.second.z() + 90.0) * M_PI / 180.0;
        pos_target.yaw_rate = 0;
    }
    inline void poseToPositionTarget(const geometry_msgs::Pose &pose, mavros_msgs::PositionTarget &pos_target){
        this->vector3dToPositionTarget(std::make_pair(Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z), Eigen::Quaterniond(pose.orientation.w,pose.orientation.x,pose.orientation.y, pose.orientation.z).toRotationMatrix().eulerAngles(0, 1, 2)), pos_target);
    }
    inline double distance(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2){
        return sqrt(pow(vec1.x() - vec2.x(), 2) + pow(vec1.y() - vec2.y(), 2) + pow(vec1.z() - vec2.z(), 2));
    }
    inline double distance(const Eigen::Vector3d &vec1, const geometry_msgs::Point &point1){
        return sqrt(pow(vec1.x() - point1.x, 2) + pow(vec1.y() - point1.y, 2) + pow(vec1.z() - point1.z, 2));
    }
    inline void actionPreemptCB(){
        ROS_INFO("%s: Preempted", this->action_name_.c_str());
        this->flight_action_server_.setPreempted();
        this->is_goal_active_ = false;
    }
public:
    Flyer(std::string name);
    ~Flyer();
    /// @brief A call back function for the action server
    /// @param goal 
    void actionGoalCB();
    /// @brief A call back function for the drone position subscriber
    /// @param msg 
    void dronePosCB(const geometry_msgs::PoseStamped::ConstPtr &msg);
    /// @brief A callback function for the state subscriber
    /// @param msg 
    void stateCB(const mavros_msgs::State::ConstPtr &msg);
    /// @brief Change the mode of the drone to offboard or manual
    /// @param mode 
    const bool setMode(const std::string &mode);
    /// @brief Arm the drone using the service client
    const bool arm(bool arm = true);
    /// @brief Disarm the drone using the service client
    const bool disarm();
    /// @brief Take off the drone using the service client
    /// @param height 
    void takeoff();
    /// @brief  Land the drone using the service client
    void land();
    /// @brief Given the pose and orientation it moves
    /// @param pos_target
    void move(mavros_msgs::PositionTarget &pos_target, const bool &hold = true);
    /// @brief Given the pair of pos and ori it moves
    /// @param vec 
    void move(const std::pair<Eigen::Vector3d, Eigen::Vector3d> &vec, const bool &hold = true);
    /// @brief Ensures that the drone is always within the limits
    /// @param pos_target 
    void validatePositionTarget(mavros_msgs::PositionTarget &pos_target);
};
} // namespace flyer
#endif