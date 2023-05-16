#ifndef CATHER_CATCHER_HPP
#define CATHER_CATCHER_HPP
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <estimator/ParabolicTrackerAction.h>
#include <flyer/FlyerCommandAction.h>
#include <reorient/TransformDtoW.h>
#include "../../../flyer/include/flyer/waypoint.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace catcher{
    class Catcher{
        private:
        // ROS Node Handle
        ros::NodeHandle nh_;

        // Tracker action client
        actionlib::SimpleActionClient<estimator::ParabolicTrackerAction> trackerClient_;
        struct trackerStruct{
            actionlib::SimpleActionClient<estimator::ParabolicTrackerAction> *trackerClient;
            bool active = false;
            estimator::ParabolicTrackerGoal goal;
        }trackerState_;
        // Active callback
        void trackerActiveCallback(){this->trackerState_.active = true;}
        // Feedback callback check the state of the tracker
        void trackerFeedbackCallback(const estimator::ParabolicTrackerFeedbackConstPtr& feedback);
        // Done callback
        void trackerDoneCallback(const actionlib::SimpleClientGoalState& state, const estimator::ParabolicTrackerResultConstPtr& result){this->trackerState_.active = false;};

        // Flyer action client
        actionlib::SimpleActionClient<flyer::FlyerCommandAction> flyerClient_;
        struct flyerStruct{
            actionlib::SimpleActionClient<flyer::FlyerCommandAction> *flyerClient;
            bool active = false;
            flyer::FlyerCommandGoal goal;
            geometry_msgs::Point target;
            bool targetSet = false;
        }flyerState_;
        // Active callback
        void flyerActiveCallback(){this->flyerState_.active = true;}
        // Feedback callback
        void flyerFeedbackCallback(const flyer::FlyerCommandFeedbackConstPtr& feedback){};
        // Done callback
        void flyerDoneCallback(const actionlib::SimpleClientGoalState& state, const flyer::FlyerCommandResultConstPtr& result);
        // Transform service client
        ros::ServiceClient transformClient_;
        // Tick Timer
        ros::Timer tickTimer_;
        // Catcher timer
        ros::Timer catcherTimer_;
        /// @brief A timeout for catching state 
        /// @param event 
        void catcherTimerCallback(const ros::TimerEvent& event);
        // Current state
        enum State {STOPPED, GROUNDED, IDLE, TRACKING, CATCHING, CAUGHT, LAND, ERROR};
        State state_;
        State nextState_ = LAND;
        // State transition timer
        ros::Timer stateTimer_;
        /// @brief Delay the state transition
        void stateTimerCallback(const ros::TimerEvent& event){this->state_ = this->nextState_;}
        // Node name
        std::string name_;

        //Async spinner
        ros::AsyncSpinner spinner_;

        geometry_msgs::TransformStamped worldToDrone_=geometry_msgs::TransformStamped();

        public:
        /// @brief 
        /// @param name 
        Catcher(std::string name);
        /// @brief
        /// @param
        ~Catcher(){};
        /// @brief 
        /// @param event 
        void tickTimerCallback(const ros::TimerEvent& event);
        void shutdown();
    };
}


#endif