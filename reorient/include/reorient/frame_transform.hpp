#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <reorient/TransformDtoW.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <tf2_ros/static_transform_broadcaster.h>
namespace tr{

    class Transform{
        private:
        protected:
        //Node handle
        ros::NodeHandle nh_;
        //Subscriber for vicon
        ros::Subscriber vicon_sub_;
        //Subscriber for drone
        ros::Subscriber drone_sub_;
        //Service
        ros::ServiceServer service_;
        Eigen::Matrix4d drone_poses_;
        Eigen::Matrix4d vicon_poses_;
        tf2_ros::StaticTransformBroadcaster br_;
        /// @brief Timer meant to handle timeouts
        ros::Timer action_timeout_; 
        public:
        Transform();
        ~Transform();
        void vicon_callback(const geometry_msgs::TransformStampedConstPtr& msg);
        void drone_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void timer_callback(const ros::TimerEvent& event);
        bool transform_service_callback(reorient::TransformDtoWRequest& req,reorient::TransformDtoWResponse& resp);
    };
}
