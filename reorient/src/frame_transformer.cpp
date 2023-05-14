#include "../include/reorient/frame_transform.hpp"
using namespace tr;

Transform::Transform(){
    //Set up timer
    // action_timeout_ = nh_.createTimer(ros::Duration(0.1), &Transform::timer_callback, this);
    // Service
    service_ = nh_.advertiseService("transform_service", &Transform::transform_service_callback, this);
};
Transform::~Transform(){};
void Transform::vicon_callback(const geometry_msgs::TransformStampedConstPtr& msg){
    ROS_INFO("VICON CALLBACK");
    // conver pose to matrix
    Eigen::Matrix4d pose;
    Eigen::Quaterniond q(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z);
    Eigen::Vector3d t(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);
    pose<< q.toRotationMatrix().transpose(), -q.toRotationMatrix().transpose()*t, 0, 0, 0, 1;
    this->vicon_poses_ = pose;
    
}
void Transform::drone_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO("DRONE CALLBACK");\
    // conver pose to matrix
    Eigen::Matrix4d pose;
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d t(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    pose << q.toRotationMatrix(), t, 0, 0, 0, 1;
    this->drone_poses_ = pose;
    }
bool Transform::transform_service_callback(reorient::TransformDtoWRequest& req,reorient::TransformDtoWResponse& resp){
    ROS_INFO_NAMED("Transform", "Transform service called");
    // Set the translation values from VICON

    this->drone_sub_=nh_.subscribe(req.dronePoseTopic.c_str(), 1, &Transform::drone_callback, this);
    this->vicon_sub_=nh_.subscribe(req.viconTopic.c_str(), 1, &Transform::vicon_callback, this);

    /// wait for sometime to get the data
    ros::Duration(1.0).sleep();

    //Turn off the subscribers
    this->drone_sub_.shutdown();
    this->vicon_sub_.shutdown();

    // Do math
    Eigen::Matrix4d T = this->vicon_poses_*this->drone_poses_;
    geometry_msgs::Vector3 t;
    t.x = T(0,3);
    t.y = T(1,3);
    t.z = T(2,3);
    geometry_msgs::Quaternion q;
    Eigen::Quaterniond q_eigen(T.block<3,3>(0,0));
    q_eigen.normalize();
    q.x = q_eigen.x();
    q.y = q_eigen.y();
    q.z = q_eigen.z();
    q.w = q_eigen.w();

    // publish the transform
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "vicon_world";
    transformStamped.child_frame_id = req.droneFrame;
    transformStamped.transform.translation = t;
    transformStamped.transform.rotation = q;

    // perform static broadcast
    br_.sendTransform(transformStamped);

    // return the transform
    resp.droneHome = transformStamped.transform;

    return true;
}