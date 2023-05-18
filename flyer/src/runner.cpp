#include "../include/flyer/flyer_server.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "Flyer_node");
  flyer::Flyer flyer(ros::this_node::getName());
  ros::spin();
  ROS_INFO("Exiting the estimator node");
  return 0;
}