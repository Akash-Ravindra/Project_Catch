#include "../include/estimator/estimator.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "estimator");
  et::Estimator estimator(ros::this_node::getName());
  ros::spin();
  ROS_INFO("Exiting the estimator node");
  return 0;
}