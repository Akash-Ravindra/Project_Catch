#include "../include/catcher/catcher.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "catcher");
  catcher::Catcher catcher("catcher");
  ros::spin();
  // while(ros::ok()){
  //   catcher.tick();
  //   ros::Duration(0.001).sleep();
  //   ros::spinOnce();
  // }
  return 0;
}