#include "../include/catcher/catcher.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "catcher");
  catcher::Catcher catcher("catcher");
  ros::spin();
  return 0;
}