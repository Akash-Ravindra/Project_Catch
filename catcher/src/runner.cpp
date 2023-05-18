#include "../include/catcher/catcher.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "catcher");
  catcher::Catcher catcher("catcher");
  ros::waitForShutdown();
  return 0;
}