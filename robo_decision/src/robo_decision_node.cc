#include "include/robo_decision.h"
#include <iostream>
using namespace robo_decision;

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    ROS_ERROR("Please provide path to config file!");
  }

  ros::init(argc, argv, "robo_decision");
  std::string config_path = argv[1];
  RoboDecision robo_decision(config_path);
  robo_decision.Run();
  return 0;
}
