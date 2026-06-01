#include <ros/ros.h>

#include "scope/mission/mission_executor.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scope_mission_executor_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  scope::MissionExecutor executor;

  if (!executor.initialize(nh, pnh))
  {
    ROS_ERROR_STREAM("[SCOPE][MissionExecutorNode] Initialization failed.");
    return 1;
  }

  ros::spin();

  return 0;
}