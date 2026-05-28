#include <ros/ros.h>

#include "scope/common/parameters.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scope_offline_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO_STREAM("[SCOPE] Offline node started.");

  std::string config_file;
  pnh.param<std::string>("config_file", config_file, "");

  if (config_file.empty())
  {
    ROS_ERROR_STREAM("[SCOPE] Parameter '~config_file' is empty.");
    ROS_ERROR_STREAM("[SCOPE] Please launch with a valid config file, for example:");
    ROS_ERROR_STREAM("[SCOPE] roslaunch scope scope_offline.launch");
    return 1;
  }

  ROS_INFO_STREAM("[SCOPE] Config file: " << config_file);

  scope::ScopeParameters params;

  if (!params.loadFromYamlFile(config_file))
  {
    ROS_ERROR_STREAM("[SCOPE] Failed to load SCOPE parameters.");
    return 1;
  }

  params.printSummary();

  ROS_INFO_STREAM("[SCOPE] Parameter loading finished.");
  ROS_INFO_STREAM("[SCOPE] V0.1 current stage only checks parameter parsing.");
  ROS_INFO_STREAM("[SCOPE] Next step: connect PCD loader and cloud preprocessor.");

  ros::spin();
  return 0;
}