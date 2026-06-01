#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "scope/mission/mission_waypoint.hpp"

namespace scope
{

struct MissionExecutorParams
{
  std::string waypoint_file;

  std::string odom_topic = "/mavros/local_position/odom";
  std::string goal_topic = "/move_base_simple/goal";
  std::string desired_yaw_topic = "/scope/desired_yaw";
  std::string capture_trigger_topic = "/scope/capture_trigger";
  std::string mission_status_topic = "/scope/mission_status";

  std::string package_name = "scope";

  bool auto_start = true;
  bool enforce_yaw_alignment = false;
  // 默认不发布，防止节点持续向规划器发布航点重规划
  bool republish_goal = false;

  double control_rate = 20.0;
  double goal_republish_period = 1.0;
  double start_delay = 1.0;

  double default_position_tolerance = 0.15;
  double default_yaw_tolerance_deg = 5.0;
  double default_capture_time = 2.0;
};

class MissionExecutor
{
public:
  MissionExecutor() = default;
  ~MissionExecutor() = default;

  bool initialize(const ros::NodeHandle& nh,
                  const ros::NodeHandle& pnh);

private:
  bool loadParams(const ros::NodeHandle& pnh);
  bool loadWaypoints(const std::string& waypoint_file);
  std::string resolvePackageRelativePath(const std::string& path) const;

  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void timerCallback(const ros::TimerEvent& event);

  void publishGoal(const MissionWaypoint& waypoint);
  void publishDesiredYaw(const MissionWaypoint& waypoint);
  void publishCaptureTrigger(bool trigger);
  void publishStatus(const std::string& text);

  void switchState(MissionExecutorState new_state);

  bool hasReachedPosition(const MissionWaypoint& waypoint) const;
  bool hasReachedYaw(const MissionWaypoint& waypoint) const;

  static double normalizeAngle(double angle_rad);
  static double deg2rad(double deg);
  static double rad2deg(double rad);

  static double yawFromQuaternion(const geometry_msgs::Quaternion& q);
  static geometry_msgs::Quaternion quaternionFromYaw(double yaw_rad);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  MissionExecutorParams params_;

  ros::Subscriber odom_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher desired_yaw_pub_;
  ros::Publisher capture_trigger_pub_;
  ros::Publisher mission_status_pub_;
  ros::Timer timer_;

  std::vector<MissionWaypoint> waypoints_;

  MissionExecutorState state_ = MissionExecutorState::WAIT_ODOM;

  bool has_odom_ = false;
  Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero();
  double current_yaw_rad_ = 0.0;

  std::size_t current_index_ = 0;

  ros::Time node_start_time_;
  ros::Time state_start_time_;
  ros::Time last_goal_publish_time_;
};

}  // namespace scope