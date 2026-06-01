#include "scope/mission/mission_executor.hpp"

#include <cmath>
#include <sstream>

#include <ros/package.h>
#include <yaml-cpp/yaml.h>

namespace scope
{

bool MissionExecutor::initialize(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& pnh)
{
  nh_ = nh;
  pnh_ = pnh;

  if (!loadParams(pnh_))
  {
    ROS_ERROR_STREAM("[SCOPE][MissionExecutor] Failed to load parameters.");
    return false;
  }

  const std::string resolved_waypoint_file =
      resolvePackageRelativePath(params_.waypoint_file);

  ROS_INFO_STREAM("[SCOPE][MissionExecutor] Waypoint file: "
                  << resolved_waypoint_file);

  if (!loadWaypoints(resolved_waypoint_file))
  {
    ROS_ERROR_STREAM("[SCOPE][MissionExecutor] Failed to load waypoint file.");
    return false;
  }

  odom_sub_ = nh_.subscribe(params_.odom_topic,
                            10,
                            &MissionExecutor::odomCallback,
                            this);

  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      params_.goal_topic, 1, true);

  desired_yaw_pub_ = nh_.advertise<std_msgs::Float64>(
      params_.desired_yaw_topic, 1, true);

  capture_trigger_pub_ = nh_.advertise<std_msgs::Bool>(
      params_.capture_trigger_topic, 1, true);

  mission_status_pub_ = nh_.advertise<std_msgs::String>(
      params_.mission_status_topic, 1, true);

  const double rate = std::max(1.0, params_.control_rate);
  timer_ = nh_.createTimer(ros::Duration(1.0 / rate),
                           &MissionExecutor::timerCallback,
                           this);

  node_start_time_ = ros::Time::now();
  state_start_time_ = node_start_time_;
  last_goal_publish_time_ = ros::Time(0);

  if (params_.auto_start)
  {
    switchState(MissionExecutorState::WAIT_ODOM);
  }
  else
  {
    switchState(MissionExecutorState::IDLE);
  }

  ROS_INFO_STREAM("[SCOPE][MissionExecutor] Initialized. waypoint_num="
                  << waypoints_.size());

  return true;
}

bool MissionExecutor::loadParams(const ros::NodeHandle& pnh)
{
  pnh.param<std::string>("waypoint_file",
                         params_.waypoint_file,
                         std::string("log/output/selected_waypoints.yaml"));

  pnh.param<std::string>("odom_topic",
                         params_.odom_topic,
                         params_.odom_topic);

  pnh.param<std::string>("goal_topic",
                         params_.goal_topic,
                         params_.goal_topic);

  pnh.param<std::string>("desired_yaw_topic",
                         params_.desired_yaw_topic,
                         params_.desired_yaw_topic);

  pnh.param<std::string>("capture_trigger_topic",
                         params_.capture_trigger_topic,
                         params_.capture_trigger_topic);

  pnh.param<std::string>("mission_status_topic",
                         params_.mission_status_topic,
                         params_.mission_status_topic);

  pnh.param<std::string>("package_name",
                         params_.package_name,
                         params_.package_name);

  pnh.param<bool>("auto_start",
                  params_.auto_start,
                  params_.auto_start);

  pnh.param<bool>("enforce_yaw_alignment",
                  params_.enforce_yaw_alignment,
                  params_.enforce_yaw_alignment);

  pnh.param<bool>("republish_goal",
                  params_.republish_goal,
                  params_.republish_goal);

  pnh.param<double>("control_rate",
                    params_.control_rate,
                    params_.control_rate);

  pnh.param<double>("goal_republish_period",
                    params_.goal_republish_period,
                    params_.goal_republish_period);

  pnh.param<double>("start_delay",
                    params_.start_delay,
                    params_.start_delay);

  pnh.param<double>("default_position_tolerance",
                    params_.default_position_tolerance,
                    params_.default_position_tolerance);

  pnh.param<double>("default_yaw_tolerance_deg",
                    params_.default_yaw_tolerance_deg,
                    params_.default_yaw_tolerance_deg);

  pnh.param<double>("default_capture_time",
                    params_.default_capture_time,
                    params_.default_capture_time);

  if (params_.waypoint_file.empty())
  {
    ROS_ERROR_STREAM("[SCOPE][MissionExecutor] waypoint_file is empty.");
    return false;
  }

  return true;
}

bool MissionExecutor::loadWaypoints(const std::string& waypoint_file)
{
  YAML::Node root;

  try
  {
    root = YAML::LoadFile(waypoint_file);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("[SCOPE][MissionExecutor] Failed to load yaml: "
                     << waypoint_file << ". Error: " << e.what());
    return false;
  }

  const YAML::Node waypoint_nodes = root["waypoints"];

  if (!waypoint_nodes || !waypoint_nodes.IsSequence())
  {
    ROS_ERROR_STREAM("[SCOPE][MissionExecutor] No valid 'waypoints' sequence.");
    return false;
  }

  waypoints_.clear();
  waypoints_.reserve(waypoint_nodes.size());

  for (std::size_t i = 0; i < waypoint_nodes.size(); ++i)
  {
    const YAML::Node node = waypoint_nodes[i];

    MissionWaypoint wp;

    wp.id = node["id"] ? node["id"].as<int>() : static_cast<int>(i);

    wp.source_candidate_id =
        node["source_candidate_id"] ?
        node["source_candidate_id"].as<int>() :
        static_cast<int>(wp.id);

    if (!node["position"] || !node["position"].IsSequence() ||
        node["position"].size() != 3)
    {
      ROS_WARN_STREAM("[SCOPE][MissionExecutor] Skip invalid waypoint position at index "
                      << i);
      continue;
    }

    wp.position = Eigen::Vector3d(node["position"][0].as<double>(),
                                  node["position"][1].as<double>(),
                                  node["position"][2].as<double>());

    if (node["target_point"] && node["target_point"].IsSequence() &&
        node["target_point"].size() == 3)
    {
      wp.target_point = Eigen::Vector3d(node["target_point"][0].as<double>(),
                                        node["target_point"][1].as<double>(),
                                        node["target_point"][2].as<double>());
    }

    wp.yaw_deg =
        node["yaw_deg"] ? node["yaw_deg"].as<double>() : 0.0;

    wp.pitch_deg =
        node["pitch_deg"] ? node["pitch_deg"].as<double>() : 0.0;

    wp.view_distance =
        node["view_distance"] ? node["view_distance"].as<double>() : 0.0;

    wp.score =
        node["score"] ? node["score"].as<double>() : 0.0;

    wp.coverage_gain =
        node["coverage_gain"] ? node["coverage_gain"].as<double>() : 0.0;

    wp.safety_clearance =
        node["safety_clearance"] ? node["safety_clearance"].as<double>() : 0.0;

    wp.covered_surface_count =
        node["covered_surface_count"] ?
        node["covered_surface_count"].as<int>() :
        0;

    wp.capture_time =
        node["capture_time"] ?
        node["capture_time"].as<double>() :
        params_.default_capture_time;

    wp.position_tolerance =
        node["position_tolerance"] ?
        node["position_tolerance"].as<double>() :
        params_.default_position_tolerance;

    wp.yaw_tolerance_deg =
        node["yaw_tolerance_deg"] ?
        node["yaw_tolerance_deg"].as<double>() :
        params_.default_yaw_tolerance_deg;

    waypoints_.push_back(wp);
  }

  if (waypoints_.empty())
  {
    ROS_ERROR_STREAM("[SCOPE][MissionExecutor] No valid waypoints loaded.");
    return false;
  }

  ROS_INFO_STREAM("[SCOPE][MissionExecutor] Loaded waypoints: "
                  << waypoints_.size());

  return true;
}

std::string MissionExecutor::resolvePackageRelativePath(
    const std::string& path) const
{
  if (path.empty())
  {
    return path;
  }

  if (path.front() == '/')
  {
    return path;
  }

  const std::string package_path =
      ros::package::getPath(params_.package_name);

  if (package_path.empty())
  {
    ROS_WARN_STREAM("[SCOPE][MissionExecutor] Failed to locate package: "
                    << params_.package_name
                    << ". Use path directly: "
                    << path);
    return path;
  }

  return package_path + "/" + path;
}

void MissionExecutor::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  current_position_.x() = msg->pose.pose.position.x;
  current_position_.y() = msg->pose.pose.position.y;
  current_position_.z() = msg->pose.pose.position.z;

  current_yaw_rad_ = yawFromQuaternion(msg->pose.pose.orientation);

  has_odom_ = true;
}

void MissionExecutor::timerCallback(const ros::TimerEvent&)
{
  if (waypoints_.empty())
  {
    switchState(MissionExecutorState::ERROR);
    publishStatus("No waypoint loaded.");
    return;
  }

  const ros::Time now = ros::Time::now();

  if ((now - node_start_time_).toSec() < params_.start_delay)
  {
    publishStatus("Waiting for start delay.");
    return;
  }

  if (!has_odom_)
  {
    switchState(MissionExecutorState::WAIT_ODOM);
    publishStatus("Waiting for odometry.");
    return;
  }

  if (current_index_ >= waypoints_.size())
  {
    publishCaptureTrigger(false);
    switchState(MissionExecutorState::FINISHED);
    publishStatus("Mission finished.");
    return;
  }

  const MissionWaypoint& wp = waypoints_[current_index_];

  switch (state_)
  {
    case MissionExecutorState::WAIT_ODOM:
    case MissionExecutorState::IDLE:
    {
      switchState(MissionExecutorState::SEND_GOAL);
      break;
    }

    case MissionExecutorState::SEND_GOAL:
    {
      publishGoal(wp);
      last_goal_publish_time_ = now;
      switchState(MissionExecutorState::WAIT_POSITION);
      break;
    }

    case MissionExecutorState::WAIT_POSITION:
    {
      if (params_.republish_goal &&
          (now - last_goal_publish_time_).toSec() >=
              params_.goal_republish_period)
      {
        publishGoal(wp);
        last_goal_publish_time_ = now;
      }

      if (hasReachedPosition(wp))
      {
        switchState(MissionExecutorState::ALIGN_YAW);
      }

      break;
    }

    case MissionExecutorState::ALIGN_YAW:
    {
      publishDesiredYaw(wp);

      if (!params_.enforce_yaw_alignment || hasReachedYaw(wp))
      {
        publishCaptureTrigger(true);
        switchState(MissionExecutorState::CAPTURE);
      }

      break;
    }

    case MissionExecutorState::CAPTURE:
    {
      publishDesiredYaw(wp);
      publishCaptureTrigger(true);

      if ((now - state_start_time_).toSec() >= wp.capture_time)
      {
        publishCaptureTrigger(false);
        ++current_index_;

        if (current_index_ >= waypoints_.size())
        {
          switchState(MissionExecutorState::FINISHED);
        }
        else
        {
          switchState(MissionExecutorState::SEND_GOAL);
        }
      }

      break;
    }

    case MissionExecutorState::FINISHED:
    {
      publishCaptureTrigger(false);
      publishStatus("Mission finished.");
      break;
    }

    case MissionExecutorState::ERROR:
    default:
    {
      publishCaptureTrigger(false);
      publishStatus("Mission executor error.");
      break;
    }
  }
}

void MissionExecutor::publishGoal(const MissionWaypoint& waypoint)
{
  geometry_msgs::PoseStamped goal;

  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "camera_init";

  goal.pose.position.x = waypoint.position.x();
  goal.pose.position.y = waypoint.position.y();
  goal.pose.position.z = waypoint.position.z();

  goal.pose.orientation =
      quaternionFromYaw(deg2rad(waypoint.yaw_deg));

  goal_pub_.publish(goal);

  std::ostringstream oss;
  oss << "Publish EGO goal. index=" << current_index_
      << ", id=" << waypoint.id
      << ", position=[" << waypoint.position.x()
      << ", " << waypoint.position.y()
      << ", " << waypoint.position.z()
      << "], yaw_deg=" << waypoint.yaw_deg;

  publishStatus(oss.str());
  ROS_INFO_STREAM("[SCOPE][MissionExecutor] " << oss.str());
}

void MissionExecutor::publishDesiredYaw(const MissionWaypoint& waypoint)
{
  std_msgs::Float64 msg;
  msg.data = deg2rad(waypoint.yaw_deg);
  desired_yaw_pub_.publish(msg);
}

void MissionExecutor::publishCaptureTrigger(bool trigger)
{
  std_msgs::Bool msg;
  msg.data = trigger;
  capture_trigger_pub_.publish(msg);
}

void MissionExecutor::publishStatus(const std::string& text)
{
  std_msgs::String msg;

  std::ostringstream oss;
  oss << "[" << missionStateToString(state_) << "] "
      << "wp=" << current_index_ << "/"
      << waypoints_.size() << ". "
      << text;

  msg.data = oss.str();
  mission_status_pub_.publish(msg);
}

void MissionExecutor::switchState(MissionExecutorState new_state)
{
  if (state_ == new_state)
  {
    return;
  }

  ROS_INFO_STREAM("[SCOPE][MissionExecutor] State "
                  << missionStateToString(state_)
                  << " -> "
                  << missionStateToString(new_state));

  state_ = new_state;
  state_start_time_ = ros::Time::now();
}

bool MissionExecutor::hasReachedPosition(
    const MissionWaypoint& waypoint) const
{
  const double error = (current_position_ - waypoint.position).norm();
  return error <= waypoint.position_tolerance;
}

bool MissionExecutor::hasReachedYaw(
    const MissionWaypoint& waypoint) const
{
  const double desired_yaw = deg2rad(waypoint.yaw_deg);
  const double yaw_error = normalizeAngle(desired_yaw - current_yaw_rad_);

  return std::abs(rad2deg(yaw_error)) <= waypoint.yaw_tolerance_deg;
}

double MissionExecutor::normalizeAngle(double angle_rad)
{
  while (angle_rad > M_PI)
  {
    angle_rad -= 2.0 * M_PI;
  }

  while (angle_rad < -M_PI)
  {
    angle_rad += 2.0 * M_PI;
  }

  return angle_rad;
}

double MissionExecutor::deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

double MissionExecutor::rad2deg(double rad)
{
  return rad * 180.0 / M_PI;
}

double MissionExecutor::yawFromQuaternion(
    const geometry_msgs::Quaternion& q)
{
  const double siny_cosp =
      2.0 * (q.w * q.z + q.x * q.y);

  const double cosy_cosp =
      1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::Quaternion MissionExecutor::quaternionFromYaw(double yaw_rad)
{
  geometry_msgs::Quaternion q;

  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw_rad * 0.5);
  q.w = std::cos(yaw_rad * 0.5);

  return q;
}

}  // namespace scope