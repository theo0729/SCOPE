#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Core>

namespace scope
{

struct MissionWaypoint
{
  std::uint32_t id = 0;
  std::uint32_t source_candidate_id = 0;

  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_point = Eigen::Vector3d::Zero();

  double yaw_deg = 0.0;
  double pitch_deg = 0.0;

  double view_distance = 0.0;
  double score = 0.0;
  double coverage_gain = 0.0;
  double safety_clearance = 0.0;

  int covered_surface_count = 0;

  double capture_time = 2.0;
  double position_tolerance = 0.15;
  double yaw_tolerance_deg = 5.0;
};

enum class MissionExecutorState
{
  WAIT_ODOM = 0,
  IDLE = 1,
  SEND_GOAL = 2,
  WAIT_POSITION = 3,
  ALIGN_YAW = 4,
  CAPTURE = 5,
  FINISHED = 6,
  ERROR = 7
};

inline std::string missionStateToString(const MissionExecutorState state)
{
  switch (state)
  {
    case MissionExecutorState::WAIT_ODOM:
      return "WAIT_ODOM";
    case MissionExecutorState::IDLE:
      return "IDLE";
    case MissionExecutorState::SEND_GOAL:
      return "SEND_GOAL";
    case MissionExecutorState::WAIT_POSITION:
      return "WAIT_POSITION";
    case MissionExecutorState::ALIGN_YAW:
      return "ALIGN_YAW";
    case MissionExecutorState::CAPTURE:
      return "CAPTURE";
    case MissionExecutorState::FINISHED:
      return "FINISHED";
    case MissionExecutorState::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

}  // namespace scope