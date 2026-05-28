#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "scope/common/types.hpp"

namespace scope
{

// ================================================================
// Parameter groups
// ================================================================

struct ScopeGeneralParams
{
  std::string mode = "offline";
  std::string world_frame = "camera_init";
};

struct InputParams
{
  bool use_cloud = true;
  bool use_mesh = false;

  std::string cloud_path;
  std::string mesh_path;
};

struct OutputParams
{
  std::string waypoint_yaml = "selected_waypoints.yaml";
  std::string waypoint_csv = "selected_waypoints.csv";
  std::string coverage_report = "coverage_report.json";
};

struct PreprocessParams
{
  double voxel_leaf_size = 0.05;

  bool enable_roi_filter = false;
  AxisAlignedBox roi_box;

  bool enable_outlier_removal = true;
  int mean_k = 30;
  double stddev_mul_thresh = 1.0;
};

struct NormalParams
{
  double search_radius = 0.20;
  int k_search = 30;
  bool flip_towards_origin = false;
};

struct CameraParams
{
  bool enable = true;

  double fov_h_deg = 90.0;
  double fov_v_deg = 60.0;

  double min_view_distance = 0.8;
  double max_view_distance = 4.0;

  double fixed_mount_pitch_deg = -20.0;
};

struct LidarParams
{
  bool enable = false;

  double fov_h_deg = 360.0;
  double fov_v_deg = 59.0;

  double min_range = 0.3;
  double max_range = 20.0;
};

struct SensorParams
{
  CameraParams camera;
  LidarParams lidar;
};

struct UavParams
{
  double body_radius = 0.35;
  double localization_error = 0.10;
  double tracking_error = 0.10;
  double safety_margin = 0.15;

  double safeRadius() const
  {
    return body_radius + localization_error + tracking_error + safety_margin;
  }
};

struct ViewpointParams
{
  double surface_sampling_resolution = 0.30;

  std::vector<double> candidate_distances = {1.0, 1.5, 2.0};

  int max_candidate_num = 5000;

  double min_height = -100.0;
  double max_height = 100.0;

  bool yaw_planning = true;

  bool enable_pitch_planning = false;
  PitchMode pitch_mode = PitchMode::FIXED_MOUNT;

  double pitch_lower_deg = -60.0;
  double pitch_upper_deg = 20.0;

  bool output_pitch = false;
};

struct CoverageParams
{
  double target_coverage_ratio = 0.90;

  int min_view_redundancy = 1;
  int max_selected_viewpoint_num = 300;
  int min_new_covered_points = 10;
};

struct SafetyParams
{
  bool enable_safety_filter = true;

  bool unknown_as_obstacle = true;

  double min_clearance = 0.70;

  bool enable_segment_check = true;
  double segment_check_resolution = 0.10;
};

struct OptimizationParams
{
  std::string route_method = "greedy_tsp";

  Eigen::Vector3d start_position = Eigen::Vector3d::Zero();

  double weight_distance = 1.0;
  double weight_yaw = 0.2;
  double weight_height = 0.2;
};

struct VisualizationParams
{
  bool publish_markers = true;

  std::string marker_topic = "/scope/markers";
  std::string raw_cloud_topic = "/scope/raw_cloud";
  std::string processed_cloud_topic = "/scope/processed_cloud";
  std::string waypoint_topic = "/scope/waypoints";
};

// ================================================================
// Global parameter container
// ================================================================

struct ScopeParameters
{
  ScopeGeneralParams scope;

  InputParams input;
  OutputParams output;

  PreprocessParams preprocess;
  NormalParams normal;

  SensorParams sensor;
  UavParams uav;

  ViewpointParams viewpoint;
  CoverageParams coverage;
  SafetyParams safety;
  OptimizationParams optimization;
  VisualizationParams visualization;

  bool loadFromYamlFile(const std::string& yaml_path);
  bool validate(std::string* error_message = nullptr) const;
  void printSummary() const;
};

// ================================================================
// Helper function declarations
// ================================================================

PitchMode parsePitchMode(const std::string& mode);
std::string pitchModeToString(const PitchMode mode);

SensorMode parseSensorMode(const std::string& mode);
std::string sensorModeToString(const SensorMode mode);

}  // namespace scope