#include "scope/common/parameters.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <iostream>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace scope
{

namespace
{

std::string toLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

template <typename T>
T readScalar(const YAML::Node& node, const std::string& key, const T& default_value)
{
  if (!node || !node[key])
  {
    return default_value;
  }

  try
  {
    return node[key].as<T>();
  }
  catch (const std::exception& e)
  {
    std::cerr << "[SCOPE][Parameters] Failed to read key '" << key
              << "'. Use default value. Error: " << e.what() << std::endl;
    return default_value;
  }
}

Eigen::Vector3d readVector3d(const YAML::Node& node,
                             const std::string& key,
                             const Eigen::Vector3d& default_value)
{
  if (!node || !node[key])
  {
    return default_value;
  }

  const YAML::Node value = node[key];

  if (!value.IsSequence() || value.size() != 3)
  {
    std::cerr << "[SCOPE][Parameters] Key '" << key
              << "' should be a sequence with 3 values. Use default value." << std::endl;
    return default_value;
  }

  try
  {
    return Eigen::Vector3d(value[0].as<double>(),
                           value[1].as<double>(),
                           value[2].as<double>());
  }
  catch (const std::exception& e)
  {
    std::cerr << "[SCOPE][Parameters] Failed to read vector key '" << key
              << "'. Use default value. Error: " << e.what() << std::endl;
    return default_value;
  }
}

std::vector<double> readDoubleVector(const YAML::Node& node,
                                     const std::string& key,
                                     const std::vector<double>& default_value)
{
  if (!node || !node[key])
  {
    return default_value;
  }

  const YAML::Node value = node[key];

  if (!value.IsSequence())
  {
    std::cerr << "[SCOPE][Parameters] Key '" << key
              << "' should be a sequence. Use default value." << std::endl;
    return default_value;
  }

  std::vector<double> result;
  result.reserve(value.size());

  try
  {
    for (std::size_t i = 0; i < value.size(); ++i)
    {
      result.push_back(value[i].as<double>());
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "[SCOPE][Parameters] Failed to read double vector key '" << key
              << "'. Use default value. Error: " << e.what() << std::endl;
    return default_value;
  }

  return result;
}

std::string vectorToString(const Eigen::Vector3d& v)
{
  std::ostringstream oss;
  oss << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
  return oss.str();
}

std::string doubleVectorToString(const std::vector<double>& values)
{
  std::ostringstream oss;
  oss << "[";

  for (std::size_t i = 0; i < values.size(); ++i)
  {
    oss << values[i];
    if (i + 1 < values.size())
    {
      oss << ", ";
    }
  }

  oss << "]";
  return oss.str();
}

}  // namespace

PitchMode parsePitchMode(const std::string& mode)
{
  const std::string m = toLower(mode);

  if (m == "fixed_mount")
  {
    return PitchMode::FIXED_MOUNT;
  }

  if (m == "virtual_camera")
  {
    return PitchMode::VIRTUAL_CAMERA;
  }

  if (m == "gimbal")
  {
    return PitchMode::GIMBAL;
  }

  if (m == "body_attitude")
  {
    return PitchMode::BODY_ATTITUDE;
  }

  std::cerr << "[SCOPE][Parameters] Unknown pitch_mode: " << mode
            << ". Use fixed_mount." << std::endl;
  return PitchMode::FIXED_MOUNT;
}

SensorMode parseSensorMode(const std::string& mode)
{
  const std::string m = toLower(mode);

  if (m == "camera")
  {
    return SensorMode::CAMERA;
  }

  if (m == "lidar")
  {
    return SensorMode::LIDAR;
  }

  if (m == "camera_lidar")
  {
    return SensorMode::CAMERA_LIDAR;
  }

  std::cerr << "[SCOPE][Parameters] Unknown sensor_mode: " << mode
            << ". Use camera." << std::endl;
  return SensorMode::CAMERA;
}

bool ScopeParameters::loadFromYamlFile(const std::string& yaml_path)
{
  YAML::Node root;

  try
  {
    root = YAML::LoadFile(yaml_path);
  }
  catch (const std::exception& e)
  {
    std::cerr << "[SCOPE][Parameters] Failed to load yaml file: " << yaml_path
              << ". Error: " << e.what() << std::endl;
    return false;
  }

  const YAML::Node scope_node = root["scope"];
  scope.mode = readScalar<std::string>(scope_node, "mode", scope.mode);
  scope.world_frame = readScalar<std::string>(scope_node, "world_frame", scope.world_frame);

  const YAML::Node input_node = root["input"];
  input.use_cloud = readScalar<bool>(input_node, "use_cloud", input.use_cloud);
  input.use_mesh = readScalar<bool>(input_node, "use_mesh", input.use_mesh);
  input.cloud_path = readScalar<std::string>(input_node, "cloud_path", input.cloud_path);
  input.mesh_path = readScalar<std::string>(input_node, "mesh_path", input.mesh_path);

  const YAML::Node output_node = root["output"];
  output.waypoint_yaml = readScalar<std::string>(output_node, "waypoint_yaml", output.waypoint_yaml);
  output.waypoint_csv = readScalar<std::string>(output_node, "waypoint_csv", output.waypoint_csv);
  output.coverage_report = readScalar<std::string>(output_node, "coverage_report", output.coverage_report);

  const YAML::Node preprocess_node = root["preprocess"];
  preprocess.voxel_leaf_size =
      readScalar<double>(preprocess_node, "voxel_leaf_size", preprocess.voxel_leaf_size);

  preprocess.enable_roi_filter =
      readScalar<bool>(preprocess_node, "enable_roi_filter", preprocess.enable_roi_filter);

  preprocess.roi_box.min =
      readVector3d(preprocess_node, "roi_min", preprocess.roi_box.min);
  preprocess.roi_box.max =
      readVector3d(preprocess_node, "roi_max", preprocess.roi_box.max);

  preprocess.enable_outlier_removal =
      readScalar<bool>(preprocess_node, "enable_outlier_removal", preprocess.enable_outlier_removal);
  preprocess.mean_k =
      readScalar<int>(preprocess_node, "mean_k", preprocess.mean_k);
  preprocess.stddev_mul_thresh =
      readScalar<double>(preprocess_node, "stddev_mul_thresh", preprocess.stddev_mul_thresh);

  const YAML::Node normal_node = root["normal"];
  normal.search_radius =
      readScalar<double>(normal_node, "search_radius", normal.search_radius);
  normal.k_search =
      readScalar<int>(normal_node, "k_search", normal.k_search);
  normal.flip_towards_origin =
      readScalar<bool>(normal_node, "flip_towards_origin", normal.flip_towards_origin);

  const YAML::Node sensor_node = root["sensor"];
  const YAML::Node camera_node = sensor_node["camera"];
  sensor.camera.enable =
      readScalar<bool>(camera_node, "enable", sensor.camera.enable);
  sensor.camera.fov_h_deg =
      readScalar<double>(camera_node, "fov_h_deg", sensor.camera.fov_h_deg);
  sensor.camera.fov_v_deg =
      readScalar<double>(camera_node, "fov_v_deg", sensor.camera.fov_v_deg);
  sensor.camera.min_view_distance =
      readScalar<double>(camera_node, "min_view_distance", sensor.camera.min_view_distance);
  sensor.camera.max_view_distance =
      readScalar<double>(camera_node, "max_view_distance", sensor.camera.max_view_distance);
  sensor.camera.fixed_mount_pitch_deg =
      readScalar<double>(camera_node, "fixed_mount_pitch_deg", sensor.camera.fixed_mount_pitch_deg);

  const YAML::Node lidar_node = sensor_node["lidar"];
  sensor.lidar.enable =
      readScalar<bool>(lidar_node, "enable", sensor.lidar.enable);
  sensor.lidar.fov_h_deg =
      readScalar<double>(lidar_node, "fov_h_deg", sensor.lidar.fov_h_deg);
  sensor.lidar.fov_v_deg =
      readScalar<double>(lidar_node, "fov_v_deg", sensor.lidar.fov_v_deg);
  sensor.lidar.min_range =
      readScalar<double>(lidar_node, "min_range", sensor.lidar.min_range);
  sensor.lidar.max_range =
      readScalar<double>(lidar_node, "max_range", sensor.lidar.max_range);

  const YAML::Node uav_node = root["uav"];
  uav.body_radius =
      readScalar<double>(uav_node, "body_radius", uav.body_radius);
  uav.localization_error =
      readScalar<double>(uav_node, "localization_error", uav.localization_error);
  uav.tracking_error =
      readScalar<double>(uav_node, "tracking_error", uav.tracking_error);
  uav.safety_margin =
      readScalar<double>(uav_node, "safety_margin", uav.safety_margin);

  const YAML::Node viewpoint_node = root["viewpoint"];
  viewpoint.surface_sampling_resolution =
      readScalar<double>(viewpoint_node, "surface_sampling_resolution",
                         viewpoint.surface_sampling_resolution);

  viewpoint.candidate_distances =
      readDoubleVector(viewpoint_node, "candidate_distances", viewpoint.candidate_distances);

  viewpoint.max_candidate_num =
      readScalar<int>(viewpoint_node, "max_candidate_num", viewpoint.max_candidate_num);
  viewpoint.min_height =
      readScalar<double>(viewpoint_node, "min_height", viewpoint.min_height);
  viewpoint.max_height =
      readScalar<double>(viewpoint_node, "max_height", viewpoint.max_height);

  viewpoint.yaw_planning =
      readScalar<bool>(viewpoint_node, "yaw_planning", viewpoint.yaw_planning);
  viewpoint.enable_pitch_planning =
      readScalar<bool>(viewpoint_node, "enable_pitch_planning",
                       viewpoint.enable_pitch_planning);

  const std::string pitch_mode_str =
      readScalar<std::string>(viewpoint_node, "pitch_mode",
                              pitchModeToString(viewpoint.pitch_mode));
  viewpoint.pitch_mode = parsePitchMode(pitch_mode_str);

  viewpoint.pitch_lower_deg =
      readScalar<double>(viewpoint_node, "pitch_lower_deg", viewpoint.pitch_lower_deg);
  viewpoint.pitch_upper_deg =
      readScalar<double>(viewpoint_node, "pitch_upper_deg", viewpoint.pitch_upper_deg);
  viewpoint.output_pitch =
      readScalar<bool>(viewpoint_node, "output_pitch", viewpoint.output_pitch);

  const YAML::Node target_surface_node = root["target_surface"];

  target_surface.enable_curvature_filter =
      readScalar<bool>(target_surface_node,
                       "enable_curvature_filter",
                       target_surface.enable_curvature_filter);

  target_surface.max_curvature =
      readScalar<double>(target_surface_node,
                         "max_curvature",
                         target_surface.max_curvature);

  target_surface.default_area =
      readScalar<double>(target_surface_node,
                         "default_area",
                         target_surface.default_area);

  target_surface.default_weight =
      readScalar<double>(target_surface_node,
                         "default_weight",
                         target_surface.default_weight);

  target_surface.use_area_weight =
      readScalar<bool>(target_surface_node,
                       "use_area_weight",
                       target_surface.use_area_weight);
  
  const YAML::Node coverage_node = root["coverage"];
  coverage.target_coverage_ratio =
      readScalar<double>(coverage_node, "target_coverage_ratio",
                         coverage.target_coverage_ratio);
  coverage.min_view_redundancy =
      readScalar<int>(coverage_node, "min_view_redundancy",
                      coverage.min_view_redundancy);
  coverage.max_selected_viewpoint_num =
      readScalar<int>(coverage_node, "max_selected_viewpoint_num",
                      coverage.max_selected_viewpoint_num);
  coverage.min_new_covered_points =
      readScalar<int>(coverage_node, "min_new_covered_points",
                      coverage.min_new_covered_points);

  const YAML::Node safety_node = root["safety"];
  safety.enable_safety_filter =
      readScalar<bool>(safety_node, "enable_safety_filter",
                       safety.enable_safety_filter);
  safety.unknown_as_obstacle =
      readScalar<bool>(safety_node, "unknown_as_obstacle",
                       safety.unknown_as_obstacle);
  safety.min_clearance =
      readScalar<double>(safety_node, "min_clearance", safety.min_clearance);
  safety.enable_segment_check =
      readScalar<bool>(safety_node, "enable_segment_check",
                       safety.enable_segment_check);
  safety.segment_check_resolution =
      readScalar<double>(safety_node, "segment_check_resolution",
                         safety.segment_check_resolution);

  const YAML::Node optimization_node = root["optimization"];
  optimization.route_method =
      readScalar<std::string>(optimization_node, "route_method",
                              optimization.route_method);
  optimization.start_position =
      readVector3d(optimization_node, "start_position",
                   optimization.start_position);
  optimization.weight_distance =
      readScalar<double>(optimization_node, "weight_distance",
                         optimization.weight_distance);
  optimization.weight_yaw =
      readScalar<double>(optimization_node, "weight_yaw",
                         optimization.weight_yaw);
  optimization.weight_height =
      readScalar<double>(optimization_node, "weight_height",
                         optimization.weight_height);

  const YAML::Node visualization_node = root["visualization"];
  visualization.publish_markers =
      readScalar<bool>(visualization_node, "publish_markers",
                       visualization.publish_markers);
  visualization.marker_topic =
      readScalar<std::string>(visualization_node, "marker_topic",
                              visualization.marker_topic);
  visualization.raw_cloud_topic =
      readScalar<std::string>(visualization_node, "raw_cloud_topic",
                              visualization.raw_cloud_topic);
  visualization.processed_cloud_topic =
      readScalar<std::string>(visualization_node, "processed_cloud_topic",
                              visualization.processed_cloud_topic);
  visualization.waypoint_topic =
      readScalar<std::string>(visualization_node, "waypoint_topic",
                              visualization.waypoint_topic);

  std::string error_message;
  if (!validate(&error_message))
  {
    std::cerr << "[SCOPE][Parameters] Invalid parameter file: " << yaml_path
              << std::endl;
    std::cerr << "[SCOPE][Parameters] " << error_message << std::endl;
    return false;
  }

  return true;
}

bool ScopeParameters::validate(std::string* error_message) const
{
  auto setError = [error_message](const std::string& msg) {
    if (error_message)
    {
      *error_message = msg;
    }
  };

  if (scope.world_frame.empty())
  {
    setError("scope.world_frame is empty.");
    return false;
  }

  if (!input.use_cloud && !input.use_mesh)
  {
    setError("Both input.use_cloud and input.use_mesh are false. At least one input source is required.");
    return false;
  }

  if (input.use_cloud && input.cloud_path.empty())
  {
    setError("input.use_cloud is true, but input.cloud_path is empty.");
    return false;
  }

  if (input.use_mesh && input.mesh_path.empty())
  {
    setError("input.use_mesh is true, but input.mesh_path is empty.");
    return false;
  }

  if (preprocess.voxel_leaf_size <= 0.0)
  {
    setError("preprocess.voxel_leaf_size must be positive.");
    return false;
  }

  if (preprocess.enable_roi_filter && !preprocess.roi_box.isValid())
  {
    setError("preprocess ROI box is invalid. roi_min should be smaller than roi_max.");
    return false;
  }

  if (preprocess.mean_k <= 0)
  {
    setError("preprocess.mean_k must be positive.");
    return false;
  }

  if (preprocess.stddev_mul_thresh <= 0.0)
  {
    setError("preprocess.stddev_mul_thresh must be positive.");
    return false;
  }

  if (normal.search_radius <= 0.0)
  {
    setError("normal.search_radius must be positive.");
    return false;
  }

  if (normal.k_search <= 0)
  {
    setError("normal.k_search must be positive.");
    return false;
  }

  if (sensor.camera.enable)
  {
    if (sensor.camera.fov_h_deg <= 0.0 || sensor.camera.fov_h_deg >= 180.0)
    {
      setError("sensor.camera.fov_h_deg should be in (0, 180).");
      return false;
    }

    if (sensor.camera.fov_v_deg <= 0.0 || sensor.camera.fov_v_deg >= 180.0)
    {
      setError("sensor.camera.fov_v_deg should be in (0, 180).");
      return false;
    }

    if (sensor.camera.min_view_distance < 0.0 ||
        sensor.camera.max_view_distance <= sensor.camera.min_view_distance)
    {
      setError("Invalid camera view distance range.");
      return false;
    }
  }

  if (sensor.lidar.enable)
  {
    if (sensor.lidar.min_range < 0.0 ||
        sensor.lidar.max_range <= sensor.lidar.min_range)
    {
      setError("Invalid lidar range.");
      return false;
    }
  }

  if (uav.body_radius < 0.0 ||
      uav.localization_error < 0.0 ||
      uav.tracking_error < 0.0 ||
      uav.safety_margin < 0.0)
  {
    setError("UAV radius and safety terms should be non-negative.");
    return false;
  }

  if (viewpoint.surface_sampling_resolution <= 0.0)
  {
    setError("viewpoint.surface_sampling_resolution must be positive.");
    return false;
  }

  if (viewpoint.candidate_distances.empty())
  {
    setError("viewpoint.candidate_distances is empty.");
    return false;
  }

  for (const double d : viewpoint.candidate_distances)
  {
    if (d <= 0.0)
    {
      setError("All viewpoint.candidate_distances should be positive.");
      return false;
    }
  }

  if (viewpoint.max_candidate_num <= 0)
  {
    setError("viewpoint.max_candidate_num must be positive.");
    return false;
  }

  if (viewpoint.min_height > viewpoint.max_height)
  {
    setError("viewpoint.min_height should not be larger than viewpoint.max_height.");
    return false;
  }

  if (viewpoint.pitch_lower_deg > viewpoint.pitch_upper_deg)
  {
    setError("viewpoint.pitch_lower_deg should not be larger than viewpoint.pitch_upper_deg.");
    return false;
  }

  if (target_surface.max_curvature < 0.0)
  {
    setError("target_surface.max_curvature should be non-negative.");
    return false;
  }

  if (target_surface.default_area <= 0.0)
  {
    setError("target_surface.default_area must be positive.");
    return false;
  }

  if (target_surface.default_weight <= 0.0)
  {
    setError("target_surface.default_weight must be positive.");
    return false;
  }

  if (coverage.target_coverage_ratio <= 0.0 || coverage.target_coverage_ratio > 1.0)
  {
    setError("coverage.target_coverage_ratio should be in (0, 1].");
    return false;
  }

  if (coverage.min_view_redundancy <= 0)
  {
    setError("coverage.min_view_redundancy must be positive.");
    return false;
  }

  if (coverage.max_selected_viewpoint_num <= 0)
  {
    setError("coverage.max_selected_viewpoint_num must be positive.");
    return false;
  }

  if (coverage.min_new_covered_points <= 0)
  {
    setError("coverage.min_new_covered_points must be positive.");
    return false;
  }

  if (safety.min_clearance < 0.0)
  {
    setError("safety.min_clearance should be non-negative.");
    return false;
  }

  if (safety.segment_check_resolution <= 0.0)
  {
    setError("safety.segment_check_resolution must be positive.");
    return false;
  }

  return true;
}

void ScopeParameters::printSummary() const
{
  std::cout << "\n";
  std::cout << "================ SCOPE Parameter Summary ================\n";

  std::cout << "[scope]\n";
  std::cout << "  mode: " << scope.mode << "\n";
  std::cout << "  world_frame: " << scope.world_frame << "\n";

  std::cout << "[input]\n";
  std::cout << "  use_cloud: " << std::boolalpha << input.use_cloud << "\n";
  std::cout << "  cloud_path: " << input.cloud_path << "\n";
  std::cout << "  use_mesh: " << input.use_mesh << "\n";
  std::cout << "  mesh_path: " << input.mesh_path << "\n";

  std::cout << "[output]\n";
  std::cout << "  waypoint_yaml: " << output.waypoint_yaml << "\n";
  std::cout << "  waypoint_csv: " << output.waypoint_csv << "\n";
  std::cout << "  coverage_report: " << output.coverage_report << "\n";

  std::cout << "[preprocess]\n";
  std::cout << "  voxel_leaf_size: " << preprocess.voxel_leaf_size << "\n";
  std::cout << "  enable_roi_filter: " << preprocess.enable_roi_filter << "\n";
  std::cout << "  roi_min: " << vectorToString(preprocess.roi_box.min) << "\n";
  std::cout << "  roi_max: " << vectorToString(preprocess.roi_box.max) << "\n";
  std::cout << "  enable_outlier_removal: " << preprocess.enable_outlier_removal << "\n";
  std::cout << "  mean_k: " << preprocess.mean_k << "\n";
  std::cout << "  stddev_mul_thresh: " << preprocess.stddev_mul_thresh << "\n";

  std::cout << "[normal]\n";
  std::cout << "  search_radius: " << normal.search_radius << "\n";
  std::cout << "  k_search: " << normal.k_search << "\n";
  std::cout << "  flip_towards_origin: " << normal.flip_towards_origin << "\n";

  std::cout << "[sensor.camera]\n";
  std::cout << "  enable: " << sensor.camera.enable << "\n";
  std::cout << "  fov_h_deg: " << sensor.camera.fov_h_deg << "\n";
  std::cout << "  fov_v_deg: " << sensor.camera.fov_v_deg << "\n";
  std::cout << "  min_view_distance: " << sensor.camera.min_view_distance << "\n";
  std::cout << "  max_view_distance: " << sensor.camera.max_view_distance << "\n";
  std::cout << "  fixed_mount_pitch_deg: " << sensor.camera.fixed_mount_pitch_deg << "\n";

  std::cout << "[sensor.lidar]\n";
  std::cout << "  enable: " << sensor.lidar.enable << "\n";
  std::cout << "  fov_h_deg: " << sensor.lidar.fov_h_deg << "\n";
  std::cout << "  fov_v_deg: " << sensor.lidar.fov_v_deg << "\n";
  std::cout << "  min_range: " << sensor.lidar.min_range << "\n";
  std::cout << "  max_range: " << sensor.lidar.max_range << "\n";

  std::cout << "[uav]\n";
  std::cout << "  body_radius: " << uav.body_radius << "\n";
  std::cout << "  localization_error: " << uav.localization_error << "\n";
  std::cout << "  tracking_error: " << uav.tracking_error << "\n";
  std::cout << "  safety_margin: " << uav.safety_margin << "\n";
  std::cout << "  safe_radius: " << uav.safeRadius() << "\n";

  std::cout << "[viewpoint]\n";
  std::cout << "  surface_sampling_resolution: " << viewpoint.surface_sampling_resolution << "\n";
  std::cout << "  candidate_distances: " << doubleVectorToString(viewpoint.candidate_distances) << "\n";
  std::cout << "  max_candidate_num: " << viewpoint.max_candidate_num << "\n";
  std::cout << "  min_height: " << viewpoint.min_height << "\n";
  std::cout << "  max_height: " << viewpoint.max_height << "\n";
  std::cout << "  yaw_planning: " << viewpoint.yaw_planning << "\n";
  std::cout << "  enable_pitch_planning: " << viewpoint.enable_pitch_planning << "\n";
  std::cout << "  pitch_mode: " << pitchModeToString(viewpoint.pitch_mode) << "\n";
  std::cout << "  pitch_lower_deg: " << viewpoint.pitch_lower_deg << "\n";
  std::cout << "  pitch_upper_deg: " << viewpoint.pitch_upper_deg << "\n";
  std::cout << "  output_pitch: " << viewpoint.output_pitch << "\n";

  std::cout << "[target_surface]\n";
  std::cout << "  enable_curvature_filter: "
            << target_surface.enable_curvature_filter << "\n";
  std::cout << "  max_curvature: "
            << target_surface.max_curvature << "\n";
  std::cout << "  default_area: "
            << target_surface.default_area << "\n";
  std::cout << "  default_weight: "
            << target_surface.default_weight << "\n";
  std::cout << "  use_area_weight: "
            << target_surface.use_area_weight << "\n";

  std::cout << "[coverage]\n";
  std::cout << "  target_coverage_ratio: " << coverage.target_coverage_ratio << "\n";
  std::cout << "  min_view_redundancy: " << coverage.min_view_redundancy << "\n";
  std::cout << "  max_selected_viewpoint_num: " << coverage.max_selected_viewpoint_num << "\n";
  std::cout << "  min_new_covered_points: " << coverage.min_new_covered_points << "\n";

  std::cout << "[safety]\n";
  std::cout << "  enable_safety_filter: " << safety.enable_safety_filter << "\n";
  std::cout << "  unknown_as_obstacle: " << safety.unknown_as_obstacle << "\n";
  std::cout << "  min_clearance: " << safety.min_clearance << "\n";
  std::cout << "  enable_segment_check: " << safety.enable_segment_check << "\n";
  std::cout << "  segment_check_resolution: " << safety.segment_check_resolution << "\n";

  std::cout << "[optimization]\n";
  std::cout << "  route_method: " << optimization.route_method << "\n";
  std::cout << "  start_position: " << vectorToString(optimization.start_position) << "\n";
  std::cout << "  weight_distance: " << optimization.weight_distance << "\n";
  std::cout << "  weight_yaw: " << optimization.weight_yaw << "\n";
  std::cout << "  weight_height: " << optimization.weight_height << "\n";

  std::cout << "[visualization]\n";
  std::cout << "  publish_markers: " << visualization.publish_markers << "\n";
  std::cout << "  marker_topic: " << visualization.marker_topic << "\n";
  std::cout << "  raw_cloud_topic: " << visualization.raw_cloud_topic << "\n";
  std::cout << "  processed_cloud_topic: " << visualization.processed_cloud_topic << "\n";
  std::cout << "  waypoint_topic: " << visualization.waypoint_topic << "\n";

  std::cout << "=========================================================\n";
  std::cout << "\n";
}

}  // namespace scope