#include <ros/ros.h>
#include <ros/package.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>

#include "scope/common/parameters.hpp"
#include "scope/common/types.hpp"
#include "scope/io/pcd_loader.hpp"
#include "scope/map/cloud_preprocessor.hpp"
#include "scope/map/normal_estimator.hpp"
#include "scope/map/target_surface.hpp"
#include "scope/viewpoint/candidate_generator.hpp"
#include "scope/safety/safety_filter.hpp"
#include "scope/io/waypoint_io.hpp"
#include "scope/viewpoint/coverage_evaluator.hpp"
#include "scope/viewpoint/viewpoint_selector.hpp"

namespace
{

std::string vecToString(const Eigen::Vector3d& v)
{
  std::ostringstream oss;
  oss << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
  return oss.str();
}

std::string statsToString(const scope::CloudStatistics& stats)
{
  std::ostringstream oss;

  if (!stats.valid)
  {
    oss << "invalid statistics";
    return oss.str();
  }

  oss << "points=" << stats.num_points
      << ", min=" << vecToString(stats.min_bound)
      << ", max=" << vecToString(stats.max_bound)
      << ", centroid=" << vecToString(stats.centroid);

  return oss.str();
}

std::string resolvePackageRelativePath(const std::string& path,
                                       const std::string& package_name)
{
  if (path.empty())
  {
    return path;
  }

  // Absolute path. Use directly.
  if (path.front() == '/')
  {
    return path;
  }

  const std::string package_path = ros::package::getPath(package_name);

  if (package_path.empty())
  {
    ROS_WARN_STREAM("[SCOPE] Failed to locate ROS package: "
                    << package_name
                    << ". Use output_dir as a relative path: "
                    << path);
    return path;
  }

  return package_path + "/" + path;
}

sensor_msgs::PointCloud2 toRosCloudMsg(const scope::PointCloudConstPtr& cloud,
                                       const std::string& frame_id)
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = frame_id;
  msg.header.stamp = ros::Time::now();
  return msg;
}

sensor_msgs::PointCloud2 toRosNormalCloudMsg(const scope::PointNormalCloudConstPtr& cloud,
                                             const std::string& frame_id)
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = frame_id;
  msg.header.stamp = ros::Time::now();
  return msg;
}

visualization_msgs::MarkerArray createNormalMarkers(
    const scope::PointNormalCloudConstPtr& normal_cloud,
    const std::string& frame_id,
    const double normal_length,
    const std::size_t max_marker_lines)
{
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "scope_normals";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.01;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  if (!normal_cloud || normal_cloud->empty())
  {
    marker_array.markers.push_back(marker);
    return marker_array;
  }

  const std::size_t stride =
      std::max<std::size_t>(1, normal_cloud->size() / std::max<std::size_t>(1, max_marker_lines));

  for (std::size_t i = 0; i < normal_cloud->size(); i += stride)
  {
    const auto& p = normal_cloud->points[i];

    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(p.normal_x) ||
        !std::isfinite(p.normal_y) ||
        !std::isfinite(p.normal_z))
    {
      continue;
    }

    geometry_msgs::Point p0;
    p0.x = p.x;
    p0.y = p.y;
    p0.z = p.z;

    geometry_msgs::Point p1;
    p1.x = p.x + normal_length * p.normal_x;
    p1.y = p.y + normal_length * p.normal_y;
    p1.z = p.z + normal_length * p.normal_z;

    marker.points.push_back(p0);
    marker.points.push_back(p1);
  }

  marker_array.markers.push_back(marker);
  return marker_array;
}

std::size_t computeGlobalVisualizationStride(const std::size_t total_count,
                                             const std::size_t max_marker_lines)
{
  if (total_count == 0 || max_marker_lines == 0)
  {
    return 1;
  }

  if (total_count <= max_marker_lines)
  {
    return 1;
  }

  return static_cast<std::size_t>(
      std::ceil(static_cast<double>(total_count) /
                static_cast<double>(max_marker_lines)));
}

visualization_msgs::MarkerArray createViewpointMarkers(
    const std::vector<scope::ViewpointCandidate>& candidates,
    const std::string& frame_id,
    const std::string& ns_prefix,
    const double line_width,
    const double point_scale,
    const std::size_t global_id_stride,
    const float point_r,
    const float point_g,
    const float point_b,
    const float point_a,
    const float line_r,
    const float line_g,
    const float line_b,
    const float line_a)
{
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = frame_id;
  line_marker.header.stamp = ros::Time::now();
  line_marker.ns = ns_prefix + "_view_rays";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.scale.x = line_width;

  line_marker.color.r = line_r;
  line_marker.color.g = line_g;
  line_marker.color.b = line_b;
  line_marker.color.a = line_a;

  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = frame_id;
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = ns_prefix + "_positions";
  point_marker.id = 1;
  point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;

  point_marker.scale.x = point_scale;
  point_marker.scale.y = point_scale;
  point_marker.scale.z = point_scale;

  point_marker.color.r = point_r;
  point_marker.color.g = point_g;
  point_marker.color.b = point_b;
  point_marker.color.a = point_a;

  const std::size_t id_stride = std::max<std::size_t>(1, global_id_stride);

  for (const auto& candidate : candidates)
  {
    if (!candidate.is_valid)
    {
      continue;
    }

    // Use the original global candidate id for visualization sampling.
    // This guarantees that all/safe/unsafe markers are sampled from
    // the same global candidate set.
    if (candidate.id % id_stride != 0)
    {
      continue;
    }

    geometry_msgs::Point p0;
    p0.x = candidate.position.x();
    p0.y = candidate.position.y();
    p0.z = candidate.position.z();

    geometry_msgs::Point p1;
    p1.x = candidate.target_point.x();
    p1.y = candidate.target_point.y();
    p1.z = candidate.target_point.z();

    line_marker.points.push_back(p0);
    line_marker.points.push_back(p1);

    point_marker.points.push_back(p0);
  }

  marker_array.markers.push_back(line_marker);
  marker_array.markers.push_back(point_marker);

  return marker_array;
}

}  // namespace

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

  if (!params.input.use_cloud)
  {
    ROS_ERROR_STREAM("[SCOPE] V0.4.0 currently requires input.use_cloud = true.");
    return 1;
  }

  ROS_INFO_STREAM("[SCOPE] Loading PCD: " << params.input.cloud_path);

  const scope::PcdLoadResult load_result =
      scope::PcdLoader::loadCloud(params.input.cloud_path);

  if (!load_result.success)
  {
    ROS_ERROR_STREAM("[SCOPE] Failed to load PCD. " << load_result.message);
    return 1;
  }

  scope::PointCloudPtr raw_cloud = load_result.cloud;

  ROS_INFO_STREAM("[SCOPE] " << load_result.message);
  ROS_INFO_STREAM("[SCOPE] Raw cloud statistics: "
                  << statsToString(load_result.statistics));

  scope::CloudPreprocessor preprocessor(params.preprocess);

  ROS_INFO_STREAM("[SCOPE] Start cloud preprocessing.");

  const scope::CloudPreprocessResult preprocess_result =
      preprocessor.process(raw_cloud);

  if (!preprocess_result.success)
  {
    ROS_ERROR_STREAM("[SCOPE] Cloud preprocessing failed. "
                     << preprocess_result.message);
    return 1;
  }

  scope::PointCloudPtr processed_cloud = preprocess_result.cloud;

  ROS_INFO_STREAM("[SCOPE] " << preprocess_result.message);
  ROS_INFO_STREAM("[SCOPE] Processed cloud statistics: "
                  << statsToString(preprocess_result.statistics));

  ROS_INFO_STREAM("[SCOPE] Preprocess point count summary:");
  ROS_INFO_STREAM("[SCOPE]   input:                 "
                  << preprocess_result.input_points);
  ROS_INFO_STREAM("[SCOPE]   after invalid removal: "
                  << preprocess_result.after_invalid_removal);
  ROS_INFO_STREAM("[SCOPE]   after voxel filter:    "
                  << preprocess_result.after_voxel_filter);
  ROS_INFO_STREAM("[SCOPE]   after ROI filter:      "
                  << preprocess_result.after_roi_filter);
  ROS_INFO_STREAM("[SCOPE]   after outlier removal: "
                  << preprocess_result.after_outlier_removal);

  // --------------------------------------------------------------------------
  // Normal estimation
  // --------------------------------------------------------------------------
  ROS_INFO_STREAM("[SCOPE] Start normal estimation.");

  scope::NormalEstimator normal_estimator(params.normal);

  const scope::NormalEstimationResult normal_result =
      normal_estimator.estimate(processed_cloud);

  if (!normal_result.success)
  {
    ROS_ERROR_STREAM("[SCOPE] Normal estimation failed. "
                     << normal_result.message);
    return 1;
  }

  scope::PointNormalCloudPtr normal_cloud = normal_result.cloud;

  ROS_INFO_STREAM("[SCOPE] " << normal_result.message);

  // --------------------------------------------------------------------------
  // Target surface construction
  // --------------------------------------------------------------------------
  ROS_INFO_STREAM("[SCOPE] Start target surface construction.");

  scope::TargetSurfaceBuilder target_surface_builder(params.target_surface);

  const scope::TargetSurfaceBuildResult target_surface_result =
      target_surface_builder.buildFromNormalCloud(normal_cloud);

  if (!target_surface_result.success)
  {
    ROS_ERROR_STREAM("[SCOPE] Target surface construction failed. "
                     << target_surface_result.message);
    return 1;
  }

  const std::vector<scope::SurfaceElement>& surface_elements =
      target_surface_result.elements;

  ROS_INFO_STREAM("[SCOPE] " << target_surface_result.message);
  ROS_INFO_STREAM("[SCOPE] Target surface element count: "
                  << surface_elements.size());

  // --------------------------------------------------------------------------
  // Candidate viewpoint generation
  // --------------------------------------------------------------------------
  ROS_INFO_STREAM("[SCOPE] Start candidate viewpoint generation.");

  scope::CandidateGenerator candidate_generator(params.viewpoint);

  const scope::CandidateGenerationResult candidate_result =
      candidate_generator.generateFromSurfaceElements(surface_elements);

  if (!candidate_result.success)
  {
    ROS_ERROR_STREAM("[SCOPE] Candidate viewpoint generation failed. "
                     << candidate_result.message);
    return 1;
  }

  const std::vector<scope::ViewpointCandidate>& candidate_viewpoints =
      candidate_result.candidates;

  ROS_INFO_STREAM("[SCOPE] " << candidate_result.message);
  ROS_INFO_STREAM("[SCOPE] Candidate viewpoint count: "
                  << candidate_viewpoints.size());      
                  
  // --------------------------------------------------------------------------
  // Safety filtering based on KD-tree clearance
  // --------------------------------------------------------------------------
  ROS_INFO_STREAM("[SCOPE] Start candidate safety filtering.");

  scope::SafetyFilter safety_filter(params.safety);

  const scope::SafetyFilterResult safety_result =
      safety_filter.filterCandidates(candidate_viewpoints, processed_cloud);

  if (!safety_result.success)
  {
    ROS_ERROR_STREAM("[SCOPE] Candidate safety filtering failed. "
                     << safety_result.message);
    return 1;
  }

  const std::vector<scope::ViewpointCandidate>& safe_candidates =
      safety_result.safe_candidates;

  const std::vector<scope::ViewpointCandidate>& unsafe_candidates =
      safety_result.unsafe_candidates;

  // 数据调试用⬇： 
  const std::size_t partition_count =
    safe_candidates.size() + unsafe_candidates.size();

  ROS_INFO_STREAM("[SCOPE] Candidate partition check: all="
                  << candidate_viewpoints.size()
                  << ", safe+unsafe=" << partition_count);

  if (partition_count != candidate_viewpoints.size())
  {
    ROS_WARN_STREAM("[SCOPE] Candidate partition mismatch!");
  }
      
  ROS_INFO_STREAM("[SCOPE] " << safety_result.message);
  ROS_INFO_STREAM("[SCOPE] Safe candidate count: "
                  << safe_candidates.size());
  ROS_INFO_STREAM("[SCOPE] Unsafe candidate count: "
                  << unsafe_candidates.size());

  // --------------------------------------------------------------------------
  // FOV-based coverage evaluation
  // --------------------------------------------------------------------------
  ROS_INFO_STREAM("[SCOPE] Start FOV-based coverage evaluation.");

  scope::CoverageEvaluator coverage_evaluator(params.sensor.camera,
                                              params.coverage);

  const scope::CoverageEvaluationResult coverage_result =
      coverage_evaluator.evaluateCandidates(safe_candidates,
                                            surface_elements);

  if (!coverage_result.success)
  {
    ROS_ERROR_STREAM("[SCOPE] FOV-based coverage evaluation failed. "
                     << coverage_result.message);
    return 1;
  }

  const std::vector<scope::ViewpointCandidate>& fov_valid_candidates =
      coverage_result.fov_valid_candidates;

  ROS_INFO_STREAM("[SCOPE] " << coverage_result.message);
  ROS_INFO_STREAM("[SCOPE] FOV-valid candidate count: "
                  << fov_valid_candidates.size());                  

  // --------------------------------------------------------------------------
  // Viewpoint scoring and ranking
  // --------------------------------------------------------------------------
  ROS_INFO_STREAM("[SCOPE] Start viewpoint scoring and ranking.");

  scope::ViewpointSelector viewpoint_selector(params.scoring);

  const scope::ViewpointScoringResult scoring_result =
      viewpoint_selector.scoreAndRankCandidates(fov_valid_candidates);

  if (!scoring_result.success)
  {
    ROS_ERROR_STREAM("[SCOPE] Viewpoint scoring failed. "
                     << scoring_result.message);
    return 1;
  }

  const std::vector<scope::ViewpointCandidate>& ranked_candidates =
      scoring_result.ranked_candidates;

  const std::vector<scope::ViewpointCandidate>& top_candidates =
      scoring_result.top_candidates;

  ROS_INFO_STREAM("[SCOPE] " << scoring_result.message);
  ROS_INFO_STREAM("[SCOPE] Ranked candidate count: "
                  << ranked_candidates.size());
  ROS_INFO_STREAM("[SCOPE] Top candidate count: "
                  << top_candidates.size());

  // --------------------------------------------------------------------------
  // Greedy coverage selection
  // --------------------------------------------------------------------------
  ROS_INFO_STREAM("[SCOPE] Start greedy coverage selection.");

  const scope::GreedyCoverageSelectionResult greedy_result =
      viewpoint_selector.selectGreedyCoverageCandidates(ranked_candidates,
                                                        surface_elements,
                                                        params.coverage);

  if (!greedy_result.success)
  {
    ROS_ERROR_STREAM("[SCOPE] Greedy coverage selection failed. "
                     << greedy_result.message);
    return 1;
  }

  const std::vector<scope::ViewpointCandidate>& selected_candidates =
      greedy_result.selected_candidates;

  ROS_INFO_STREAM("[SCOPE] " << greedy_result.message);
  ROS_INFO_STREAM("[SCOPE] Selected candidate count: "
                  << selected_candidates.size());
  ROS_INFO_STREAM("[SCOPE] Selected coverage ratio: "
                  << greedy_result.coverage_ratio);

  // --------------------------------------------------------------------------
  // Selected waypoint export
  // --------------------------------------------------------------------------
  scope::OutputParams resolved_output_params = params.output;

  resolved_output_params.output_dir =
      resolvePackageRelativePath(params.output.output_dir, "scope");

  ROS_INFO_STREAM("[SCOPE] Output directory resolved to: "
                  << resolved_output_params.output_dir);

  if (params.output.enable_selected_waypoint_export)
  {
    ROS_INFO_STREAM("[SCOPE] Start selected waypoint export.");

    const scope::SelectedWaypointExportResult waypoint_export_result =
        scope::WaypointIO::exportSelectedWaypoints(resolved_output_params,
                                                  params.mission,
                                                  selected_candidates,
                                                  params.scope.world_frame);

    if (!waypoint_export_result.success)
    {
      ROS_ERROR_STREAM("[SCOPE] Selected waypoint export failed. "
                      << waypoint_export_result.message);
      return 1;
    }

    ROS_INFO_STREAM("[SCOPE] " << waypoint_export_result.message);
  }
                
  // --------------------------------------------------------------------------
  // Candidate diagnostics export
  // --------------------------------------------------------------------------
  if (params.output.enable_candidate_export)
  {
    ROS_INFO_STREAM("[SCOPE] Start candidate diagnostics export.");

    // scope::OutputParams resolved_output_params = params.output;

    resolved_output_params.output_dir =
        resolvePackageRelativePath(params.output.output_dir, "scope");

    ROS_INFO_STREAM("[SCOPE] Candidate export output_dir resolved to: "
                    << resolved_output_params.output_dir);

    scope::SafetyFilterResult export_safety_result = safety_result;

    // Use FOV-evaluated candidates for export, so the exported files include
    // coverage_gain, score, and covered_surface_indices.
    export_safety_result.safe_candidates = ranked_candidates;
    export_safety_result.safe_count = export_safety_result.safe_candidates.size();

    const scope::CandidateExportResult export_result =
        scope::WaypointIO::exportCandidateDiagnostics(resolved_output_params,
                                                      candidate_result,
                                                      export_safety_result,
                                                      params.scope.world_frame);

    if (!export_result.success)
    {
      ROS_ERROR_STREAM("[SCOPE] Candidate diagnostics export failed. "
                      << export_result.message);
      return 1;
    }

    ROS_INFO_STREAM("[SCOPE] " << export_result.message);
    ROS_INFO_STREAM("[SCOPE]   statistics json: "
                    << export_result.statistics_json_path);
    ROS_INFO_STREAM("[SCOPE]   safe csv: "
                    << export_result.safe_csv_path);
    ROS_INFO_STREAM("[SCOPE]   unsafe csv: "
                    << export_result.unsafe_csv_path);
    ROS_INFO_STREAM("[SCOPE]   safe yaml: "
                    << export_result.safe_yaml_path);
    ROS_INFO_STREAM("[SCOPE]   unsafe yaml: "
                    << export_result.unsafe_yaml_path);
  }                

  // --------------------------------------------------------------------------
  // Publishers
  // --------------------------------------------------------------------------
  ros::Publisher raw_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>(
          params.visualization.raw_cloud_topic, 1, true);

  ros::Publisher processed_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>(
          params.visualization.processed_cloud_topic, 1, true);

  ros::Publisher normal_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>(
          "/scope/normal_cloud", 1, true);

  ros::Publisher normal_marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>(
          "/scope/normal_markers", 1, true);

  ros::Publisher candidate_marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>(
          "/scope/candidate_markers", 1, true);

  ros::Publisher safe_candidate_marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>(
          "/scope/safe_candidate_markers", 1, true);

  ros::Publisher unsafe_candidate_marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>(
          "/scope/unsafe_candidate_markers", 1, true);
          
  ros::Publisher fov_candidate_marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>(
          "/scope/fov_candidate_markers", 1, true);          

  ros::Publisher top_candidate_marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>(
          "/scope/top_candidate_markers", 1, true);

  ros::Publisher selected_candidate_marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>(
          "/scope/selected_candidate_markers", 1, true);

  const sensor_msgs::PointCloud2 raw_msg =
      toRosCloudMsg(raw_cloud, params.scope.world_frame);

  const sensor_msgs::PointCloud2 processed_msg =
      toRosCloudMsg(processed_cloud, params.scope.world_frame);

  const sensor_msgs::PointCloud2 normal_msg =
      toRosNormalCloudMsg(normal_cloud, params.scope.world_frame);

  const visualization_msgs::MarkerArray normal_markers =
      createNormalMarkers(normal_cloud,
                          params.scope.world_frame,
                          0.20,
                          1000);

  const std::size_t candidate_visualization_stride =
      computeGlobalVisualizationStride(candidate_viewpoints.size(), 1000);

  ROS_INFO_STREAM("[SCOPE] Candidate visualization global id stride: "
                  << candidate_visualization_stride);

  const visualization_msgs::MarkerArray candidate_markers =
      createViewpointMarkers(candidate_viewpoints,
                             params.scope.world_frame,
                             "scope_all_candidates",
                             0.01,
                             0.05,
                             candidate_visualization_stride,
                             1.0f, 0.7f, 0.0f, 0.9f,
                             0.0f, 0.4f, 1.0f, 0.8f);

  const visualization_msgs::MarkerArray safe_candidate_markers =
      createViewpointMarkers(safe_candidates,
                             params.scope.world_frame,
                             "scope_safe_candidates",
                             0.012,
                             0.06,
                             candidate_visualization_stride,
                             0.0f, 1.0f, 0.0f, 0.9f,
                             0.0f, 1.0f, 1.0f, 0.8f);

  const visualization_msgs::MarkerArray unsafe_candidate_markers =
      createViewpointMarkers(unsafe_candidates,
                             params.scope.world_frame,
                             "scope_unsafe_candidates",
                             0.012,
                             0.06,
                             candidate_visualization_stride,
                             1.0f, 0.0f, 0.0f, 0.9f,
                             1.0f, 0.0f, 0.5f, 0.8f);
                             
  const visualization_msgs::MarkerArray fov_candidate_markers =
      createViewpointMarkers(fov_valid_candidates,
                             params.scope.world_frame,
                             "scope_fov_candidates",
                             0.014,
                             0.07,
                             candidate_visualization_stride,
                             1.0f, 1.0f, 1.0f, 0.95f,
                             1.0f, 1.0f, 0.0f, 0.85f);                             

  const visualization_msgs::MarkerArray top_candidate_markers =
      createViewpointMarkers(top_candidates,
                             params.scope.world_frame,
                             "scope_top_candidates",
                             0.018,
                             0.09,
                             1,
                             0.5f, 0.0f, 1.0f, 0.95f,
                             0.8f, 0.4f, 1.0f, 0.9f);

  const visualization_msgs::MarkerArray selected_candidate_markers =
      createViewpointMarkers(selected_candidates,
                             params.scope.world_frame,
                             "scope_selected_candidates",
                             0.022,
                             0.12,
                             1,
                             1.0f, 0.2f, 1.0f, 0.98f,
                             1.0f, 0.8f, 1.0f, 0.9f);

  ros::Duration(0.5).sleep();

  raw_cloud_pub.publish(raw_msg);
  processed_cloud_pub.publish(processed_msg);
  normal_cloud_pub.publish(normal_msg);
  normal_marker_pub.publish(normal_markers);
  candidate_marker_pub.publish(candidate_markers);
  candidate_marker_pub.publish(candidate_markers);
  safe_candidate_marker_pub.publish(safe_candidate_markers);
  unsafe_candidate_marker_pub.publish(unsafe_candidate_markers);
  fov_candidate_marker_pub.publish(fov_candidate_markers);
  top_candidate_marker_pub.publish(top_candidate_markers);
  selected_candidate_marker_pub.publish(selected_candidate_markers);

  ROS_INFO_STREAM("[SCOPE] Published raw cloud topic: "
                  << params.visualization.raw_cloud_topic);
  ROS_INFO_STREAM("[SCOPE] Published processed cloud topic: "
                  << params.visualization.processed_cloud_topic);
  ROS_INFO_STREAM("[SCOPE] Published normal cloud topic: /scope/normal_cloud");
  ROS_INFO_STREAM("[SCOPE] Published normal marker topic: /scope/normal_markers");
  ROS_INFO_STREAM("[SCOPE] Published candidate marker topic: /scope/candidate_markers");
  ROS_INFO_STREAM("[SCOPE] Published safe candidate marker topic: /scope/safe_candidate_markers");
  ROS_INFO_STREAM("[SCOPE] Published unsafe candidate marker topic: /scope/unsafe_candidate_markers");
  ROS_INFO_STREAM("[SCOPE] Published FOV candidate marker topic: /scope/fov_candidate_markers");
  ROS_INFO_STREAM("[SCOPE] Published top candidate marker topic: /scope/top_candidate_markers");
  ROS_INFO_STREAM("[SCOPE] Published selected candidate marker topic: /scope/selected_candidate_markers");
  ROS_INFO_STREAM("[SCOPE] Fixed frame should be set to: "
                  << params.scope.world_frame);
  ROS_INFO_STREAM("[SCOPE] V0.4.0 finished. Keep node alive for RViz visualization.");

  ros::spin();

  return 0;
} 