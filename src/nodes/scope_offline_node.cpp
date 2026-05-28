#include <ros/ros.h>

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
    ROS_ERROR_STREAM("[SCOPE] V0.1.4 currently requires input.use_cloud = true.");
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

  ros::Duration(0.5).sleep();

  raw_cloud_pub.publish(raw_msg);
  processed_cloud_pub.publish(processed_msg);
  normal_cloud_pub.publish(normal_msg);
  normal_marker_pub.publish(normal_markers);

  ROS_INFO_STREAM("[SCOPE] Published raw cloud topic: "
                  << params.visualization.raw_cloud_topic);
  ROS_INFO_STREAM("[SCOPE] Published processed cloud topic: "
                  << params.visualization.processed_cloud_topic);
  ROS_INFO_STREAM("[SCOPE] Published normal cloud topic: /scope/normal_cloud");
  ROS_INFO_STREAM("[SCOPE] Published normal marker topic: /scope/normal_markers");
  ROS_INFO_STREAM("[SCOPE] Fixed frame should be set to: "
                  << params.scope.world_frame);
  ROS_INFO_STREAM("[SCOPE] V0.1.4 finished. Keep node alive for RViz visualization.");

  ros::spin();

  return 0;
}