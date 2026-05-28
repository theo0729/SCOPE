#include <ros/ros.h>

#include <sstream>
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "scope/common/parameters.hpp"
#include "scope/common/types.hpp"
#include "scope/io/pcd_loader.hpp"
#include "scope/map/cloud_preprocessor.hpp"

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
    ROS_ERROR_STREAM("[SCOPE] V0.1.2 currently requires input.use_cloud = true.");
    return 1;
  }

  // --------------------------------------------------------------------------
  // 1. Load raw PCD
  // --------------------------------------------------------------------------
  ROS_INFO_STREAM("[SCOPE] Loading PCD: " << params.input.cloud_path);

  const scope::PcdLoadResult load_result =
      scope::PcdLoader::loadXYZ(params.input.cloud_path);

  if (!load_result.success)
  {
    ROS_ERROR_STREAM("[SCOPE] Failed to load PCD. " << load_result.message);
    return 1;
  }

  scope::PointCloudPtr raw_cloud = load_result.cloud;

  ROS_INFO_STREAM("[SCOPE] " << load_result.message);
  ROS_INFO_STREAM("[SCOPE] Raw cloud statistics: "
                  << statsToString(load_result.statistics));

  // --------------------------------------------------------------------------
  // 2. Preprocess cloud
  // --------------------------------------------------------------------------
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
  // 3. Publish cloud to RViz
  // --------------------------------------------------------------------------
  ros::Publisher raw_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>(
          params.visualization.raw_cloud_topic, 1, true);

  ros::Publisher processed_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>(
          params.visualization.processed_cloud_topic, 1, true);

  const sensor_msgs::PointCloud2 raw_msg =
      toRosCloudMsg(raw_cloud, params.scope.world_frame);

  const sensor_msgs::PointCloud2 processed_msg =
      toRosCloudMsg(processed_cloud, params.scope.world_frame);

  // Wait briefly for publisher registration.
  ros::Duration(0.5).sleep();

  raw_cloud_pub.publish(raw_msg);
  processed_cloud_pub.publish(processed_msg);

  ROS_INFO_STREAM("[SCOPE] Published raw cloud topic: "
                  << params.visualization.raw_cloud_topic);
  ROS_INFO_STREAM("[SCOPE] Published processed cloud topic: "
                  << params.visualization.processed_cloud_topic);
  ROS_INFO_STREAM("[SCOPE] Fixed frame should be set to: "
                  << params.scope.world_frame);
  ROS_INFO_STREAM("[SCOPE] V0.1.2 finished. Keep node alive for RViz visualization.");

  ros::spin();

  return 0;
}