#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace scope
{

// ================================================================
// Basic point cloud aliases
// ================================================================

using PointT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloudT::Ptr;
using PointCloudConstPtr = PointCloudT::ConstPtr;

using PointNormalT = pcl::PointXYZRGBNormal;
using PointNormalCloudT = pcl::PointCloud<PointNormalT>;
using PointNormalCloudPtr = PointNormalCloudT::Ptr;
using PointNormalCloudConstPtr = PointNormalCloudT::ConstPtr;

// ================================================================
// Basic geometric utilities
// ================================================================

struct AxisAlignedBox
{
  Eigen::Vector3d min = Eigen::Vector3d(-10.0, -10.0, -10.0);
  Eigen::Vector3d max = Eigen::Vector3d(10.0, 10.0, 10.0);

  bool isValid() const
  {
    return (min.x() <= max.x()) && (min.y() <= max.y()) && (min.z() <= max.z());
  }

  bool contains(const Eigen::Vector3d& p) const
  {
    return (p.x() >= min.x() && p.x() <= max.x() &&
            p.y() >= min.y() && p.y() <= max.y() &&
            p.z() >= min.z() && p.z() <= max.z());
  }
};

struct CloudStatistics
{
  std::size_t num_points = 0;

  Eigen::Vector3d min_bound =
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN());

  Eigen::Vector3d max_bound =
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN());

  Eigen::Vector3d centroid =
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN());

  bool valid = false;
};

// ================================================================
// Target surface representation
// ================================================================

enum class SurfaceSource : std::uint8_t
{
  UNKNOWN = 0,
  POINT_CLOUD = 1,
  MESH_TRIANGLE = 2
};

struct SurfaceElement
{
  std::uint32_t id = 0;
  std::uint32_t source_index = 0;

  SurfaceSource source = SurfaceSource::UNKNOWN;

  // Representative surface location.
  // For mesh input, this is usually the triangle centroid.
  // For point-cloud input, this is the oriented surface point.
  Eigen::Vector3d position = Eigen::Vector3d::Zero();

  // Outward or observation-side surface normal.
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();

  // Area represented by this element.
  // For mesh input, this can be the triangle area.
  // For point cloud input, this can be an approximated local area weight.
  double area = 1.0;

  // Coverage priority weight.
  // It can be area * ROI weight in later versions.
  double weight = 1.0;

  // Local curvature from normal estimation.
  // It is useful for filtering unstable edge points.
  double curvature = 0.0;

  int roi_id = 0;

  bool covered = false;
  std::uint32_t covered_count = 0;
};

// ================================================================
// Viewpoint and waypoint representations
// ================================================================

enum class PitchMode : std::uint8_t
{
  FIXED_MOUNT = 0,
  VIRTUAL_CAMERA = 1,
  GIMBAL = 2,
  BODY_ATTITUDE = 3
};

enum class SensorMode : std::uint8_t
{
  CAMERA = 0,
  LIDAR = 1,
  CAMERA_LIDAR = 2
};

struct ViewpointCandidate
{
  std::uint32_t id = 0;

  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_point = Eigen::Vector3d::Zero();

  // Direction from viewpoint to target point.
  Eigen::Vector3d viewing_direction = Eigen::Vector3d::UnitX();

  double yaw_rad = 0.0;
  double pitch_rad = 0.0;

  bool pitch_enabled = false;

  double view_distance = 0.0;
  double incidence_angle_rad = 0.0;

  double coverage_gain = 0.0;
  double safety_clearance = std::numeric_limits<double>::infinity();
  double score = 0.0;

  bool is_safe = true;
  bool is_valid = true;

  std::vector<std::uint32_t> covered_surface_indices;
};

struct InspectionWaypoint
{
  std::uint32_t id = 0;

  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_point = Eigen::Vector3d::Zero();

  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
  bool pitch_enabled = false;

  double view_distance = 0.0;
  double expected_coverage_ratio = 0.0;
  double capture_time = 2.0;

  SensorMode sensor_mode = SensorMode::CAMERA;

  bool is_safe = true;
};

struct CoverageReport
{
  std::size_t num_surface_elements = 0;
  std::size_t num_candidate_viewpoints = 0;
  std::size_t num_selected_waypoints = 0;

  double covered_weight = 0.0;
  double total_weight = 0.0;
  double coverage_ratio = 0.0;

  double total_path_length = 0.0;
  double min_safety_clearance = std::numeric_limits<double>::infinity();
};

// ================================================================
// Helper conversion declarations
// ================================================================

inline double rad2deg(const double rad)
{
  return rad * 180.0 / M_PI;
}

inline double deg2rad(const double deg)
{
  return deg * M_PI / 180.0;
}

inline std::string surfaceSourceToString(const SurfaceSource source)
{
  switch (source)
  {
    case SurfaceSource::POINT_CLOUD:
      return "point_cloud";
    case SurfaceSource::MESH_TRIANGLE:
      return "mesh_triangle";
    case SurfaceSource::UNKNOWN:
    default:
      return "unknown";
  }
}

inline std::string sensorModeToString(const SensorMode mode)
{
  switch (mode)
  {
    case SensorMode::CAMERA:
      return "camera";
    case SensorMode::LIDAR:
      return "lidar";
    case SensorMode::CAMERA_LIDAR:
      return "camera_lidar";
    default:
      return "camera";
  }
}

inline std::string pitchModeToString(const PitchMode mode)
{
  switch (mode)
  {
    case PitchMode::FIXED_MOUNT:
      return "fixed_mount";
    case PitchMode::VIRTUAL_CAMERA:
      return "virtual_camera";
    case PitchMode::GIMBAL:
      return "gimbal";
    case PitchMode::BODY_ATTITUDE:
      return "body_attitude";
    default:
      return "fixed_mount";
  }
}

}  // namespace scope