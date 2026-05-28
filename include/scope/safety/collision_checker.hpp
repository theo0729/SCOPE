#pragma once

#include <string>

#include <pcl/kdtree/kdtree_flann.h>

#include "scope/common/types.hpp"

namespace scope
{

struct ClearanceQueryResult
{
  bool success = false;
  std::string message;

  double distance = std::numeric_limits<double>::infinity();
  int nearest_index = -1;
};

struct SegmentCheckResult
{
  bool success = false;
  std::string message;

  bool is_safe = false;
  double min_distance = std::numeric_limits<double>::infinity();
  std::size_t checked_points = 0;
};

class CollisionChecker
{
public:
  CollisionChecker() = default;
  ~CollisionChecker() = default;

  bool setObstacleCloud(const PointCloudConstPtr& obstacle_cloud,
                        std::string* error_message = nullptr);

  bool hasMap() const;
  std::size_t obstaclePointNum() const;

  ClearanceQueryResult queryNearestDistance(
      const Eigen::Vector3d& query_position) const;

  bool hasEnoughClearance(const Eigen::Vector3d& query_position,
                          double min_clearance,
                          double* nearest_distance = nullptr) const;

  SegmentCheckResult checkSegmentClearance(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& end,
      double min_clearance,
      double resolution) const;

private:
  static bool isFiniteVector3d(const Eigen::Vector3d& p);

private:
  PointCloudConstPtr obstacle_cloud_;
  pcl::KdTreeFLANN<PointT> kdtree_;
  bool has_map_ = false;
};

}  // namespace scope