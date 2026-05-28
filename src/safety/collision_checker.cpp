#include "scope/safety/collision_checker.hpp"

#include <cmath>
#include <sstream>
#include <vector>

namespace scope
{

bool CollisionChecker::setObstacleCloud(const PointCloudConstPtr& obstacle_cloud,
                                        std::string* error_message)
{
  if (!obstacle_cloud)
  {
    if (error_message)
    {
      *error_message = "Obstacle cloud is null.";
    }
    has_map_ = false;
    return false;
  }

  if (obstacle_cloud->empty())
  {
    if (error_message)
    {
      *error_message = "Obstacle cloud is empty.";
    }
    has_map_ = false;
    return false;
  }

  obstacle_cloud_ = obstacle_cloud;
  kdtree_.setInputCloud(obstacle_cloud_);
  has_map_ = true;

  if (error_message)
  {
    std::ostringstream oss;
    oss << "KD-tree obstacle cloud is set. points="
        << obstacle_cloud_->size();
    *error_message = oss.str();
  }

  return true;
}

bool CollisionChecker::hasMap() const
{
  return has_map_;
}

std::size_t CollisionChecker::obstaclePointNum() const
{
  if (!obstacle_cloud_)
  {
    return 0;
  }

  return obstacle_cloud_->size();
}

ClearanceQueryResult CollisionChecker::queryNearestDistance(
    const Eigen::Vector3d& query_position) const
{
  ClearanceQueryResult result;

  if (!has_map_ || !obstacle_cloud_)
  {
    result.success = false;
    result.message = "KD-tree obstacle map is not initialized.";
    return result;
  }

  if (!isFiniteVector3d(query_position))
  {
    result.success = false;
    result.message = "Query position is not finite.";
    return result;
  }

  PointT query_point;
  query_point.x = static_cast<float>(query_position.x());
  query_point.y = static_cast<float>(query_position.y());
  query_point.z = static_cast<float>(query_position.z());

  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);

  const int found_num =
      kdtree_.nearestKSearch(query_point, 1, indices, squared_distances);

  if (found_num <= 0)
  {
    result.success = false;
    result.message = "Nearest neighbor search failed.";
    return result;
  }

  result.nearest_index = indices[0];
  result.distance = std::sqrt(static_cast<double>(squared_distances[0]));
  result.success = true;
  result.message = "Nearest distance query succeeded.";

  return result;
}

bool CollisionChecker::hasEnoughClearance(const Eigen::Vector3d& query_position,
                                          double min_clearance,
                                          double* nearest_distance) const
{
  const ClearanceQueryResult result = queryNearestDistance(query_position);

  if (!result.success)
  {
    if (nearest_distance)
    {
      *nearest_distance = std::numeric_limits<double>::quiet_NaN();
    }
    return false;
  }

  if (nearest_distance)
  {
    *nearest_distance = result.distance;
  }

  return result.distance >= min_clearance;
}

SegmentCheckResult CollisionChecker::checkSegmentClearance(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    double min_clearance,
    double resolution) const
{
  SegmentCheckResult result;

  if (!has_map_)
  {
    result.success = false;
    result.message = "KD-tree obstacle map is not initialized.";
    return result;
  }

  if (!isFiniteVector3d(start) || !isFiniteVector3d(end))
  {
    result.success = false;
    result.message = "Segment endpoint is not finite.";
    return result;
  }

  if (resolution <= 0.0)
  {
    result.success = false;
    result.message = "Segment check resolution must be positive.";
    return result;
  }

  const Eigen::Vector3d diff = end - start;
  const double length = diff.norm();

  if (length < 1.0e-9)
  {
    double distance = std::numeric_limits<double>::infinity();
    const bool safe = hasEnoughClearance(start, min_clearance, &distance);

    result.success = true;
    result.is_safe = safe;
    result.min_distance = distance;
    result.checked_points = 1;
    result.message = "Zero-length segment was checked as a point.";
    return result;
  }

  const std::size_t steps =
      static_cast<std::size_t>(std::ceil(length / resolution));

  result.min_distance = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i <= steps; ++i)
  {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const Eigen::Vector3d p = start + t * diff;

    const ClearanceQueryResult query = queryNearestDistance(p);

    if (!query.success)
    {
      result.success = false;
      result.message = "Nearest distance query failed during segment check.";
      return result;
    }

    result.min_distance = std::min(result.min_distance, query.distance);
    ++result.checked_points;

    if (query.distance < min_clearance)
    {
      result.success = true;
      result.is_safe = false;
      result.message = "Segment violates minimum clearance.";
      return result;
    }
  }

  result.success = true;
  result.is_safe = true;
  result.message = "Segment clearance check succeeded.";

  return result;
}

bool CollisionChecker::isFiniteVector3d(const Eigen::Vector3d& p)
{
  return std::isfinite(p.x()) &&
         std::isfinite(p.y()) &&
         std::isfinite(p.z());
}

}  // namespace scope