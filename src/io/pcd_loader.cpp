#include "scope/io/pcd_loader.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/common/point_tests.h>

namespace scope
{

bool PcdLoader::fileExists(const std::string& path)
{
  std::ifstream file(path.c_str());
  return file.good();
}

PcdLoadResult PcdLoader::loadXYZ(const std::string& pcd_path)
{
  PcdLoadResult result;
  result.cloud.reset(new PointCloudT);

  if (pcd_path.empty())
  {
    result.success = false;
    result.message = "PCD path is empty.";
    return result;
  }

  if (!fileExists(pcd_path))
  {
    result.success = false;
    result.message = "PCD file does not exist: " + pcd_path;
    return result;
  }

  const int ret = pcl::io::loadPCDFile<PointT>(pcd_path, *(result.cloud));

  if (ret < 0)
  {
    std::ostringstream oss;
    oss << "Failed to load PCD file: " << pcd_path
        << ", pcl error code: " << ret;
    result.success = false;
    result.message = oss.str();
    return result;
  }

  result.statistics = computeStatistics(result.cloud);

  if (!result.statistics.valid)
  {
    result.success = false;
    result.message = "PCD file was loaded, but no valid finite points were found.";
    return result;
  }

  std::ostringstream oss;
  oss << "Loaded PCD file successfully: " << pcd_path
      << ", points: " << result.cloud->size();
  result.success = true;
  result.message = oss.str();

  return result;
}

bool PcdLoader::loadXYZ(const std::string& pcd_path,
                        PointCloudPtr* cloud,
                        std::string* error_message)
{
  if (cloud == nullptr)
  {
    if (error_message)
    {
      *error_message = "Output cloud pointer is null.";
    }
    return false;
  }

  const PcdLoadResult result = loadXYZ(pcd_path);

  if (!result.success)
  {
    if (error_message)
    {
      *error_message = result.message;
    }
    return false;
  }

  *cloud = result.cloud;

  if (error_message)
  {
    *error_message = result.message;
  }

  return true;
}

CloudStatistics PcdLoader::computeStatistics(const PointCloudConstPtr& cloud)
{
  CloudStatistics stats;

  if (!cloud || cloud->empty())
  {
    stats.valid = false;
    stats.num_points = 0;
    return stats;
  }

  Eigen::Vector3d min_bound(std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::infinity());

  Eigen::Vector3d max_bound(-std::numeric_limits<double>::infinity(),
                            -std::numeric_limits<double>::infinity(),
                            -std::numeric_limits<double>::infinity());

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();

  std::size_t valid_count = 0;

  for (const auto& p : cloud->points)
  {
    if (!pcl::isFinite(p))
    {
      continue;
    }

    const Eigen::Vector3d v(static_cast<double>(p.x),
                            static_cast<double>(p.y),
                            static_cast<double>(p.z));

    min_bound = min_bound.cwiseMin(v);
    max_bound = max_bound.cwiseMax(v);
    sum += v;
    ++valid_count;
  }

  stats.num_points = valid_count;

  if (valid_count == 0)
  {
    stats.valid = false;
    return stats;
  }

  stats.min_bound = min_bound;
  stats.max_bound = max_bound;
  stats.centroid = sum / static_cast<double>(valid_count);
  stats.valid = true;

  return stats;
}

}  // namespace scope