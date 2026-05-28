#include "scope/map/normal_estimator.hpp"

#include <cmath>
#include <sstream>

#include <pcl/common/point_tests.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

namespace scope
{

NormalEstimator::NormalEstimator(const NormalParams& params)
  : params_(params)
{
}

void NormalEstimator::setParams(const NormalParams& params)
{
  params_ = params;
}

const NormalParams& NormalEstimator::params() const
{
  return params_;
}

NormalEstimationResult NormalEstimator::estimate(const PointCloudConstPtr& input_cloud) const
{
  NormalEstimationResult result;
  result.cloud.reset(new PointNormalCloudT);

  if (!input_cloud)
  {
    result.success = false;
    result.message = "Input cloud is null.";
    return result;
  }

  if (input_cloud->empty())
  {
    result.success = false;
    result.message = "Input cloud is empty.";
    return result;
  }

  result.input_points = input_cloud->size();

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud(input_cloud);

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  ne.setSearchMethod(tree);

  if (params_.search_radius > 0.0)
  {
    ne.setRadiusSearch(params_.search_radius);
  }
  else
  {
    ne.setKSearch(params_.k_search);
  }

  try
  {
    ne.compute(*normals);
  }
  catch (const std::exception& e)
  {
    result.success = false;
    result.message = std::string("Normal estimation failed. Error: ") + e.what();
    return result;
  }

  if (normals->size() != input_cloud->size())
  {
    std::ostringstream oss;
    oss << "Normal size mismatch. input=" << input_cloud->size()
        << ", normals=" << normals->size();
    result.success = false;
    result.message = oss.str();
    return result;
  }

  result.cloud->points.reserve(input_cloud->size());

  for (std::size_t i = 0; i < input_cloud->size(); ++i)
  {
    const PointT& p = input_cloud->points[i];
    const pcl::Normal& n = normals->points[i];

    if (!pcl::isFinite(p))
    {
      continue;
    }

    if (!isNormalFinite(n.normal_x, n.normal_y, n.normal_z))
    {
      continue;
    }

    Eigen::Vector3d normal(static_cast<double>(n.normal_x),
                           static_cast<double>(n.normal_y),
                           static_cast<double>(n.normal_z));

    const double norm = normal.norm();

    if (norm < 1.0e-9)
    {
      continue;
    }

    normal.normalize();

    if (params_.flip_towards_origin)
    {
      const Eigen::Vector3d point(static_cast<double>(p.x),
                                  static_cast<double>(p.y),
                                  static_cast<double>(p.z));
      flipNormalTowardsOriginIfNeeded(point, &normal);
    }

    PointNormalT pn = mergePointAndNormal(p, normal, n.curvature);
    result.cloud->points.push_back(pn);
  }

  result.cloud->width = static_cast<std::uint32_t>(result.cloud->points.size());
  result.cloud->height = 1;
  result.cloud->is_dense = false;

  result.estimated_points = normals->size();
  result.valid_normal_points = result.cloud->size();

  if (result.cloud->empty())
  {
    result.success = false;
    result.message = "No valid normal points were generated.";
    return result;
  }

  std::ostringstream oss;
  oss << "Normal estimation finished. "
      << "input=" << result.input_points
      << ", estimated=" << result.estimated_points
      << ", valid=" << result.valid_normal_points
      << ", search_radius=" << params_.search_radius
      << ", k_search=" << params_.k_search
      << ", flip_towards_origin=" << std::boolalpha << params_.flip_towards_origin;

  result.success = true;
  result.message = oss.str();

  return result;
}

bool NormalEstimator::isNormalFinite(double nx, double ny, double nz)
{
  return std::isfinite(nx) && std::isfinite(ny) && std::isfinite(nz);
}

void NormalEstimator::flipNormalTowardsOriginIfNeeded(const Eigen::Vector3d& point,
                                                      Eigen::Vector3d* normal)
{
  if (normal == nullptr)
  {
    return;
  }

  const Eigen::Vector3d direction_to_origin = -point;

  if (direction_to_origin.norm() < 1.0e-9)
  {
    return;
  }

  if (normal->dot(direction_to_origin) < 0.0)
  {
    *normal = -*normal;
  }
}

PointNormalT NormalEstimator::mergePointAndNormal(const PointT& point,
                                                  const Eigen::Vector3d& normal,
                                                  float curvature)
{
  PointNormalT output;

  output.x = point.x;
  output.y = point.y;
  output.z = point.z;

  output.r = point.r;
  output.g = point.g;
  output.b = point.b;

  output.normal_x = static_cast<float>(normal.x());
  output.normal_y = static_cast<float>(normal.y());
  output.normal_z = static_cast<float>(normal.z());

  output.curvature = curvature;

  return output;
}

}  // namespace scope