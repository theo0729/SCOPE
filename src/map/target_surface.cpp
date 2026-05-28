#include "scope/map/target_surface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace scope
{

TargetSurfaceBuilder::TargetSurfaceBuilder(const TargetSurfaceParams& params)
  : params_(params)
{
}

void TargetSurfaceBuilder::setParams(const TargetSurfaceParams& params)
{
  params_ = params;
}

const TargetSurfaceParams& TargetSurfaceBuilder::params() const
{
  return params_;
}

TargetSurfaceBuildResult TargetSurfaceBuilder::buildFromNormalCloud(
    const PointNormalCloudConstPtr& normal_cloud) const
{
  TargetSurfaceBuildResult result;

  if (!normal_cloud)
  {
    result.success = false;
    result.message = "Input normal cloud is null.";
    return result;
  }

  if (normal_cloud->empty())
  {
    result.success = false;
    result.message = "Input normal cloud is empty.";
    return result;
  }

  result.input_points = normal_cloud->size();
  result.elements.reserve(normal_cloud->size());

  double curvature_sum = 0.0;
  std::size_t curvature_valid_count = 0;

  double min_curv = std::numeric_limits<double>::infinity();
  double max_curv = -std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < normal_cloud->size(); ++i)
  {
    const PointNormalT& p = normal_cloud->points[i];

    if (!isFinitePoint(p))
    {
      ++result.invalid_point_count;
      continue;
    }

    if (!isFiniteNormal(p))
    {
      ++result.invalid_normal_count;
      continue;
    }

    const double curvature =
        std::isfinite(static_cast<double>(p.curvature))
            ? static_cast<double>(p.curvature)
            : 0.0;

    min_curv = std::min(min_curv, curvature);
    max_curv = std::max(max_curv, curvature);
    curvature_sum += curvature;
    ++curvature_valid_count;

    if (params_.enable_curvature_filter &&
        curvature > params_.max_curvature)
    {
      ++result.curvature_filtered_count;
      continue;
    }

    const std::uint32_t element_id =
        static_cast<std::uint32_t>(result.elements.size());

    SurfaceElement element =
        createSurfaceElement(p,
                             static_cast<std::uint32_t>(i),
                             element_id);

    result.total_weight += element.weight;
    result.elements.push_back(element);
  }

  result.output_elements = result.elements.size();

  if (curvature_valid_count > 0)
  {
    result.min_curvature = min_curv;
    result.max_curvature = max_curv;
    result.mean_curvature =
        curvature_sum / static_cast<double>(curvature_valid_count);
  }

  if (result.elements.empty())
  {
    result.success = false;
    result.message =
        "No valid target surface elements were generated. "
        "Please check curvature threshold or normal estimation parameters.";
    return result;
  }

  std::ostringstream oss;
  oss << "Target surface build finished. "
      << "input=" << result.input_points
      << ", invalid_points=" << result.invalid_point_count
      << ", invalid_normals=" << result.invalid_normal_count
      << ", curvature_filtered=" << result.curvature_filtered_count
      << ", output_elements=" << result.output_elements
      << ", total_weight=" << result.total_weight
      << ", curvature[min/mean/max]="
      << result.min_curvature << "/"
      << result.mean_curvature << "/"
      << result.max_curvature;

  result.success = true;
  result.message = oss.str();

  return result;
}

bool TargetSurfaceBuilder::isFinitePoint(const PointNormalT& p)
{
  return std::isfinite(static_cast<double>(p.x)) &&
         std::isfinite(static_cast<double>(p.y)) &&
         std::isfinite(static_cast<double>(p.z));
}

bool TargetSurfaceBuilder::isFiniteNormal(const PointNormalT& p)
{
  if (!std::isfinite(static_cast<double>(p.normal_x)) ||
      !std::isfinite(static_cast<double>(p.normal_y)) ||
      !std::isfinite(static_cast<double>(p.normal_z)))
  {
    return false;
  }

  const Eigen::Vector3d n(static_cast<double>(p.normal_x),
                          static_cast<double>(p.normal_y),
                          static_cast<double>(p.normal_z));

  return n.norm() > 1.0e-9;
}

SurfaceElement TargetSurfaceBuilder::createSurfaceElement(
    const PointNormalT& p,
    std::uint32_t source_index,
    std::uint32_t element_id) const
{
  SurfaceElement element;

  element.id = element_id;
  element.source_index = source_index;
  element.source = SurfaceSource::POINT_CLOUD;

  element.position = Eigen::Vector3d(static_cast<double>(p.x),
                                     static_cast<double>(p.y),
                                     static_cast<double>(p.z));

  Eigen::Vector3d normal(static_cast<double>(p.normal_x),
                         static_cast<double>(p.normal_y),
                         static_cast<double>(p.normal_z));

  normal.normalize();
  element.normal = normal;

  element.curvature =
      std::isfinite(static_cast<double>(p.curvature))
          ? static_cast<double>(p.curvature)
          : 0.0;

  element.area = params_.default_area;

  if (params_.use_area_weight)
  {
    element.weight = element.area;
  }
  else
  {
    element.weight = params_.default_weight;
  }

  element.roi_id = 0;
  element.covered = false;
  element.covered_count = 0;

  return element;
}

}  // namespace scope