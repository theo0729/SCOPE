#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "scope/common/parameters.hpp"
#include "scope/common/types.hpp"

namespace scope
{

struct TargetSurfaceBuildResult
{
  bool success = false;
  std::string message;

  std::vector<SurfaceElement> elements;

  std::size_t input_points = 0;
  std::size_t invalid_point_count = 0;
  std::size_t invalid_normal_count = 0;
  std::size_t curvature_filtered_count = 0;
  std::size_t output_elements = 0;

  double total_weight = 0.0;

  double min_curvature = 0.0;
  double max_curvature = 0.0;
  double mean_curvature = 0.0;
};

class TargetSurfaceBuilder
{
public:
  TargetSurfaceBuilder() = default;
  explicit TargetSurfaceBuilder(const TargetSurfaceParams& params);

  ~TargetSurfaceBuilder() = default;

  void setParams(const TargetSurfaceParams& params);
  const TargetSurfaceParams& params() const;

  TargetSurfaceBuildResult buildFromNormalCloud(
      const PointNormalCloudConstPtr& normal_cloud) const;

private:
  static bool isFinitePoint(const PointNormalT& p);
  static bool isFiniteNormal(const PointNormalT& p);

  SurfaceElement createSurfaceElement(const PointNormalT& p,
                                      std::uint32_t source_index,
                                      std::uint32_t element_id) const;

private:
  TargetSurfaceParams params_;
};

}  // namespace scope