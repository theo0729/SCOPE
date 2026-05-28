#pragma once

#include <string>

#include "scope/common/parameters.hpp"
#include "scope/common/types.hpp"

namespace scope
{

struct NormalEstimationResult
{
  bool success = false;
  std::string message;

  PointNormalCloudPtr cloud;

  std::size_t input_points = 0;
  std::size_t estimated_points = 0;
  std::size_t valid_normal_points = 0;
};

class NormalEstimator
{
public:
  NormalEstimator() = default;
  explicit NormalEstimator(const NormalParams& params);

  ~NormalEstimator() = default;

  void setParams(const NormalParams& params);
  const NormalParams& params() const;

  NormalEstimationResult estimate(const PointCloudConstPtr& input_cloud) const;

private:
  static bool isNormalFinite(double nx, double ny, double nz);
  static void flipNormalTowardsOriginIfNeeded(const Eigen::Vector3d& point,
                                              Eigen::Vector3d* normal);
  static PointNormalT mergePointAndNormal(const PointT& point,
                                          const Eigen::Vector3d& normal,
                                          float curvature);
private:
  NormalParams params_;
};

}  // namespace scope