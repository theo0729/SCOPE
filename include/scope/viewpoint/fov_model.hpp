#pragma once

#include <string>

#include "scope/common/parameters.hpp"
#include "scope/common/types.hpp"

namespace scope
{

struct FovCheckResult
{
  bool visible = false;
  std::string reason;

  double distance = 0.0;
  double horizontal_angle_rad = 0.0;
  double vertical_angle_rad = 0.0;
  double incidence_angle_rad = 0.0;
};

class FovModel
{
public:
  FovModel() = default;
  explicit FovModel(const CameraParams& params);

  ~FovModel() = default;

  void setParams(const CameraParams& params);
  const CameraParams& params() const;

  FovCheckResult checkSurfaceElement(
      const ViewpointCandidate& candidate,
      const SurfaceElement& element,
      double max_incidence_angle_deg) const;

private:
  static bool buildCameraBasis(const Eigen::Vector3d& forward,
                               Eigen::Vector3d* right,
                               Eigen::Vector3d* up);

  static bool isFiniteVector3d(const Eigen::Vector3d& v);
  static double clamp(double value, double lower, double upper);

private:
  CameraParams params_;
};

}  // namespace scope