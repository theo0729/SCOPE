#include "scope/viewpoint/fov_model.hpp"

#include <algorithm>
#include <cmath>

namespace scope
{

FovModel::FovModel(const CameraParams& params)
  : params_(params)
{
}

void FovModel::setParams(const CameraParams& params)
{
  params_ = params;
}

const CameraParams& FovModel::params() const
{
  return params_;
}

FovCheckResult FovModel::checkSurfaceElement(
    const ViewpointCandidate& candidate,
    const SurfaceElement& element,
    double max_incidence_angle_deg) const
{
  FovCheckResult result;

  if (!params_.enable)
  {
    result.visible = false;
    result.reason = "camera disabled";
    return result;
  }

  if (!isFiniteVector3d(candidate.position) ||
      !isFiniteVector3d(candidate.viewing_direction) ||
      !isFiniteVector3d(element.position) ||
      !isFiniteVector3d(element.normal))
  {
    result.visible = false;
    result.reason = "non-finite input";
    return result;
  }

  Eigen::Vector3d forward = candidate.viewing_direction;

  if (forward.norm() < 1.0e-9)
  {
    result.visible = false;
    result.reason = "invalid viewing direction";
    return result;
  }

  forward.normalize();

  Eigen::Vector3d right;
  Eigen::Vector3d up;

  if (!buildCameraBasis(forward, &right, &up))
  {
    result.visible = false;
    result.reason = "failed to build camera basis";
    return result;
  }

  const Eigen::Vector3d rel = element.position - candidate.position;
  const double distance = rel.norm();
  result.distance = distance;

  if (distance < params_.min_view_distance ||
      distance > params_.max_view_distance)
  {
    result.visible = false;
    result.reason = "outside view distance range";
    return result;
  }

  if (distance < 1.0e-9)
  {
    result.visible = false;
    result.reason = "zero view distance";
    return result;
  }

  const Eigen::Vector3d ray_dir = rel / distance;

  const double forward_depth = rel.dot(forward);

  if (forward_depth <= 0.0)
  {
    result.visible = false;
    result.reason = "behind camera";
    return result;
  }

  result.horizontal_angle_rad =
      std::atan2(rel.dot(right), forward_depth);

  result.vertical_angle_rad =
      std::atan2(rel.dot(up), forward_depth);

  const double half_h = deg2rad(params_.fov_h_deg * 0.5);
  const double half_v = deg2rad(params_.fov_v_deg * 0.5);

  if (std::abs(result.horizontal_angle_rad) > half_h ||
      std::abs(result.vertical_angle_rad) > half_v)
  {
    result.visible = false;
    result.reason = "outside fov";
    return result;
  }

  Eigen::Vector3d normal = element.normal;

  if (normal.norm() < 1.0e-9)
  {
    result.visible = false;
    result.reason = "invalid surface normal";
    return result;
  }

  normal.normalize();

  // Incidence angle is measured between the surface normal and the direction
  // from the surface point to the camera.
  const double cos_incidence =
      clamp(normal.dot(-ray_dir), -1.0, 1.0);

  result.incidence_angle_rad = std::acos(cos_incidence);

  if (result.incidence_angle_rad > deg2rad(max_incidence_angle_deg))
  {
    result.visible = false;
    result.reason = "large incidence angle";
    return result;
  }

  result.visible = true;
  result.reason = "visible";
  return result;
}

bool FovModel::buildCameraBasis(const Eigen::Vector3d& forward,
                                Eigen::Vector3d* right,
                                Eigen::Vector3d* up)
{
  if (right == nullptr || up == nullptr)
  {
    return false;
  }

  if (forward.norm() < 1.0e-9)
  {
    return false;
  }

  Eigen::Vector3d f = forward.normalized();

  Eigen::Vector3d world_up = Eigen::Vector3d::UnitZ();

  if (std::abs(f.dot(world_up)) > 0.95)
  {
    world_up = Eigen::Vector3d::UnitY();
  }

  Eigen::Vector3d r = f.cross(world_up);

  if (r.norm() < 1.0e-9)
  {
    return false;
  }

  r.normalize();

  Eigen::Vector3d u = r.cross(f);

  if (u.norm() < 1.0e-9)
  {
    return false;
  }

  u.normalize();

  *right = r;
  *up = u;

  return true;
}

bool FovModel::isFiniteVector3d(const Eigen::Vector3d& v)
{
  return std::isfinite(v.x()) &&
         std::isfinite(v.y()) &&
         std::isfinite(v.z());
}

double FovModel::clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

}  // namespace scope