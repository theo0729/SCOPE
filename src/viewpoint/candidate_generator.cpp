#include "scope/viewpoint/candidate_generator.hpp"

#include <cmath>
#include <sstream>

namespace scope
{

CandidateGenerator::CandidateGenerator(const ViewpointParams& params)
  : params_(params)
{
}

void CandidateGenerator::setParams(const ViewpointParams& params)
{
  params_ = params;
}

const ViewpointParams& CandidateGenerator::params() const
{
  return params_;
}

CandidateGenerationResult CandidateGenerator::generateFromSurfaceElements(
    const std::vector<SurfaceElement>& surface_elements) const
{
  CandidateGenerationResult result;

  result.input_surface_elements = surface_elements.size();

  if (surface_elements.empty())
  {
    result.success = false;
    result.message = "Input surface element list is empty.";
    return result;
  }

  if (params_.candidate_distances.empty())
  {
    result.success = false;
    result.message = "candidate_distances is empty.";
    return result;
  }

  std::size_t possible_candidate_num =
      surface_elements.size() * params_.candidate_distances.size();

  std::size_t sampling_stride = 1;

  if (params_.max_candidate_num > 0 &&
      possible_candidate_num > static_cast<std::size_t>(params_.max_candidate_num))
  {
    sampling_stride =
        static_cast<std::size_t>(
            std::ceil(static_cast<double>(possible_candidate_num) /
                      static_cast<double>(params_.max_candidate_num)));
  }

  result.candidates.reserve(
      std::min<std::size_t>(possible_candidate_num,
                            static_cast<std::size_t>(params_.max_candidate_num)));

  std::size_t flat_index = 0;

  for (const auto& element : surface_elements)
  {
    if (!isValidSurfaceElement(element))
    {
      ++result.skipped_invalid_surface;
      flat_index += params_.candidate_distances.size();
      continue;
    }

    for (const double distance : params_.candidate_distances)
    {
      if (sampling_stride > 1 && (flat_index % sampling_stride != 0))
      {
        ++result.skipped_by_sampling_stride;
        ++flat_index;
        continue;
      }

      if (distance <= 0.0 || !std::isfinite(distance))
      {
        ++result.skipped_invalid_surface;
        ++flat_index;
        continue;
      }

      if (params_.max_candidate_num > 0 &&
          result.candidates.size() >= static_cast<std::size_t>(params_.max_candidate_num))
      {
        ++flat_index;
        continue;
      }

      ViewpointCandidate candidate =
          createCandidate(element,
                          distance,
                          static_cast<std::uint32_t>(result.candidates.size()));

      if (candidate.position.z() < params_.min_height ||
          candidate.position.z() > params_.max_height)
      {
        ++result.skipped_by_height;
        ++flat_index;
        continue;
      }

      result.candidates.push_back(candidate);
      ++flat_index;
    }
  }

  result.generated_candidates = result.candidates.size();

  if (result.candidates.empty())
  {
    result.success = false;
    result.message = "No candidate viewpoints were generated.";
    return result;
  }

  std::ostringstream oss;
  oss << "Candidate generation finished. "
      << "surface_elements=" << result.input_surface_elements
      << ", candidate_distances=" << params_.candidate_distances.size()
      << ", sampling_stride=" << sampling_stride
      << ", skipped_invalid_surface=" << result.skipped_invalid_surface
      << ", skipped_by_sampling_stride=" << result.skipped_by_sampling_stride
      << ", skipped_by_height=" << result.skipped_by_height
      << ", generated=" << result.generated_candidates;

  result.success = true;
  result.message = oss.str();

  return result;
}

bool CandidateGenerator::isValidSurfaceElement(const SurfaceElement& element)
{
  if (!std::isfinite(element.position.x()) ||
      !std::isfinite(element.position.y()) ||
      !std::isfinite(element.position.z()))
  {
    return false;
  }

  if (!std::isfinite(element.normal.x()) ||
      !std::isfinite(element.normal.y()) ||
      !std::isfinite(element.normal.z()))
  {
    return false;
  }

  return element.normal.norm() > 1.0e-9;
}

double CandidateGenerator::computeYawFromDirection(const Eigen::Vector3d& direction)
{
  return std::atan2(direction.y(), direction.x());
}

double CandidateGenerator::computePitchFromDirection(const Eigen::Vector3d& direction)
{
  const double horizontal_norm =
      std::sqrt(direction.x() * direction.x() + direction.y() * direction.y());

  return std::atan2(direction.z(), horizontal_norm);
}

ViewpointCandidate CandidateGenerator::createCandidate(
    const SurfaceElement& element,
    double distance,
    std::uint32_t candidate_id) const
{
  ViewpointCandidate candidate;

  Eigen::Vector3d normal = element.normal;
  normal.normalize();

  candidate.id = candidate_id;

  candidate.target_point = element.position;

  // For point-cloud based target surfaces, the candidate is generated along the
  // estimated surface normal. Whether the normal points to the true free-space
  // side will be verified later by the safety filter.
  candidate.position = element.position + normal * distance;

  Eigen::Vector3d viewing_direction =
      candidate.target_point - candidate.position;

  if (viewing_direction.norm() > 1.0e-9)
  {
    viewing_direction.normalize();
  }
  else
  {
    viewing_direction = -normal;
  }

  candidate.viewing_direction = viewing_direction;

  candidate.yaw_rad = computeYawFromDirection(viewing_direction);
  candidate.pitch_rad = computePitchFromDirection(viewing_direction);

  candidate.pitch_enabled = params_.enable_pitch_planning;

  candidate.view_distance = distance;

  // The ideal viewing direction is approximately opposite to the surface normal.
  const double cos_incidence =
      std::max(-1.0, std::min(1.0, (-viewing_direction).dot(normal)));

  candidate.incidence_angle_rad = std::acos(cos_incidence);

  candidate.coverage_gain = 0.0;
  candidate.safety_clearance = std::numeric_limits<double>::infinity();
  candidate.score = 0.0;

  candidate.is_safe = true;
  candidate.is_valid = true;

  candidate.covered_surface_indices.push_back(element.id);

  return candidate;
}

}  // namespace scope