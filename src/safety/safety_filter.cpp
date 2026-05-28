#include "scope/safety/safety_filter.hpp"

#include <cmath>
#include <limits>
#include <sstream>

namespace scope
{

SafetyFilter::SafetyFilter(const SafetyParams& params)
  : params_(params)
{
}

void SafetyFilter::setParams(const SafetyParams& params)
{
  params_ = params;
}

const SafetyParams& SafetyFilter::params() const
{
  return params_;
}

SafetyFilterResult SafetyFilter::filterCandidates(
    const std::vector<ViewpointCandidate>& candidates,
    const PointCloudConstPtr& obstacle_cloud) const
{
  SafetyFilterResult result;
  result.input_candidates = candidates.size();

  if (candidates.empty())
  {
    result.success = false;
    result.message = "Input candidate list is empty.";
    return result;
  }

  if (!params_.enable_safety_filter)
  {
    result.safe_candidates = candidates;

    for (auto& candidate : result.safe_candidates)
    {
      candidate.is_safe = true;
      candidate.safety_clearance = std::numeric_limits<double>::infinity();
    }

    result.safe_count = result.safe_candidates.size();
    result.unsafe_count = 0;
    result.success = true;
    result.message = "Safety filter is disabled. All candidates are kept as safe.";
    return result;
  }

  CollisionChecker collision_checker;
  std::string map_message;

  if (!collision_checker.setObstacleCloud(obstacle_cloud, &map_message))
  {
    result.success = false;
    result.message = "Failed to initialize collision checker. " + map_message;
    return result;
  }

  double clearance_sum = 0.0;
  std::size_t valid_clearance_count = 0;

  result.min_clearance = std::numeric_limits<double>::infinity();
  result.max_clearance = 0.0;

  result.safe_candidates.reserve(candidates.size());
  result.unsafe_candidates.reserve(candidates.size());

  for (const auto& input_candidate : candidates)
  {
    ViewpointCandidate candidate = input_candidate;

    if (!isFiniteCandidatePosition(candidate))
    {
      candidate.is_safe = false;
      candidate.is_valid = false;
      candidate.safety_clearance = std::numeric_limits<double>::quiet_NaN();

      ++result.invalid_position_count;
      result.unsafe_candidates.push_back(candidate);
      continue;
    }

    const ClearanceQueryResult clearance =
        collision_checker.queryNearestDistance(candidate.position);

    if (!clearance.success)
    {
      candidate.is_safe = false;
      candidate.safety_clearance =
          std::numeric_limits<double>::quiet_NaN();

      ++result.query_failed_count;
      result.unsafe_candidates.push_back(candidate);
      continue;
    }

    candidate.safety_clearance = clearance.distance;

    result.min_clearance = std::min(result.min_clearance, clearance.distance);
    result.max_clearance = std::max(result.max_clearance, clearance.distance);
    clearance_sum += clearance.distance;
    ++valid_clearance_count;

    if (clearance.distance < params_.min_clearance)
    {
      candidate.is_safe = false;

      ++result.below_clearance_count;
      result.unsafe_candidates.push_back(candidate);
      continue;
    }

    candidate.is_safe = true;
    result.safe_candidates.push_back(candidate);
  }

  result.safe_count = result.safe_candidates.size();
  result.unsafe_count = result.unsafe_candidates.size();

  if (valid_clearance_count > 0)
  {
    result.mean_clearance =
        clearance_sum / static_cast<double>(valid_clearance_count);
  }

  if (!std::isfinite(result.min_clearance))
  {
    result.min_clearance = 0.0;
  }

  std::ostringstream oss;
  oss << "Safety filtering finished. "
      << "input=" << result.input_candidates
      << ", safe=" << result.safe_count
      << ", unsafe=" << result.unsafe_count
      << ", invalid_position=" << result.invalid_position_count
      << ", query_failed=" << result.query_failed_count
      << ", below_clearance=" << result.below_clearance_count
      << ", min_clearance_threshold=" << params_.min_clearance
      << ", clearance[min/mean/max]="
      << result.min_clearance << "/"
      << result.mean_clearance << "/"
      << result.max_clearance;

  result.success = true;
  result.message = oss.str();

  return result;
}

bool SafetyFilter::isFiniteCandidatePosition(
    const ViewpointCandidate& candidate)
{
  return std::isfinite(candidate.position.x()) &&
         std::isfinite(candidate.position.y()) &&
         std::isfinite(candidate.position.z());
}

}  // namespace scope