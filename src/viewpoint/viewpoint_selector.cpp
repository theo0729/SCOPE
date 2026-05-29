#include "scope/viewpoint/viewpoint_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace scope
{

ViewpointSelector::ViewpointSelector(const ScoringParams& params)
  : params_(params)
{
}

void ViewpointSelector::setParams(const ScoringParams& params)
{
  params_ = params;
}

const ScoringParams& ViewpointSelector::params() const
{
  return params_;
}

ViewpointScoringResult ViewpointSelector::scoreAndRankCandidates(
    const std::vector<ViewpointCandidate>& candidates) const
{
  ViewpointScoringResult result;
  result.input_candidates = candidates.size();

  if (candidates.empty())
  {
    result.success = false;
    result.message = "Input candidate list is empty.";
    return result;
  }

  result.max_coverage_gain = computeMaxCoverageGain(candidates);

  result.ranked_candidates.reserve(candidates.size());

  double score_sum = 0.0;
  result.min_score = std::numeric_limits<double>::infinity();
  result.max_score = -std::numeric_limits<double>::infinity();

  for (const auto& input_candidate : candidates)
  {
    if (!input_candidate.is_valid)
    {
      continue;
    }

    ViewpointCandidate candidate = input_candidate;

    if (params_.enable_scoring)
    {
      candidate.score =
          computeCandidateScore(candidate, result.max_coverage_gain);
    }
    else
    {
      candidate.score = candidate.coverage_gain;
    }

    result.min_score = std::min(result.min_score, candidate.score);
    result.max_score = std::max(result.max_score, candidate.score);
    score_sum += candidate.score;

    result.ranked_candidates.push_back(candidate);
  }

  if (result.ranked_candidates.empty())
  {
    result.success = false;
    result.message = "No valid candidates remained for scoring.";
    return result;
  }

  std::sort(result.ranked_candidates.begin(),
            result.ranked_candidates.end(),
            [](const ViewpointCandidate& a, const ViewpointCandidate& b)
            {
              return a.score > b.score;
            });

  result.ranked_count = result.ranked_candidates.size();
  result.mean_score = score_sum / static_cast<double>(result.ranked_count);

  const std::size_t top_num =
      std::min<std::size_t>(static_cast<std::size_t>(params_.top_candidate_num),
                            result.ranked_candidates.size());

  result.top_candidates.assign(result.ranked_candidates.begin(),
                               result.ranked_candidates.begin() + top_num);

  result.top_count = result.top_candidates.size();

  if (!std::isfinite(result.min_score))
  {
    result.min_score = 0.0;
  }

  if (!std::isfinite(result.max_score))
  {
    result.max_score = 0.0;
  }

  std::ostringstream oss;
  oss << "Viewpoint scoring finished. "
      << "input=" << result.input_candidates
      << ", ranked=" << result.ranked_count
      << ", top=" << result.top_count
      << ", max_coverage_gain=" << result.max_coverage_gain
      << ", score[min/mean/max]="
      << result.min_score << "/"
      << result.mean_score << "/"
      << result.max_score;

  result.success = true;
  result.message = oss.str();

  return result;
}

double ViewpointSelector::clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

double ViewpointSelector::computeMaxCoverageGain(
    const std::vector<ViewpointCandidate>& candidates)
{
  double max_gain = 0.0;

  for (const auto& candidate : candidates)
  {
    max_gain = std::max(max_gain, candidate.coverage_gain);
  }

  return max_gain;
}

double ViewpointSelector::computeCandidateScore(
    const ViewpointCandidate& candidate,
    double max_coverage_gain) const
{
  const double coverage_score =
      max_coverage_gain > 1.0e-12
          ? candidate.coverage_gain / max_coverage_gain
          : 0.0;

  const double clearance_score =
      std::isfinite(candidate.safety_clearance)
          ? clamp(candidate.safety_clearance / params_.max_clearance_for_score,
                  0.0,
                  1.0)
          : 0.0;

  double distance_score = 0.0;

  if (params_.ideal_view_distance > 1.0e-12 &&
      std::isfinite(candidate.view_distance))
  {
    const double distance_error =
        std::abs(candidate.view_distance - params_.ideal_view_distance);

    distance_score =
        clamp(1.0 - distance_error / params_.ideal_view_distance,
              0.0,
              1.0);
  }

  const double incidence_score =
      std::isfinite(candidate.incidence_angle_rad)
          ? clamp(1.0 - candidate.incidence_angle_rad / (0.5 * M_PI),
                  0.0,
                  1.0)
          : 0.0;

  const double score =
      params_.weight_coverage * coverage_score +
      params_.weight_clearance * clearance_score +
      params_.weight_distance * distance_score +
      params_.weight_incidence * incidence_score;

  return score;
}

}  // namespace scope