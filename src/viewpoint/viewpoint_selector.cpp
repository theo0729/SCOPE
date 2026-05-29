#include "scope/viewpoint/viewpoint_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

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

GreedyCoverageSelectionResult ViewpointSelector::selectGreedyCoverageCandidates(
    const std::vector<ViewpointCandidate>& ranked_candidates,
    const std::vector<SurfaceElement>& surface_elements,
    const CoverageParams& coverage_params) const
{
  GreedyCoverageSelectionResult result;

  result.input_candidates = ranked_candidates.size();
  result.input_surface_elements = surface_elements.size();
  result.total_surface_weight = computeTotalSurfaceWeight(surface_elements);

  if (ranked_candidates.empty())
  {
    result.success = false;
    result.message = "Input ranked candidate list is empty.";
    return result;
  }

  if (surface_elements.empty())
  {
    result.success = false;
    result.message = "Input surface element list is empty.";
    return result;
  }

  if (result.total_surface_weight <= 1.0e-12)
  {
    result.success = false;
    result.message = "Total surface weight is too small.";
    return result;
  }

  const int required_redundancy =
      std::max(1, coverage_params.min_view_redundancy);

  std::vector<int> covered_counts(surface_elements.size(), 0);
  std::vector<bool> used_candidates(ranked_candidates.size(), false);

  result.selected_candidates.reserve(
      static_cast<std::size_t>(coverage_params.max_selected_viewpoint_num));

  for (int iter = 0; iter < coverage_params.max_selected_viewpoint_num; ++iter)
  {
    int best_index = -1;
    std::size_t best_new_count = 0;
    double best_new_weight = 0.0;
    double best_score = -std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < ranked_candidates.size(); ++i)
    {
      if (used_candidates[i])
      {
        continue;
      }

      const auto& candidate = ranked_candidates[i];

      if (!candidate.is_valid)
      {
        continue;
      }

      std::size_t new_count = 0;
      double new_weight = 0.0;

      for (const auto surface_id : candidate.covered_surface_indices)
      {
        if (surface_id >= surface_elements.size())
        {
          continue;
        }

        if (covered_counts[surface_id] < required_redundancy)
        {
          ++new_count;
          new_weight += surface_elements[surface_id].weight;
        }
      }

      if (new_count == 0)
      {
        continue;
      }

      const bool better_gain =
          new_weight > best_new_weight + 1.0e-12;

      const bool tie_better_score =
          std::abs(new_weight - best_new_weight) <= 1.0e-12 &&
          candidate.score > best_score;

      if (better_gain || tie_better_score)
      {
        best_index = static_cast<int>(i);
        best_new_count = new_count;
        best_new_weight = new_weight;
        best_score = candidate.score;
      }
    }

    if (best_index < 0)
    {
      result.stop_reason = 1;
      break;
    }

    if (static_cast<int>(best_new_count) < coverage_params.min_new_covered_points)
    {
      result.stop_reason = 2;
      break;
    }

    used_candidates[best_index] = true;

    ViewpointCandidate selected = ranked_candidates[best_index];

    result.selected_candidates.push_back(selected);

    for (const auto surface_id : selected.covered_surface_indices)
    {
      if (surface_id >= surface_elements.size())
      {
        continue;
      }

      const int before_count = covered_counts[surface_id];
      covered_counts[surface_id] += 1;

      if (before_count < required_redundancy &&
          covered_counts[surface_id] >= required_redundancy)
      {
        ++result.covered_surface_count;
        result.covered_surface_weight += surface_elements[surface_id].weight;
      }
    }

    result.coverage_ratio =
        result.covered_surface_weight / result.total_surface_weight;

    if (result.coverage_ratio >= coverage_params.target_coverage_ratio)
    {
      result.stop_reason = 3;
      break;
    }
  }

  result.selected_count = result.selected_candidates.size();

  if (result.selected_candidates.empty())
  {
    result.success = false;
    result.message =
        "No candidates were selected by greedy coverage selection.";
    return result;
  }

  if (result.stop_reason == 0)
  {
    result.stop_reason = 4;
  }

  std::ostringstream oss;
  oss << "Greedy coverage selection finished. "
      << "input_candidates=" << result.input_candidates
      << ", surface_elements=" << result.input_surface_elements
      << ", selected=" << result.selected_count
      << ", covered_surface=" << result.covered_surface_count
      << ", coverage_ratio=" << result.coverage_ratio
      << ", covered_weight=" << result.covered_surface_weight
      << ", total_weight=" << result.total_surface_weight
      << ", stop_reason=" << result.stop_reason;

  result.success = true;
  result.message = oss.str();

  return result;
}

double ViewpointSelector::computeTotalSurfaceWeight(
    const std::vector<SurfaceElement>& surface_elements)
{
  double total = 0.0;

  for (const auto& element : surface_elements)
  {
    total += element.weight;
  }

  return total;
}

}  // namespace scope