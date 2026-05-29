#include "scope/viewpoint/coverage_evaluator.hpp"

#include <algorithm>
#include <sstream>

namespace scope
{

CoverageEvaluator::CoverageEvaluator(const CameraParams& camera_params,
                                     const CoverageParams& coverage_params)
  : camera_params_(camera_params),
    coverage_params_(coverage_params)
{
}

void CoverageEvaluator::setCameraParams(const CameraParams& params)
{
  camera_params_ = params;
}

void CoverageEvaluator::setCoverageParams(const CoverageParams& params)
{
  coverage_params_ = params;
}

CoverageEvaluationResult CoverageEvaluator::evaluateCandidates(
    const std::vector<ViewpointCandidate>& candidates,
    const std::vector<SurfaceElement>& surface_elements) const
{
  CoverageEvaluationResult result;

  result.input_candidates = candidates.size();
  result.input_surface_elements = surface_elements.size();

  if (candidates.empty())
  {
    result.success = false;
    result.message = "Input candidate list is empty.";
    return result;
  }

  if (surface_elements.empty())
  {
    result.success = false;
    result.message = "Input surface element list is empty.";
    return result;
  }

  result.total_surface_weight = computeTotalSurfaceWeight(surface_elements);

  std::vector<bool> covered_union(surface_elements.size(), false);

  FovModel fov_model(camera_params_);

  double coverage_gain_sum = 0.0;

  result.evaluated_candidates.reserve(candidates.size());
  result.fov_valid_candidates.reserve(candidates.size());

  for (const auto& input_candidate : candidates)
  {
    ViewpointCandidate candidate = input_candidate;

    candidate.covered_surface_indices.clear();
    candidate.coverage_gain = 0.0;
    candidate.score = 0.0;

    if (!coverage_params_.enable_fov_filter)
    {
      candidate.is_valid = true;
      candidate.score = candidate.safety_clearance;
      result.evaluated_candidates.push_back(candidate);
      result.fov_valid_candidates.push_back(candidate);
      continue;
    }

    for (const auto& element : surface_elements)
    {
      const FovCheckResult fov_result =
          fov_model.checkSurfaceElement(candidate,
                                        element,
                                        coverage_params_.max_incidence_angle_deg);

      if (!fov_result.visible)
      {
        continue;
      }

      candidate.covered_surface_indices.push_back(element.id);
      candidate.coverage_gain += element.weight;

      if (element.id < covered_union.size())
      {
        covered_union[element.id] = true;
      }
    }

    candidate.score = candidate.coverage_gain;

    result.max_coverage_gain =
        std::max(result.max_coverage_gain, candidate.coverage_gain);

    coverage_gain_sum += candidate.coverage_gain;

    if (static_cast<int>(candidate.covered_surface_indices.size()) >=
        coverage_params_.min_visible_surface_points)
    {
      candidate.is_valid = true;
      result.fov_valid_candidates.push_back(candidate);
    }
    else
    {
      candidate.is_valid = false;
      ++result.fov_invalid_candidate_count;
    }

    result.evaluated_candidates.push_back(candidate);
  }

  result.fov_valid_candidate_count = result.fov_valid_candidates.size();

  if (!result.evaluated_candidates.empty())
  {
    result.mean_coverage_gain =
        coverage_gain_sum / static_cast<double>(result.evaluated_candidates.size());
  }

  for (std::size_t i = 0; i < surface_elements.size(); ++i)
  {
    if (!covered_union[i])
    {
      continue;
    }

    ++result.covered_surface_union_count;
    result.covered_surface_union_weight += surface_elements[i].weight;
  }

  if (result.total_surface_weight > 1.0e-12)
  {
    result.estimated_surface_coverage_ratio =
        result.covered_surface_union_weight / result.total_surface_weight;
  }

  std::ostringstream oss;
  oss << "Coverage evaluation finished. "
      << "input_candidates=" << result.input_candidates
      << ", surface_elements=" << result.input_surface_elements
      << ", fov_valid_candidates=" << result.fov_valid_candidate_count
      << ", fov_invalid_candidates=" << result.fov_invalid_candidate_count
      << ", covered_surface_union=" << result.covered_surface_union_count
      << ", estimated_coverage_ratio=" << result.estimated_surface_coverage_ratio
      << ", coverage_gain[max/mean]="
      << result.max_coverage_gain << "/"
      << result.mean_coverage_gain;

  result.success = true;
  result.message = oss.str();

  return result;
}

double CoverageEvaluator::computeTotalSurfaceWeight(
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