#pragma once

#include <string>
#include <vector>

#include "scope/common/parameters.hpp"
#include "scope/common/types.hpp"
#include "scope/viewpoint/fov_model.hpp"

namespace scope
{

struct CoverageEvaluationResult
{
  bool success = false;
  std::string message;

  std::vector<ViewpointCandidate> evaluated_candidates;
  std::vector<ViewpointCandidate> fov_valid_candidates;

  std::size_t input_candidates = 0;
  std::size_t input_surface_elements = 0;

  std::size_t fov_valid_candidate_count = 0;
  std::size_t fov_invalid_candidate_count = 0;

  std::size_t covered_surface_union_count = 0;
  double covered_surface_union_weight = 0.0;
  double total_surface_weight = 0.0;
  double estimated_surface_coverage_ratio = 0.0;

  double max_coverage_gain = 0.0;
  double mean_coverage_gain = 0.0;
};

class CoverageEvaluator
{
public:
  CoverageEvaluator() = default;
  CoverageEvaluator(const CameraParams& camera_params,
                    const CoverageParams& coverage_params);

  ~CoverageEvaluator() = default;

  void setCameraParams(const CameraParams& params);
  void setCoverageParams(const CoverageParams& params);

  CoverageEvaluationResult evaluateCandidates(
      const std::vector<ViewpointCandidate>& candidates,
      const std::vector<SurfaceElement>& surface_elements) const;

private:
  static double computeTotalSurfaceWeight(
      const std::vector<SurfaceElement>& surface_elements);

private:
  CameraParams camera_params_;
  CoverageParams coverage_params_;
};

}  // namespace scope