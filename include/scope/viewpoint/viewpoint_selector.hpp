#pragma once

#include <string>
#include <vector>

#include "scope/common/parameters.hpp"
#include "scope/common/types.hpp"

namespace scope
{

struct ViewpointScoringResult
{
  bool success = false;
  std::string message;

  std::vector<ViewpointCandidate> ranked_candidates;
  std::vector<ViewpointCandidate> top_candidates;

  std::size_t input_candidates = 0;
  std::size_t ranked_count = 0;
  std::size_t top_count = 0;

  double max_coverage_gain = 0.0;
  double min_score = 0.0;
  double max_score = 0.0;
  double mean_score = 0.0;
};

class ViewpointSelector
{
public:
  ViewpointSelector() = default;
  explicit ViewpointSelector(const ScoringParams& params);

  ~ViewpointSelector() = default;

  void setParams(const ScoringParams& params);
  const ScoringParams& params() const;

  ViewpointScoringResult scoreAndRankCandidates(
      const std::vector<ViewpointCandidate>& candidates) const;

private:
  static double clamp(double value, double lower, double upper);

  static double computeMaxCoverageGain(
      const std::vector<ViewpointCandidate>& candidates);

  double computeCandidateScore(const ViewpointCandidate& candidate,
                               double max_coverage_gain) const;

private:
  ScoringParams params_;
};

}  // namespace scope