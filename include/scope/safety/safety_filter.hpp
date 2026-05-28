#pragma once

#include <string>
#include <vector>

#include "scope/common/parameters.hpp"
#include "scope/common/types.hpp"
#include "scope/safety/collision_checker.hpp"

namespace scope
{

struct SafetyFilterResult
{
  bool success = false;
  std::string message;

  std::vector<ViewpointCandidate> safe_candidates;
  std::vector<ViewpointCandidate> unsafe_candidates;

  std::size_t input_candidates = 0;
  std::size_t invalid_position_count = 0;
  std::size_t query_failed_count = 0;
  std::size_t below_clearance_count = 0;

  std::size_t safe_count = 0;
  std::size_t unsafe_count = 0;

  double min_clearance = std::numeric_limits<double>::infinity();
  double mean_clearance = 0.0;
  double max_clearance = 0.0;
};

class SafetyFilter
{
public:
  SafetyFilter() = default;
  explicit SafetyFilter(const SafetyParams& params);

  ~SafetyFilter() = default;

  void setParams(const SafetyParams& params);
  const SafetyParams& params() const;

  SafetyFilterResult filterCandidates(
      const std::vector<ViewpointCandidate>& candidates,
      const PointCloudConstPtr& obstacle_cloud) const;

private:
  static bool isFiniteCandidatePosition(const ViewpointCandidate& candidate);

private:
  SafetyParams params_;
};

}  // namespace scope