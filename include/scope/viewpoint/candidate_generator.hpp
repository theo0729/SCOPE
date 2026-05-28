#pragma once

#include <string>
#include <vector>

#include "scope/common/parameters.hpp"
#include "scope/common/types.hpp"

namespace scope
{

struct CandidateGenerationResult
{
  bool success = false;
  std::string message;

  std::vector<ViewpointCandidate> candidates;

  std::size_t input_surface_elements = 0;
  std::size_t skipped_invalid_surface = 0;
  std::size_t skipped_by_sampling_stride = 0;
  std::size_t skipped_by_height = 0;
  std::size_t generated_candidates = 0;
};

class CandidateGenerator
{
public:
  CandidateGenerator() = default;
  explicit CandidateGenerator(const ViewpointParams& params);

  ~CandidateGenerator() = default;

  void setParams(const ViewpointParams& params);
  const ViewpointParams& params() const;

  CandidateGenerationResult generateFromSurfaceElements(
      const std::vector<SurfaceElement>& surface_elements) const;

private:
  static bool isValidSurfaceElement(const SurfaceElement& element);

  static double computeYawFromDirection(const Eigen::Vector3d& direction);
  static double computePitchFromDirection(const Eigen::Vector3d& direction);

  ViewpointCandidate createCandidate(const SurfaceElement& element,
                                     double distance,
                                     std::uint32_t candidate_id) const;

private:
  ViewpointParams params_;
};

}  // namespace scope