#pragma once

#include <string>
#include <vector>

#include "scope/common/parameters.hpp"
#include "scope/common/types.hpp"
#include "scope/safety/safety_filter.hpp"
#include "scope/viewpoint/candidate_generator.hpp"

namespace scope
{

struct CandidateExportResult
{
  bool success = false;
  std::string message;

  std::string statistics_json_path;
  std::string safe_csv_path;
  std::string unsafe_csv_path;
  std::string safe_yaml_path;
  std::string unsafe_yaml_path;
};

class WaypointIO
{
public:
  WaypointIO() = default;
  ~WaypointIO() = default;

  static CandidateExportResult exportCandidateDiagnostics(
      const OutputParams& output_params,
      const CandidateGenerationResult& candidate_result,
      const SafetyFilterResult& safety_result,
      const std::string& frame_id);

  static bool writeCandidatesCsv(
      const std::vector<ViewpointCandidate>& candidates,
      const std::string& file_path,
      std::string* error_message = nullptr);

  static bool writeCandidatesYaml(
      const std::vector<ViewpointCandidate>& candidates,
      const std::string& file_path,
      const std::string& frame_id,
      const std::string& candidate_type,
      std::string* error_message = nullptr);

  static bool writeCandidateStatisticsJson(
      const CandidateGenerationResult& candidate_result,
      const SafetyFilterResult& safety_result,
      const std::string& file_path,
      const std::string& frame_id,
      std::string* error_message = nullptr);

private:
  static bool ensureDirectory(const std::string& directory,
                              std::string* error_message = nullptr);

  static std::string joinPath(const std::string& directory,
                              const std::string& filename);

  static std::string boolToString(bool value);
};

}  // namespace scope