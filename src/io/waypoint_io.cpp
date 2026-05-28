#include "scope/io/waypoint_io.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace scope
{

namespace fs = std::filesystem;

CandidateExportResult WaypointIO::exportCandidateDiagnostics(
    const OutputParams& output_params,
    const CandidateGenerationResult& candidate_result,
    const SafetyFilterResult& safety_result,
    const std::string& frame_id)
{
  CandidateExportResult result;

  std::string error_message;
  if (!ensureDirectory(output_params.output_dir, &error_message))
  {
    result.success = false;
    result.message = "Failed to create output directory. " + error_message;
    return result;
  }

  result.statistics_json_path =
      joinPath(output_params.output_dir, output_params.candidate_statistics_json);
  result.safe_csv_path =
      joinPath(output_params.output_dir, output_params.safe_candidates_csv);
  result.unsafe_csv_path =
      joinPath(output_params.output_dir, output_params.unsafe_candidates_csv);
  result.safe_yaml_path =
      joinPath(output_params.output_dir, output_params.safe_candidates_yaml);
  result.unsafe_yaml_path =
      joinPath(output_params.output_dir, output_params.unsafe_candidates_yaml);

  if (!writeCandidateStatisticsJson(candidate_result,
                                    safety_result,
                                    result.statistics_json_path,
                                    frame_id,
                                    &error_message))
  {
    result.success = false;
    result.message = "Failed to write candidate statistics json. " + error_message;
    return result;
  }

  if (!writeCandidatesCsv(safety_result.safe_candidates,
                          result.safe_csv_path,
                          &error_message))
  {
    result.success = false;
    result.message = "Failed to write safe candidate csv. " + error_message;
    return result;
  }

  if (!writeCandidatesCsv(safety_result.unsafe_candidates,
                          result.unsafe_csv_path,
                          &error_message))
  {
    result.success = false;
    result.message = "Failed to write unsafe candidate csv. " + error_message;
    return result;
  }

  if (!writeCandidatesYaml(safety_result.safe_candidates,
                           result.safe_yaml_path,
                           frame_id,
                           "safe",
                           &error_message))
  {
    result.success = false;
    result.message = "Failed to write safe candidate yaml. " + error_message;
    return result;
  }

  if (!writeCandidatesYaml(safety_result.unsafe_candidates,
                           result.unsafe_yaml_path,
                           frame_id,
                           "unsafe",
                           &error_message))
  {
    result.success = false;
    result.message = "Failed to write unsafe candidate yaml. " + error_message;
    return result;
  }

  std::ostringstream oss;
  oss << "Candidate diagnostics exported to directory: "
      << output_params.output_dir;

  result.success = true;
  result.message = oss.str();

  return result;
}

bool WaypointIO::writeCandidatesCsv(
    const std::vector<ViewpointCandidate>& candidates,
    const std::string& file_path,
    std::string* error_message)
{
  std::ofstream file(file_path);

  if (!file.is_open())
  {
    if (error_message)
    {
      *error_message = "Cannot open file: " + file_path;
    }
    return false;
  }

  file << std::fixed << std::setprecision(6);

  file
      << "id,"
      << "is_safe,is_valid,"
      << "x,y,z,"
      << "target_x,target_y,target_z,"
      << "view_dir_x,view_dir_y,view_dir_z,"
      << "yaw_deg,pitch_deg,pitch_enabled,"
      << "view_distance,incidence_angle_deg,"
      << "safety_clearance,coverage_gain,score,"
      << "covered_surface_count,first_covered_surface_id\n";

  for (const auto& c : candidates)
  {
    const int first_surface_id =
        c.covered_surface_indices.empty()
            ? -1
            : static_cast<int>(c.covered_surface_indices.front());

    file
        << c.id << ","
        << boolToString(c.is_safe) << ","
        << boolToString(c.is_valid) << ","
        << c.position.x() << ","
        << c.position.y() << ","
        << c.position.z() << ","
        << c.target_point.x() << ","
        << c.target_point.y() << ","
        << c.target_point.z() << ","
        << c.viewing_direction.x() << ","
        << c.viewing_direction.y() << ","
        << c.viewing_direction.z() << ","
        << rad2deg(c.yaw_rad) << ","
        << rad2deg(c.pitch_rad) << ","
        << boolToString(c.pitch_enabled) << ","
        << c.view_distance << ","
        << rad2deg(c.incidence_angle_rad) << ","
        << c.safety_clearance << ","
        << c.coverage_gain << ","
        << c.score << ","
        << c.covered_surface_indices.size() << ","
        << first_surface_id << "\n";
  }

  return true;
}

bool WaypointIO::writeCandidatesYaml(
    const std::vector<ViewpointCandidate>& candidates,
    const std::string& file_path,
    const std::string& frame_id,
    const std::string& candidate_type,
    std::string* error_message)
{
  YAML::Emitter out;

  out << YAML::BeginMap;

  out << YAML::Key << "generated_by" << YAML::Value << "SCOPE";
  out << YAML::Key << "candidate_type" << YAML::Value << candidate_type;
  out << YAML::Key << "frame_id" << YAML::Value << frame_id;
  out << YAML::Key << "candidate_count" << YAML::Value
      << static_cast<int>(candidates.size());

  out << YAML::Key << "candidates" << YAML::Value << YAML::BeginSeq;

  for (const auto& c : candidates)
  {
    out << YAML::BeginMap;

    out << YAML::Key << "id" << YAML::Value << static_cast<int>(c.id);
    out << YAML::Key << "is_safe" << YAML::Value << c.is_safe;
    out << YAML::Key << "is_valid" << YAML::Value << c.is_valid;

    out << YAML::Key << "position" << YAML::Value << YAML::Flow
        << YAML::BeginSeq
        << c.position.x()
        << c.position.y()
        << c.position.z()
        << YAML::EndSeq;

    out << YAML::Key << "target_point" << YAML::Value << YAML::Flow
        << YAML::BeginSeq
        << c.target_point.x()
        << c.target_point.y()
        << c.target_point.z()
        << YAML::EndSeq;

    out << YAML::Key << "viewing_direction" << YAML::Value << YAML::Flow
        << YAML::BeginSeq
        << c.viewing_direction.x()
        << c.viewing_direction.y()
        << c.viewing_direction.z()
        << YAML::EndSeq;

    out << YAML::Key << "yaw_deg" << YAML::Value << rad2deg(c.yaw_rad);
    out << YAML::Key << "pitch_deg" << YAML::Value << rad2deg(c.pitch_rad);
    out << YAML::Key << "pitch_enabled" << YAML::Value << c.pitch_enabled;

    out << YAML::Key << "view_distance" << YAML::Value << c.view_distance;
    out << YAML::Key << "incidence_angle_deg" << YAML::Value
        << rad2deg(c.incidence_angle_rad);

    out << YAML::Key << "safety_clearance" << YAML::Value
        << c.safety_clearance;

    out << YAML::Key << "coverage_gain" << YAML::Value << c.coverage_gain;
    out << YAML::Key << "score" << YAML::Value << c.score;

    out << YAML::Key << "covered_surface_indices" << YAML::Value
        << YAML::Flow << YAML::BeginSeq;

    for (const auto idx : c.covered_surface_indices)
    {
      out << static_cast<int>(idx);
    }

    out << YAML::EndSeq;

    out << YAML::EndMap;
  }

  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream file(file_path);

  if (!file.is_open())
  {
    if (error_message)
    {
      *error_message = "Cannot open file: " + file_path;
    }
    return false;
  }

  file << out.c_str();
  return true;
}

bool WaypointIO::writeCandidateStatisticsJson(
    const CandidateGenerationResult& candidate_result,
    const SafetyFilterResult& safety_result,
    const std::string& file_path,
    const std::string& frame_id,
    std::string* error_message)
{
  std::ofstream file(file_path);

  if (!file.is_open())
  {
    if (error_message)
    {
      *error_message = "Cannot open file: " + file_path;
    }
    return false;
  }

  file << std::fixed << std::setprecision(6);

  file << "{\n";
  file << "  \"generated_by\": \"SCOPE\",\n";
  file << "  \"version\": \"v0.2.3\",\n";
  file << "  \"frame_id\": \"" << frame_id << "\",\n";

  file << "  \"candidate_generation\": {\n";
  file << "    \"success\": " << (candidate_result.success ? "true" : "false") << ",\n";
  file << "    \"message\": \"" << candidate_result.message << "\",\n";
  file << "    \"input_surface_elements\": "
       << candidate_result.input_surface_elements << ",\n";
  file << "    \"skipped_invalid_surface\": "
       << candidate_result.skipped_invalid_surface << ",\n";
  file << "    \"skipped_by_sampling_stride\": "
       << candidate_result.skipped_by_sampling_stride << ",\n";
  file << "    \"skipped_by_height\": "
       << candidate_result.skipped_by_height << ",\n";
  file << "    \"generated_candidates\": "
       << candidate_result.generated_candidates << "\n";
  file << "  },\n";

  file << "  \"safety_filtering\": {\n";
  file << "    \"success\": " << (safety_result.success ? "true" : "false") << ",\n";
  file << "    \"message\": \"" << safety_result.message << "\",\n";
  file << "    \"input_candidates\": "
       << safety_result.input_candidates << ",\n";
  file << "    \"safe_count\": " << safety_result.safe_count << ",\n";
  file << "    \"unsafe_count\": " << safety_result.unsafe_count << ",\n";
  file << "    \"invalid_position_count\": "
       << safety_result.invalid_position_count << ",\n";
  file << "    \"query_failed_count\": "
       << safety_result.query_failed_count << ",\n";
  file << "    \"below_clearance_count\": "
       << safety_result.below_clearance_count << ",\n";
  file << "    \"min_clearance\": "
       << safety_result.min_clearance << ",\n";
  file << "    \"mean_clearance\": "
       << safety_result.mean_clearance << ",\n";
  file << "    \"max_clearance\": "
       << safety_result.max_clearance << "\n";
  file << "  }\n";

  file << "}\n";

  return true;
}

bool WaypointIO::ensureDirectory(const std::string& directory,
                                 std::string* error_message)
{
  try
  {
    if (directory.empty())
    {
      if (error_message)
      {
        *error_message = "Directory path is empty.";
      }
      return false;
    }

    fs::path dir_path(directory);

    if (fs::exists(dir_path))
    {
      if (!fs::is_directory(dir_path))
      {
        if (error_message)
        {
          *error_message = "Path exists but is not a directory: " + directory;
        }
        return false;
      }

      return true;
    }

    return fs::create_directories(dir_path);
  }
  catch (const std::exception& e)
  {
    if (error_message)
    {
      *error_message = e.what();
    }
    return false;
  }
}

std::string WaypointIO::joinPath(const std::string& directory,
                                 const std::string& filename)
{
  fs::path p = fs::path(directory) / fs::path(filename);
  return p.string();
}

std::string WaypointIO::boolToString(bool value)
{
  return value ? "true" : "false";
}

}  // namespace scope