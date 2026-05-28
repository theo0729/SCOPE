#pragma once

#include <string>

#include "scope/common/types.hpp"

namespace scope
{

struct PcdLoadResult
{
  bool success = false;
  std::string message;

  PointCloudPtr cloud;
  CloudStatistics statistics;
};

class PcdLoader
{
public:
  PcdLoader() = default;
  ~PcdLoader() = default;

  static PcdLoadResult loadXYZ(const std::string& pcd_path);

  static bool loadXYZ(const std::string& pcd_path,
                      PointCloudPtr* cloud,
                      std::string* error_message = nullptr);

  static CloudStatistics computeStatistics(const PointCloudConstPtr& cloud);

private:
  static bool fileExists(const std::string& path);
};

}  // namespace scope