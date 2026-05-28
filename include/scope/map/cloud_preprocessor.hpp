#pragma once

#include <string>

#include "scope/common/parameters.hpp"
#include "scope/common/types.hpp"

namespace scope
{

struct CloudPreprocessResult
{
  bool success = false;
  std::string message;

  PointCloudPtr cloud;
  CloudStatistics statistics;

  std::size_t input_points = 0;
  std::size_t after_invalid_removal = 0;
  std::size_t after_voxel_filter = 0;
  std::size_t after_roi_filter = 0;
  std::size_t after_outlier_removal = 0;
};

class CloudPreprocessor
{
public:
  CloudPreprocessor() = default;
  explicit CloudPreprocessor(const PreprocessParams& params);

  ~CloudPreprocessor() = default;

  void setParams(const PreprocessParams& params);
  const PreprocessParams& params() const;

  CloudPreprocessResult process(const PointCloudConstPtr& input_cloud) const;

private:
  PointCloudPtr removeInvalidPoints(const PointCloudConstPtr& input_cloud) const;
  PointCloudPtr applyVoxelFilter(const PointCloudConstPtr& input_cloud) const;
  PointCloudPtr applyRoiFilter(const PointCloudConstPtr& input_cloud) const;
  PointCloudPtr applyOutlierRemoval(const PointCloudConstPtr& input_cloud) const;

private:
  PreprocessParams params_;
};

}  // namespace scope