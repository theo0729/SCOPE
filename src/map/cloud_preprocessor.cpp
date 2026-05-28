#include "scope/map/cloud_preprocessor.hpp"

#include <sstream>

#include <pcl/common/point_tests.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "scope/io/pcd_loader.hpp"

namespace scope
{

CloudPreprocessor::CloudPreprocessor(const PreprocessParams& params)
  : params_(params)
{
}

void CloudPreprocessor::setParams(const PreprocessParams& params)
{
  params_ = params;
}

const PreprocessParams& CloudPreprocessor::params() const
{
  return params_;
}

CloudPreprocessResult CloudPreprocessor::process(const PointCloudConstPtr& input_cloud) const
{
  CloudPreprocessResult result;

  if (!input_cloud)
  {
    result.success = false;
    result.message = "Input cloud is null.";
    return result;
  }

  if (input_cloud->empty())
  {
    result.success = false;
    result.message = "Input cloud is empty.";
    return result;
  }

  result.input_points = input_cloud->size();

  PointCloudPtr current = removeInvalidPoints(input_cloud);
  result.after_invalid_removal = current ? current->size() : 0;

  if (!current || current->empty())
  {
    result.success = false;
    result.message = "No valid finite points after invalid point removal.";
    return result;
  }

  current = applyVoxelFilter(current);
  result.after_voxel_filter = current ? current->size() : 0;

  if (!current || current->empty())
  {
    result.success = false;
    result.message = "Point cloud is empty after voxel filtering.";
    return result;
  }

  current = applyRoiFilter(current);
  result.after_roi_filter = current ? current->size() : 0;

  if (!current || current->empty())
  {
    result.success = false;
    result.message = "Point cloud is empty after ROI filtering.";
    return result;
  }

  current = applyOutlierRemoval(current);
  result.after_outlier_removal = current ? current->size() : 0;

  if (!current || current->empty())
  {
    result.success = false;
    result.message = "Point cloud is empty after outlier removal.";
    return result;
  }

  result.cloud = current;
  result.statistics = PcdLoader::computeStatistics(current);

  if (!result.statistics.valid)
  {
    result.success = false;
    result.message = "Processed cloud has no valid finite statistics.";
    return result;
  }

  std::ostringstream oss;
  oss << "Cloud preprocessing finished. "
      << "input=" << result.input_points
      << ", valid=" << result.after_invalid_removal
      << ", voxel=" << result.after_voxel_filter
      << ", roi=" << result.after_roi_filter
      << ", outlier=" << result.after_outlier_removal;

  result.success = true;
  result.message = oss.str();

  return result;
}

PointCloudPtr CloudPreprocessor::removeInvalidPoints(const PointCloudConstPtr& input_cloud) const
{
  PointCloudPtr output(new PointCloudT);

  if (!input_cloud)
  {
    return output;
  }

  output->reserve(input_cloud->size());

  for (const auto& p : input_cloud->points)
  {
    if (pcl::isFinite(p))
    {
      output->push_back(p);
    }
  }

  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = true;

  return output;
}

PointCloudPtr CloudPreprocessor::applyVoxelFilter(const PointCloudConstPtr& input_cloud) const
{
  PointCloudPtr output(new PointCloudT);

  if (!input_cloud || input_cloud->empty())
  {
    return output;
  }

  if (params_.voxel_leaf_size <= 0.0)
  {
    *output = *input_cloud;
    return output;
  }

  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(input_cloud);
  voxel_filter.setLeafSize(static_cast<float>(params_.voxel_leaf_size),
                           static_cast<float>(params_.voxel_leaf_size),
                           static_cast<float>(params_.voxel_leaf_size));
  voxel_filter.filter(*output);

  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = true;

  return output;
}

PointCloudPtr CloudPreprocessor::applyRoiFilter(const PointCloudConstPtr& input_cloud) const
{
  PointCloudPtr output(new PointCloudT);

  if (!input_cloud || input_cloud->empty())
  {
    return output;
  }

  if (!params_.enable_roi_filter)
  {
    *output = *input_cloud;
    return output;
  }

  const auto& box = params_.roi_box;

  pcl::CropBox<PointT> crop_box;
  crop_box.setInputCloud(input_cloud);

  crop_box.setMin(Eigen::Vector4f(static_cast<float>(box.min.x()),
                                  static_cast<float>(box.min.y()),
                                  static_cast<float>(box.min.z()),
                                  1.0f));

  crop_box.setMax(Eigen::Vector4f(static_cast<float>(box.max.x()),
                                  static_cast<float>(box.max.y()),
                                  static_cast<float>(box.max.z()),
                                  1.0f));

  crop_box.filter(*output);

  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = true;

  return output;
}

PointCloudPtr CloudPreprocessor::applyOutlierRemoval(const PointCloudConstPtr& input_cloud) const
{
  PointCloudPtr output(new PointCloudT);

  if (!input_cloud || input_cloud->empty())
  {
    return output;
  }

  if (!params_.enable_outlier_removal)
  {
    *output = *input_cloud;
    return output;
  }

  if (params_.mean_k <= 0 || params_.stddev_mul_thresh <= 0.0)
  {
    *output = *input_cloud;
    return output;
  }

  if (input_cloud->size() <= static_cast<std::size_t>(params_.mean_k))
  {
    *output = *input_cloud;
    return output;
  }

  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(input_cloud);
  sor.setMeanK(params_.mean_k);
  sor.setStddevMulThresh(params_.stddev_mul_thresh);
  sor.filter(*output);

  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = true;

  return output;
}

}  // namespace scope