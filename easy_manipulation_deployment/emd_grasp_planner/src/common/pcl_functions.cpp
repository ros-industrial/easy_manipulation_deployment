// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// Main PCL files

#include "emd/common/pcl_functions.hpp"

bool PCLFunctions::passthrough_filter(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  const float & ptFilter_Ulimit_x,
  const float & ptFilter_Llimit_x,
  const float & ptFilter_Ulimit_y,
  const float & ptFilter_Llimit_y,
  const float & ptFilter_Ulimit_z,
  const float & ptFilter_Llimit_z)
{
  // Remove NaN values
  std::vector<int> nanIndices;

  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndices);

  // Remove background points
  pcl::PassThrough<pcl::PointXYZRGB> ptFilter;
  ptFilter.setInputCloud(cloud);
  ptFilter.setFilterFieldName("z");
  ptFilter.setFilterLimits(ptFilter_Llimit_z, ptFilter_Ulimit_z);
  ptFilter.filter(*cloud);

  ptFilter.setInputCloud(cloud);
  ptFilter.setFilterFieldName("y");
  ptFilter.setFilterLimits(ptFilter_Llimit_y, ptFilter_Ulimit_y);
  ptFilter.filter(*cloud);

  ptFilter.setInputCloud(cloud);
  ptFilter.setFilterFieldName("x");
  ptFilter.setFilterLimits(ptFilter_Llimit_x, ptFilter_Ulimit_x);
  ptFilter.filter(*cloud);
  return true;
}

void PCLFunctions::sensor_msg_to_pcl_pointcloud2(
  const sensor_msgs::msg::PointCloud2 & pc2,
  pcl::PCLPointCloud2 & pcl_pc2)
{
  try {
    pcl_pc2.header.stamp = pc2.header.stamp.nanosec / 1000ull;
    pcl_pc2.header.seq = 0;
    pcl_pc2.header.frame_id = pc2.header.frame_id;
    pcl_pc2.height = pc2.height;
    pcl_pc2.width = pc2.width;
    pcl_pc2.fields.clear();

    for (int i = 0; i < static_cast<int>(pc2.fields.size()); i++) {
      pcl::PCLPointField pcl_pf;
      pcl_pf.name = pc2.fields[i].name;
      pcl_pf.offset = pc2.fields[i].offset;
      pcl_pf.datatype = pc2.fields[i].datatype;
      pcl_pf.count = pc2.fields[i].count;
      pcl_pc2.fields.push_back(pcl_pf);
    }
    pcl_pc2.is_bigendian = pc2.is_bigendian;
    pcl_pc2.point_step = pc2.point_step;
    pcl_pc2.row_step = pc2.row_step;
    pcl_pc2.is_dense = pc2.is_dense;

    pcl_pc2.data = pc2.data;
  } catch (...) {
    std::cout << "ERROR" << std::endl;
  }
}

bool PCLFunctions::plane_segmentation(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_removed,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_table,
  const int & segmentation_max_iterations,
  const float & segmentation_distance_threshold)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(segmentation_max_iterations);
  seg.setDistanceThreshold(segmentation_distance_threshold);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);    // used to be table coeff

  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return false;
  }
  // Remove the planar inliers, extract the rest
  pcl::ExtractIndices<pcl::PointXYZRGB> indExtractor2;
  indExtractor2.setInputCloud(cloud);
  indExtractor2.setIndices(inliers);
  indExtractor2.setNegative(false);

  // Remove the planar inliers, extract the rest
  indExtractor2.filter(*cloud_table);


  // Remove the planar inliers, extract the rest
  pcl::ExtractIndices<pcl::PointXYZRGB> indExtractor;
  indExtractor.setInputCloud(cloud);
  indExtractor.setIndices(inliers);
  indExtractor.setNegative(false);

  // Remove the planar inliers, extract the rest
  indExtractor.setNegative(true);
  indExtractor.filter(*cloud_plane_removed);
  return true;
}

void PCLFunctions::remove_statistical_outlier(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  float threshold)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(10);
  sor.setStddevMulThresh(threshold);
  sor.filter(*cloud);
}

void PCLFunctions::get_closest_points_by_radius(
  const pcl::PointNormal & point,
  const float & radius,
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & inputCloud,
  const pcl::PointCloud<pcl::PointNormal>::Ptr & inputNormalCloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & outputCloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr & outputNormalCloud)
{
  pcl::search::KdTree<pcl::PointNormal>::Ptr treeSearch(
    new pcl::search::KdTree<pcl::PointNormal>());
  pcl::PointIndices::Ptr pointsIndex(new pcl::PointIndices);
  std::vector<float> pointsSquaredDistance;
  treeSearch->setInputCloud(inputNormalCloud);

  if (treeSearch->radiusSearch(point, radius, pointsIndex->indices, pointsSquaredDistance)) {
    extract_inliers_cloud<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
      pcl::ExtractIndices<pcl::PointXYZRGB>>(
      inputCloud,
      pointsIndex, outputCloud);
    extract_inliers_cloud<pcl::PointCloud<pcl::PointNormal>::Ptr,
      pcl::ExtractIndices<pcl::PointNormal>>(
      inputNormalCloud,
      pointsIndex, outputNormalCloud);
  }
}

void PCLFunctions::compute_cloud_normal(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal,
  const float & cloud_normal_radius)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
    new pcl::search::KdTree<pcl::PointXYZRGB>());
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> normal_estimation;
  normal_estimation.setNumberOfThreads(4);
  normal_estimation.setInputCloud(cloud);
  normal_estimation.setSearchMethod(tree);
  normal_estimation.setRadiusSearch(cloud_normal_radius);
  normal_estimation.compute(*cloud_normal);

  for (size_t i = 0; i < cloud_normal->points.size(); ++i) {
    cloud_normal->points[i].x = cloud->points[i].x;
    cloud_normal->points[i].y = cloud->points[i].y;
    cloud_normal->points[i].z = cloud->points[i].z;
  }
}

Eigen::Vector3f PCLFunctions::convert_pcl_normal_to_eigen(
  const pcl::PointNormal & pcl_point)
{
  return Eigen::Vector3f(pcl_point.normal_x, pcl_point.normal_y, pcl_point.normal_z);
}

std::vector<pcl::PointIndices> PCLFunctions::extract_pointcloud_clusters(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  float cluster_tolerance,
  int min_cluster_size)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecExtractor;

  tree->setInputCloud(cloud);
  ecExtractor.setClusterTolerance(cluster_tolerance);
  ecExtractor.setMinClusterSize(min_cluster_size);
  ecExtractor.setSearchMethod(tree);
  ecExtractor.setInputCloud(cloud);
  ecExtractor.extract(clusterIndices);
  return clusterIndices;
}

// float PCLFunctions::point_to_plane(Eigen::Vector4f & plane, pcl::PointXYZRGB const & point)
// {
//   return std::abs(plane(0) * point.x + plane(1) * point.y + plane(2) * point.z + plane(3)) /
//          std::sqrt(std::pow(plane(0), 2) + std::pow(plane(1), 2) + std::pow(plane(2), 2));
// }

// float PCLFunctions::point_to_plane(Eigen::Vector4f & plane, pcl::PointNormal const & point)
// {
//   return std::abs(plane(0) * point.x + plane(1) * point.y + plane(2) * point.z + plane(3)) /
//          std::sqrt(std::pow(plane(0), 2) + std::pow(plane(1), 2) + std::pow(plane(2), 2));
// }

// float PCLFunctions::point_to_plane(Eigen::Vector3f & plane, pcl::PointNormal const & point)
// {
//   return std::abs(plane(0) * point.x + plane(1) * point.y + plane(2) * point.z + plane(3)) /
//          std::sqrt(std::pow(plane(0), 2) + std::pow(plane(1), 2) + std::pow(plane(2), 2));
// }
