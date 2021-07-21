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

#ifndef EMD__GRASP_PLANNER__COMMON__PCL_FUNCTIONS_HPP_
#define EMD__GRASP_PLANNER__COMMON__PCL_FUNCTIONS_HPP_

// Main PCL files
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

// For Statistical outlier removal
#include <pcl/filters/statistical_outlier_removal.h>

// For Voxelization
#include <pcl/filters/voxel_grid.h>

// For Plane Segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// For Object Segmentation
#include <pcl/segmentation/extract_clusters.h>

// For Cloud Filtering
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>

// For Visualization
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

// Normal Estimation
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

// ROS2 Libraries
// #include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

// Other Libraries
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <memory>
#include <future>
#include <string>
#include <vector>

// EMD Libraries
// #include "grasp_object.h"


namespace PCLFunctions
{

bool passthroughFilter(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  const float & ptFilter_Ulimit_x,
  const float & ptFilter_Llimit_x,
  const float & ptFilter_Ulimit_y,
  const float & ptFilter_Llimit_y,
  const float & ptFilter_Ulimit_z,
  const float & ptFilter_Llimit_z);

void SensorMsgtoPCLPointCloud2(
  const sensor_msgs::msg::PointCloud2 & pc2,
  pcl::PCLPointCloud2 & pcl_pc2);

bool planeSegmentation(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_removed,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_table,
  const int & segmentation_max_iterations,
  const float & segmentation_distance_threshold);

void removeStatisticalOutlier(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  float threshold);

void getClosestPointsByRadius(
  const pcl::PointNormal & point,
  const float & radius, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & inputCloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr & inputNormalCloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & outputCloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr & outputNormalCloud);

void computeCloudNormal(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal,
  const float & cloud_normal_radius);

Eigen::Vector3f convertPCLNormaltoEigen(
  const pcl::PointNormal & pcl_point);

template<typename T, typename U>
void voxelizeCloud(const T & inputCloud, const float & leafSize, T outputCloud)
{
  U voxelFilter;
  voxelFilter.setInputCloud(inputCloud);
  voxelFilter.setLeafSize(leafSize, leafSize, leafSize);
  voxelFilter.filter(*outputCloud);
}

template<typename V>
Eigen::Vector3f convertPCLtoEigen(const V & pcl_point)
{
  return Eigen::Vector3f(pcl_point.x, pcl_point.y, pcl_point.z);
}

template<typename W, typename X>
float pointToPlane(const W & plane, const X & point)
{
  return std::abs(plane(0) * point.x + plane(1) * point.y + plane(2) * point.z + plane(3)) /
         std::sqrt(std::pow(plane(0), 2) + std::pow(plane(1), 2) + std::pow(plane(2), 2));
}

template<typename T, typename U>
void extractInliersCloud(
  const T & inputCloud,
  const pcl::PointIndices::Ptr & inputCloudInliers, T outputCloud)
{
  U extractor;
  extractor.setInputCloud(inputCloud);

  extractor.setIndices(inputCloudInliers);

  extractor.filter(*outputCloud);
}

std::vector<pcl::PointIndices> extractPointCloudClusters(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  float cluster_tolerance,
  int min_cluster_size);


}  // namespace PCLFunctions


// float pointToPlane(Eigen::Vector4f & plane, pcl::PointXYZRGB const & point);

// float pointToPlane(Eigen::Vector4f & plane, pcl::PointNormal const & point);

// float pointToPlane(Eigen::Vector3f & plane, pcl::PointNormal const & point);

#endif  // EMD__GRASP_PLANNER__COMMON__PCL_FUNCTIONS_HPP_
