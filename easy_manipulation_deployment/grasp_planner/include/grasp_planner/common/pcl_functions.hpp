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

#ifndef GRASP_PLANNER__COMMON__PCL_FUNCTIONS_HPP_
#define GRASP_PLANNER__COMMON__PCL_FUNCTIONS_HPP_

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

// For Collision Detection
#include <fcl/narrowphase/collision_object.h>  //NOLINT
#include "fcl/geometry/octree/octree.h"
#include <octomap/octomap.h>  //NOLINT

// EMD Libraries
// #include "grasp_object.h"


namespace PCLFunctions
{
  template < typename T, typename U >
  void extractInliersCloud(
    const T & inputCloud,
    const pcl::PointIndices::Ptr & inputCloudInliers, T outputCloud)
  {
    U extractor;
    extractor.setInputCloud(inputCloud);

    extractor.setIndices(inputCloudInliers);

    extractor.filter(*outputCloud);
  }

  void centerCamera(
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr target_cloud,
    pcl::visualization::PCLVisualizer::Ptr viewer);


  void viewCloud(
    pcl::PointCloud < pcl::PointNormal > ::Ptr target_cloud,
    pcl::visualization::PCLVisualizer::Ptr viewer);

  void viewerAddNormalCloud(
    pcl::PointCloud < pcl::PointNormal > ::Ptr target_ncloud,
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr target_cloud,
    std::string name, pcl::visualization::PCLVisualizer::Ptr viewer);

  void viewerAddRGBCloud(
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr target_cloud, std::string name,
    pcl::visualization::PCLVisualizer::Ptr viewer);


  bool passthroughFilter(
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud,
    float ptFilter_Ulimit_x,
    float ptFilter_Llimit_x,
    float ptFilter_Ulimit_y,
    float ptFilter_Llimit_y,
    float ptFilter_Ulimit_z,
    float ptFilter_Llimit_z);

  bool passthroughFilter(
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud,
    float ptFilter_Ulimit_x,
    float ptFilter_Llimit_x,
    float ptFilter_Ulimit_y,
    float ptFilter_Llimit_y,
    float ptFilter_Ulimit_z,
    float ptFilter_Llimit_z,
    pcl::visualization::PCLVisualizer::Ptr viewer);

  void SensorMsgtoPCLPointCloud2(
    const sensor_msgs::msg::PointCloud2 & pc2,
    pcl::PCLPointCloud2 & pcl_pc2);

  bool planeSegmentation(
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud,
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud_plane_removed,
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud_table, int segmentation_max_iterations,
    float segmentation_distance_threshold);

  void removeAllZeros(pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud);

  void removeAllZeros(pcl::PointCloud < pcl::PointXYZ > ::Ptr cloud);

  float pointToPlane(Eigen::Vector4f & plane, pcl::PointXYZRGB const & point);

  float pointToPlane(Eigen::Vector4f & plane, pcl::PointNormal const & point);

  float pointToPlane(Eigen::Vector3f & plane, pcl::PointNormal const & point);

  void removeStatisticalOutlier(
    const pcl::PointCloud < pcl::PointXYZRGB > ::Ptr & cloud,
    float threshold);

  void getClosestPointsByRadius(
    const pcl::PointNormal & point,
    const float & radius, pcl::PointCloud < pcl::PointXYZRGB > ::Ptr & inputCloud,
    pcl::PointCloud < pcl::PointNormal > ::Ptr & inputNormalCloud,
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr & outputCloud,
    pcl::PointCloud < pcl::PointNormal > ::Ptr & outputNormalCloud);

  void computeCloudNormal(
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud,
    pcl::PointCloud < pcl::PointNormal > ::Ptr cloud_normal, const float & cloud_normal_radius);

  template < typename T, typename U >
  void voxelizeCloud(const T & inputCloud, const float & leafSize, T outputCloud)
  {
    U voxelFilter;
    voxelFilter.setInputCloud(inputCloud);
    voxelFilter.setLeafSize(leafSize, leafSize, leafSize);
    voxelFilter.filter(*outputCloud);
  }


  void createSphereCloud(
    pcl::PointCloud < pcl::PointXYZ > ::Ptr output_sphere_cloud,
    Eigen::Vector3f & centerpoint,
    const float & radius, const int & resolution);

  float normalize(const float & target, const float & min, const float & max);

  float normalizeInt(const int & target, const int & min, const int & max);

  void createRectangularCloud(pcl::visualization::PCLVisualizer::Ptr viewer);
}  // namespace PCLFunctions

#endif  // GRASP_PLANNER__COMMON__PCL_FUNCTIONS_HPP_
