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

/**
 * PCL implementation of passthrough filter with given input cloud
 *
 * \param[in] cloud Target Cloud
 * \param[in] ptFilter_Ulimit_x Upper limit for passthrough filter in the x direction
 * \param[in] ptFilter_Llimit_x Lower limit for passthrough filter in the x direction
 * \param[in] ptFilter_Ulimit_y Upper limit for passthrough filter in the y direction
 * \param[in] ptFilter_Llimit_y Lower limit for passthrough filter in the y direction
 * \param[in] ptFilter_Ulimit_z Upper limit for passthrough filter in the z direction
 * \param[in] ptFilter_Llimit_z Lower limit for passthrough filter in the z direction
 */
bool passthrough_filter(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  const float & ptFilter_Ulimit_x,
  const float & ptFilter_Llimit_x,
  const float & ptFilter_Ulimit_y,
  const float & ptFilter_Llimit_y,
  const float & ptFilter_Ulimit_z,
  const float & ptFilter_Llimit_z);

/**
 * Conversion of sensor_msg's PointCloud 2 type to pcl's PointCloud2 type
 *
 * \param[in] pc2 Target input messsage
 * \param[in] pcl_pc2 Target output message
 */
void sensor_msg_to_pcl_pointcloud2(
  const sensor_msgs::msg::PointCloud2 & pc2,
  pcl::PCLPointCloud2 & pcl_pc2);

/**
 * PCL implementation of plane segmentation and removal
 *
 * \param[in] cloud Target Cloud input
 * \param[in] cloud_plane_removed Output cloud with removed plane surface
 * \param[in] cloud_table Pointcloud of plane surface removed
 * \param[in] segmentation_max_iterations Maximum segmentation iterations for segmentation
 * \param[in] segmentation_distance_threshold Segmentation distance threshold for segmentation
 */
bool plane_segmentation(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_removed,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_table,
  const int & segmentation_max_iterations,
  const float & segmentation_distance_threshold);

/**
 * PCL implementation of statistical outlier removal
 *
 * \param[in] cloud Target Cloud input
 * \param[in] threshold Threshold for
 */
void remove_statistical_outlier(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  float threshold);

/**
 * Gets the closest Pointcloud points within a fixed radius from a point
 *
 * \param[in] point Target Point
 * \param[in] radius Radius of search from the target point
 * \param[in] inputCloud Target Cloud Input
 * \param[in] inputNormalCloud Target Normal Input
 * \param[in] outputCloud Output Cloud of the closest points
 * \param[in] outputNormalCloud Output Normal Cloud of the closest points
 */
void get_closest_points_by_radius(
  const pcl::PointNormal & point,
  const float & radius,
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & inputCloud,
  const pcl::PointCloud<pcl::PointNormal>::Ptr & inputNormalCloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & outputCloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr & outputNormalCloud);

/**
 * Method to generate Cloud Normal from a point cloud
 *
 * \param[in] cloud Target Cloud input
 * \param[in] cloud_normal Output Cloud Normal
 * \param[in] cloud_normal_radius Radius value for cloud normal
 */
void compute_cloud_normal(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal,
  const float & cloud_normal_radius);


/**
 *  Convert PCL PointNormal point to an EigenVector3f vector
 *
 * \param[in] pcl_point Target PointNormal to be converted
 */
Eigen::Vector3f convert_pcl_normal_to_eigen(
  const pcl::PointNormal & pcl_point);

/**
 * Method to downsample a point cloud
 *
 * \param[in] inputCloud Target Cloud input
 * \param[in] leafSize Size of each leaf after voxelization
 * \param[in] outputCloud Downsampled Cloud Output
 */
template<typename T, typename U>
void voxelize_cloud(const T & inputCloud, const float & leafSize, T outputCloud)
{
  U voxelFilter;
  voxelFilter.setInputCloud(inputCloud);
  voxelFilter.setLeafSize(leafSize, leafSize, leafSize);
  voxelFilter.filter(*outputCloud);
}

/**
 *  Convert a PCL point (XYZ, XYZRGB) to an EigenVector3f vector
 *
 * \param[in] pcl_point Target PointNormal to be converted
 */
template<typename V>
Eigen::Vector3f convert_pcl_to_eigen(const V & pcl_point)
{
  return Eigen::Vector3f(pcl_point.x, pcl_point.y, pcl_point.z);
}

/**
 *  Returns the distance from a 3D point to a plane
 *
 * \param[in] plane Coefficient of a planm
 * \param[in] point 3D Coordinates of point
 */
template<typename W, typename X>
float point_to_plane(const W & plane, const X & point)
{
  return std::abs(plane(0) * point.x + plane(1) * point.y + plane(2) * point.z + plane(3)) /
         std::sqrt(std::pow(plane(0), 2) + std::pow(plane(1), 2) + std::pow(plane(2), 2));
}

/**
 *  PCL Implementation of extracting inliers within a cloud
 *
 * \param[in] inputCloud Input Cloud
 * \param[in] inputCloudInliers Indices parameters for extraction
 * \param[in] outputCloud output cloud after extraction
 */
template<typename T, typename U>
void extract_inliers_cloud(
  const T & inputCloud,
  const pcl::PointIndices::Ptr & inputCloudInliers, T outputCloud)
{
  U extractor;
  extractor.setInputCloud(inputCloud);

  extractor.setIndices(inputCloudInliers);

  extractor.filter(*outputCloud);
}

/**
 *  PCL Implementation of extracting point cloud clusters from a point cloud.
 *  Returns a vector of point indices representing the extracted point cloud
 *
 * \param[in] cloud Input Cloud
 * \param[in] cluster_tolerance Distance around a point to be considered as part of a cluster
 * \param[in] min_cluster_size Minimum size of extracted cluster to be considered
 */
std::vector<pcl::PointIndices> extract_pointcloud_clusters(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  float cluster_tolerance,
  int min_cluster_size);
}  // namespace PCLFunctions


// float point_to_plane(Eigen::Vector4f & plane, pcl::PointXYZRGB const & point);

// float point_to_plane(Eigen::Vector4f & plane, pcl::PointNormal const & point);

// float point_to_plane(Eigen::Vector3f & plane, pcl::PointNormal const & point);

#endif  // EMD__GRASP_PLANNER__COMMON__PCL_FUNCTIONS_HPP_
