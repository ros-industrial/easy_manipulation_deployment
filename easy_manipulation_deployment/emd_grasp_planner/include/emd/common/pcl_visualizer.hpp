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

#ifndef EMD__GRASP_PLANNER__COMMON__PCL_VISUALIZER_HPP_
#define EMD__GRASP_PLANNER__COMMON__PCL_VISUALIZER_HPP_

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

// EMD Libraries
// #include "grasp_object.h"


namespace PCLVisualizer
{

/**
 * Center a point cloud in the middle of the PCL Viewer
 *
 * \param[in] target_cloud Target CLoud
 * \param[in] viewer PCL Visualizer instance
 */
void center_camera(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud,
  pcl::visualization::PCLVisualizer::Ptr viewer);

/**
 * Function to add a pointcloud and spin the PCL Visualizer
 *
 * \param[in] target_cloud Target CLoud
 * \param[in] viewer PCL Visualizer instance
 */
void view_cloud(
  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud,
  pcl::visualization::PCLVisualizer::Ptr viewer);

/**
 * Function to add a Pointcloud Normal cloud in a PCL Visualizer
 *
 * \param[in] target_ncloud Target PointNormal CLoud
 * \param[in] target_cloud Target PointXYZRGB Cloud
 * \param[in] viewer PCL Visualizer instance
 */
void viewer_add_normal_cloud(
  pcl::PointCloud<pcl::PointNormal>::Ptr target_ncloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud,
  std::string name, pcl::visualization::PCLVisualizer::Ptr viewer);


/**
 * Function to add a Pointcloud RGB cloud in a PCL Visualizer
 *
 * \param[in] target_cloud Target PointXYZRGB Cloud
 * \param[in] name Name of Cloud
 * \param[in] viewer PCL Visualizer instance
 */
void viewer_add_rgb_cloud(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud, std::string name,
  pcl::visualization::PCLVisualizer::Ptr viewer);
}  // namespace PCLVisualizer

#endif  // EMD__GRASP_PLANNER__COMMON__PCL_VISUALIZER_HPP_
