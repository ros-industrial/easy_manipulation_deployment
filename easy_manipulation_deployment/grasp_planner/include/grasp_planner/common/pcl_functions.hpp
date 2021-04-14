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

#ifndef PCL_FUNCTIONS_HPP_
#define PCL_FUNCTIONS_HPP_

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
    pcl::visualization::PCLVisualizer::Ptr viewer)
  {
    pcl::CentroidPoint < pcl::PointXYZRGB > centroid;
    for (int i = 0; i < static_cast < int > (target_cloud->size()); i++) {
      centroid.add(target_cloud->points[i]);
    }
    pcl::PointXYZ c1;
    centroid.get(c1);
    viewer->setCameraPosition(c1.x, c1.y, c1.z, 0, 0, 1);
  }

  void viewCloud(
    pcl::PointCloud < pcl::PointNormal > ::Ptr target_cloud,
    pcl::visualization::PCLVisualizer::Ptr viewer)
  {
    // viewer->removeAllPointClouds();
    viewer->addPointCloud < pcl::PointNormal > (target_cloud, "Main cloud");
    viewer->spin();
    viewer->close();
  }

  void viewerAddNormalCloud(
    pcl::PointCloud < pcl::PointNormal > ::Ptr target_ncloud,
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr target_cloud,
    std::string name, pcl::visualization::PCLVisualizer::Ptr viewer)
  {
    std::vector < int > nanNormalIndices;
    pcl::removeNaNNormalsFromPointCloud(*target_ncloud, *target_ncloud, nanNormalIndices);
    viewer->addPointCloudNormals < pcl::PointXYZRGB,
    pcl::PointNormal > (target_cloud, target_ncloud, 10, 0.05, name);
    viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,
      1.0, name);
  }

  void viewerAddRGBCloud(
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr target_cloud, std::string name,
    pcl::visualization::PCLVisualizer::Ptr viewer)
  {
    pcl::visualization::PointCloudColorHandlerRGBField < pcl::PointXYZRGB > rgb(target_cloud);
    // viewer->removeAllPointClouds();
    viewer->addPointCloud < pcl::PointXYZRGB > (target_cloud, rgb, name);
  }


  bool passthroughFilter(
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud,
    float ptFilter_Ulimit_x,
    float ptFilter_Llimit_x,
    float ptFilter_Ulimit_y,
    float ptFilter_Llimit_y,
    float ptFilter_Ulimit_z,
    float ptFilter_Llimit_z)
  {
    // Remove NaN values
    std::vector < int > nanIndices;

    pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndices);

    // Remove background points
    pcl::PassThrough < pcl::PointXYZRGB > ptFilter;
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

  bool passthroughFilter(
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud,
    float ptFilter_Ulimit_x,
    float ptFilter_Llimit_x,
    float ptFilter_Ulimit_y,
    float ptFilter_Llimit_y,
    float ptFilter_Ulimit_z,
    float ptFilter_Llimit_z,
    pcl::visualization::PCLVisualizer::Ptr viewer)
  {
    std::cout << ptFilter_Ulimit_x << std::endl;
    std::cout << ptFilter_Llimit_x << std::endl;
    std::cout << ptFilter_Ulimit_y << std::endl;
    std::cout << ptFilter_Llimit_y << std::endl;
    std::cout << ptFilter_Ulimit_z << std::endl;
    std::cout << ptFilter_Llimit_z << std::endl;
    // Remove NaN values
    std::vector < int > nanIndices;

    // pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndices);

    // Remove background points
    pcl::PassThrough < pcl::PointXYZRGB > ptFilter;
    ptFilter.setInputCloud(cloud);
    ptFilter.setFilterFieldName("z");
    ptFilter.setFilterLimits(ptFilter_Llimit_z, ptFilter_Ulimit_z);
    ptFilter.filter(*cloud);

    PCLFunctions::viewerAddRGBCloud(cloud, "original_cloud", viewer);
    viewer->spin();
    viewer->close();
    viewer->removeAllPointClouds();


    ptFilter.setInputCloud(cloud);
    ptFilter.setFilterFieldName("y");
    ptFilter.setFilterLimits(ptFilter_Llimit_y, ptFilter_Ulimit_y);
    ptFilter.filter(*cloud);

    PCLFunctions::viewerAddRGBCloud(cloud, "original_cloud", viewer);
    viewer->spin();
    viewer->close();
    viewer->removeAllPointClouds();


    ptFilter.setInputCloud(cloud);
    ptFilter.setFilterFieldName("x");
    ptFilter.setFilterLimits(ptFilter_Llimit_x, ptFilter_Ulimit_x);
    ptFilter.filter(*cloud);

    PCLFunctions::viewerAddRGBCloud(cloud, "original_cloud", viewer);
    viewer->spin();
    viewer->close();
    viewer->removeAllPointClouds();

    return true;
  }


  void SensorMsgtoPCLPointCloud2(
    const sensor_msgs::msg::PointCloud2 & pc2,
    pcl::PCLPointCloud2 & pcl_pc2)
  {
    try {  /* */
      pcl_pc2.header.stamp = pc2.header.stamp.nanosec / 1000ull;
      pcl_pc2.header.seq = 0;
      pcl_pc2.header.frame_id = pc2.header.frame_id;
      pcl_pc2.height = pc2.height;
      pcl_pc2.width = pc2.width;
      pcl_pc2.fields.clear();

      for (int i = 0; i < static_cast < int > (pc2.fields.size()); i++) {
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
    }
    catch(...) {
      std::cout << "ERROR" << std::endl;
    }
  }

  bool planeSegmentation(
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud,
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud_plane_removed,
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud_table, int segmentation_max_iterations,
    float segmentation_distance_threshold)
  {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation < pcl::PointXYZRGB > seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(segmentation_max_iterations);
    seg.setDistanceThreshold(segmentation_distance_threshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);  // used to be table coeff

    if (inliers->indices.size() == 0) {
      PCL_ERROR("Could not estimate a planar model for the given dataset.");
      return false;
    }
    std::cout << "indExt" << std::endl;
    // Remove the planar inliers, extract the rest
    pcl::ExtractIndices < pcl::PointXYZRGB > indExtractor2;
    indExtractor2.setInputCloud(cloud);
    indExtractor2.setIndices(inliers);
    indExtractor2.setNegative(false);

    // Remove the planar inliers, extract the rest
    indExtractor2.filter(*cloud_table);


    // Remove the planar inliers, extract the rest
    pcl::ExtractIndices < pcl::PointXYZRGB > indExtractor;
    indExtractor.setInputCloud(cloud);
    indExtractor.setIndices(inliers);
    indExtractor.setNegative(false);

    // Remove the planar inliers, extract the rest
    indExtractor.setNegative(true);
    indExtractor.filter(*cloud_plane_removed);
    return true;
  }

  void removeAllZeros(pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud)
  {
    int initial_size = cloud->points.size();
    pcl::PassThrough < pcl::PointXYZRGB > ptFilter;
    ptFilter.setInputCloud(cloud);
    ptFilter.setFilterFieldName("z");
    // ptFilter.setFilterLimitsNegative (true);
    ptFilter.setFilterLimits(0, 0.0001);
    ptFilter.filter(*cloud);
    int after_size = cloud->points.size();
    std::cout << "Removed " << initial_size - after_size << " Number of zero points" << std::endl;
  }

  void removeAllZeros(pcl::PointCloud < pcl::PointXYZ > ::Ptr cloud)
  {
    int initial_size = cloud->points.size();
    pcl::PassThrough < pcl::PointXYZ > ptFilter;
    ptFilter.setInputCloud(cloud);
    ptFilter.setFilterFieldName("z");
    // ptFilter.setFilterLimitsNegative (true);
    ptFilter.setFilterLimits(0, 0.0001);
    ptFilter.filter(*cloud);
    int after_size = cloud->points.size();
    std::cout << "Removed " << initial_size - after_size << " Number of zero points" << std::endl;
  }

  float pointToPlane(Eigen::Vector4f & plane, pcl::PointXYZRGB const & point)
  {
    return std::abs(plane(0) * point.x + plane(1) * point.y + plane(2) * point.z + plane(3)) /
           std::sqrt(std::pow(plane(0), 2) + std::pow(plane(1), 2) + std::pow(plane(2), 2));
  }

  float pointToPlane(Eigen::Vector4f & plane, pcl::PointNormal const & point)
  {
    return std::abs(plane(0) * point.x + plane(1) * point.y + plane(2) * point.z + plane(3)) /
           std::sqrt(std::pow(plane(0), 2) + std::pow(plane(1), 2) + std::pow(plane(2), 2));
  }

  float pointToPlane(Eigen::Vector3f & plane, pcl::PointNormal const & point)
  {
    return std::abs(plane(0) * point.x + plane(1) * point.y + plane(2) * point.z + plane(3)) /
           std::sqrt(std::pow(plane(0), 2) + std::pow(plane(1), 2) + std::pow(plane(2), 2));
  }

  void removeStatisticalOutlier(
    const pcl::PointCloud < pcl::PointXYZRGB > ::Ptr & cloud,
    float threshold)
  {
    pcl::StatisticalOutlierRemoval < pcl::PointXYZRGB > sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(10);
    sor.setStddevMulThresh(threshold);
    // sor.setStddevMulThresh (0.7);
    sor.filter(*cloud);
  }

  void getClosestPointsByRadius(
    const pcl::PointNormal & point,
    const float & radius, pcl::PointCloud < pcl::PointXYZRGB > ::Ptr & inputCloud,
    pcl::PointCloud < pcl::PointNormal > ::Ptr & inputNormalCloud,
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr & outputCloud,
    pcl::PointCloud < pcl::PointNormal > ::Ptr & outputNormalCloud)
  {
    pcl::search::KdTree < pcl::PointNormal > ::Ptr treeSearch(
      new pcl::search::KdTree < pcl::PointNormal > ());
    pcl::PointIndices::Ptr pointsIndex(new pcl::PointIndices);
    std::vector < float > pointsSquaredDistance;
    treeSearch->setInputCloud(inputNormalCloud);

    if (treeSearch->radiusSearch(point, radius, pointsIndex->indices, pointsSquaredDistance)) {
      extractInliersCloud < pcl::PointCloud < pcl::PointXYZRGB > ::Ptr,
      pcl::ExtractIndices < pcl::PointXYZRGB >> (inputCloud,
      pointsIndex, outputCloud);
      extractInliersCloud < pcl::PointCloud < pcl::PointNormal > ::Ptr,
      pcl::ExtractIndices < pcl::PointNormal >> (inputNormalCloud,
      pointsIndex, outputNormalCloud);
    }
  }

  void computeCloudNormal(
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud,
    pcl::PointCloud < pcl::PointNormal > ::Ptr cloud_normal, const float & cloud_normal_radius)
  {
    std::chrono::steady_clock::time_point getSlicedCloud1 = std::chrono::steady_clock::now();
    pcl::search::KdTree < pcl::PointXYZRGB > ::Ptr tree(
      new pcl::search::KdTree < pcl::PointXYZRGB > ());
    pcl::NormalEstimationOMP < pcl::PointXYZRGB, pcl::PointNormal > normal_estimation;
    normal_estimation.setNumberOfThreads(4);
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(cloud_normal_radius);
    normal_estimation.compute(*cloud_normal);
    // NormalEstimation only fills with normals the output
    std::chrono::steady_clock::time_point getSlicedCloud2 = std::chrono::steady_clock::now();
    std::cout << "Grasp planning time for computeCloudNormal " <<
      std::chrono::duration_cast < std::chrono::milliseconds >
    (getSlicedCloud2 - getSlicedCloud1).count() << "[ms]" << std::endl;

    for (size_t i = 0; i < cloud_normal->points.size(); ++i) {
      cloud_normal->points[i].x = cloud->points[i].x;
      cloud_normal->points[i].y = cloud->points[i].y;
      cloud_normal->points[i].z = cloud->points[i].z;
    }
  }

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
    const float & radius, const int & resolution)
  {
    float px, py, pz;
    for (float phi = 0; phi < M_PI; phi += M_PI / resolution) {
      pz = radius * cos(phi);
      for (float theta = 0; theta < 2 * M_PI; theta += 2 * M_PI / resolution) {
        px = radius * sin(phi) * cos(theta) + centerpoint(0);
        py = radius * sin(phi) * sin(theta) + centerpoint(1);
        pcl::PointXYZ point {px, py, pz + centerpoint(2)};
        output_sphere_cloud->points.push_back(point);
      }
    }
    output_sphere_cloud->is_dense = true;
    output_sphere_cloud->height = 1;
    output_sphere_cloud->width = output_sphere_cloud->points.size();
  }

  inline float normalize(const float & target, const float & min, const float & max)
  {
    return (target + 1e-8 - min) / (max - min);
  }

  inline float normalizeInt(const int & target, const int & min, const int & max)
  {
    return float(target - min) / float(max - min);
  }

  void createRectangularCloud(pcl::visualization::PCLVisualizer::Ptr viewer)
  {
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr rectangle_cloud(
      new pcl::PointCloud < pcl::PointXYZRGB >);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    // pcl::PointCloud<pcl::PointNormal>::Ptr
    float length = 0.05;
    float breadth = 0.01;
    float height = 0.02;

    for (float length_ = 0.0; length_ < length; length_ += 0.001) {
      for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.001) {
        for (float height_ = 0.0; height_ < height; height_ += 0.001) {
          pcl::PointXYZRGB temp_point(length_, breadth_, height_);
          rectangle_cloud->points.push_back(temp_point);
        }
      }
    }
    pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZRGB > rgb(
      rectangle_cloud, 0,
      255, 0);  // This is blue
    viewer->addPointCloud < pcl::PointXYZRGB > (rectangle_cloud, rgb, "rectangle_cloud");
    viewer->spin();
    viewer->close();
    viewer->removeAllPointClouds();
  }
}  // namespace PCLFunctions

#endif  // PCL_FUNCTIONS_HPP_
