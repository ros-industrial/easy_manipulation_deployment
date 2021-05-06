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

#include "grasp_planner/common/pcl_functions.hpp"

bool PCLFunctions::passthroughFilter(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  float ptFilter_Ulimit_x,
  float ptFilter_Llimit_x,
  float ptFilter_Ulimit_y,
  float ptFilter_Llimit_y,
  float ptFilter_Ulimit_z,
  float ptFilter_Llimit_z)
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

void PCLFunctions::SensorMsgtoPCLPointCloud2(
  const sensor_msgs::msg::PointCloud2 & pc2,
  pcl::PCLPointCloud2 & pcl_pc2)
{
  try {    /* */
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

bool PCLFunctions::planeSegmentation(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_removed,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_table, int segmentation_max_iterations,
  float segmentation_distance_threshold)
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
  std::cout << "indExt" << std::endl;
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

void PCLFunctions::removeAllZeros(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  int initial_size = cloud->points.size();
  pcl::PassThrough<pcl::PointXYZRGB> ptFilter;
  ptFilter.setInputCloud(cloud);
  ptFilter.setFilterFieldName("z");
  // ptFilter.setFilterLimitsNegative (true);
  ptFilter.setFilterLimits(0, 0.0001);
  ptFilter.filter(*cloud);
  int after_size = cloud->points.size();
  std::cout << "Removed " << initial_size - after_size << " Number of zero points" << std::endl;
}

void PCLFunctions::removeAllZeros(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  int initial_size = cloud->points.size();
  pcl::PassThrough<pcl::PointXYZ> ptFilter;
  ptFilter.setInputCloud(cloud);
  ptFilter.setFilterFieldName("z");
  // ptFilter.setFilterLimitsNegative (true);
  ptFilter.setFilterLimits(0, 0.0001);
  ptFilter.filter(*cloud);
  int after_size = cloud->points.size();
  std::cout << "Removed " << initial_size - after_size << " Number of zero points" << std::endl;
}

float PCLFunctions::pointToPlane(Eigen::Vector4f & plane, pcl::PointXYZRGB const & point)
{
  return std::abs(plane(0) * point.x + plane(1) * point.y + plane(2) * point.z + plane(3)) /
         std::sqrt(std::pow(plane(0), 2) + std::pow(plane(1), 2) + std::pow(plane(2), 2));
}

float PCLFunctions::pointToPlane(Eigen::Vector4f & plane, pcl::PointNormal const & point)
{
  return std::abs(plane(0) * point.x + plane(1) * point.y + plane(2) * point.z + plane(3)) /
         std::sqrt(std::pow(plane(0), 2) + std::pow(plane(1), 2) + std::pow(plane(2), 2));
}

float PCLFunctions::pointToPlane(Eigen::Vector3f & plane, pcl::PointNormal const & point)
{
  return std::abs(plane(0) * point.x + plane(1) * point.y + plane(2) * point.z + plane(3)) /
         std::sqrt(std::pow(plane(0), 2) + std::pow(plane(1), 2) + std::pow(plane(2), 2));
}

void PCLFunctions::removeStatisticalOutlier(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  float threshold)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(10);
  sor.setStddevMulThresh(threshold);
  // sor.setStddevMulThresh (0.7);
  sor.filter(*cloud);
}

void PCLFunctions::getClosestPointsByRadius(
  const pcl::PointNormal & point,
  const float & radius, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & inputCloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr & inputNormalCloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & outputCloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr & outputNormalCloud)
{
  pcl::search::KdTree<pcl::PointNormal>::Ptr treeSearch(
    new pcl::search::KdTree<pcl::PointNormal>());
  pcl::PointIndices::Ptr pointsIndex(new pcl::PointIndices);
  std::vector<float> pointsSquaredDistance;
  treeSearch->setInputCloud(inputNormalCloud);

  if (treeSearch->radiusSearch(point, radius, pointsIndex->indices, pointsSquaredDistance)) {
    extractInliersCloud<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
      pcl::ExtractIndices<pcl::PointXYZRGB>>(
      inputCloud,
      pointsIndex, outputCloud);
    extractInliersCloud<pcl::PointCloud<pcl::PointNormal>::Ptr,
      pcl::ExtractIndices<pcl::PointNormal>>(
      inputNormalCloud,
      pointsIndex, outputNormalCloud);
  }
}

void PCLFunctions::computeCloudNormal(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal, const float & cloud_normal_radius)
{
  std::chrono::steady_clock::time_point getSlicedCloud1 = std::chrono::steady_clock::now();
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
    new pcl::search::KdTree<pcl::PointXYZRGB>());
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> normal_estimation;
  normal_estimation.setNumberOfThreads(4);
  normal_estimation.setInputCloud(cloud);
  normal_estimation.setSearchMethod(tree);
  normal_estimation.setRadiusSearch(cloud_normal_radius);
  normal_estimation.compute(*cloud_normal);
  // NormalEstimation only fills with normals the output
  std::chrono::steady_clock::time_point getSlicedCloud2 = std::chrono::steady_clock::now();
  std::cout << "Grasp planning time for computeCloudNormal " <<
    std::chrono::duration_cast<std::chrono::milliseconds>(
    getSlicedCloud2 -
    getSlicedCloud1).count() << "[ms]" << std::endl;

  for (size_t i = 0; i < cloud_normal->points.size(); ++i) {
    cloud_normal->points[i].x = cloud->points[i].x;
    cloud_normal->points[i].y = cloud->points[i].y;
    cloud_normal->points[i].z = cloud->points[i].z;
  }
}

void PCLFunctions::createSphereCloud(
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_sphere_cloud,
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