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

#include "emd/common/pcl_visualizer.hpp"


void PCLVisualizer::center_camera(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud,
  pcl::visualization::PCLVisualizer::Ptr viewer)
{
  pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
  for (int i = 0; i < static_cast<int>(target_cloud->size()); i++) {
    centroid.add(target_cloud->points[i]);
  }
  pcl::PointXYZ c1;
  centroid.get(c1);
  viewer->setCameraPosition(c1.x, c1.y, c1.z, 0, 0, 1);
}

void PCLVisualizer::view_cloud(
  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud,
  pcl::visualization::PCLVisualizer::Ptr viewer)
{
  // viewer->removeAllPointClouds();
  viewer->addPointCloud<pcl::PointNormal>(target_cloud, "Main cloud");
  viewer->spin();
  viewer->close();
}

void PCLVisualizer::viewer_add_normal_cloud(
  pcl::PointCloud<pcl::PointNormal>::Ptr target_ncloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud,
  std::string name, pcl::visualization::PCLVisualizer::Ptr viewer)
{
  std::vector<int> nanNormalIndices;
  pcl::removeNaNNormalsFromPointCloud(*target_ncloud, *target_ncloud, nanNormalIndices);
  viewer->addPointCloudNormals<pcl::PointXYZRGB,
    pcl::PointNormal>(target_cloud, target_ncloud, 10, 0.05, name);
  viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,
    1.0, name);
}

void PCLVisualizer::viewer_add_rgb_cloud(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud, std::string name,
  pcl::visualization::PCLVisualizer::Ptr viewer)
{
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(target_cloud);
  // viewer->removeAllPointClouds();
  viewer->addPointCloud<pcl::PointXYZRGB>(target_cloud, rgb, name);
}
