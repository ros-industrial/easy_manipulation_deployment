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

//Main PCL files
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


//For Plane Segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//For Object Segmentation
#include <pcl/segmentation/extract_clusters.h>

//For Cloud Filtering
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

//For Visualization
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>


//ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud viewer"));

class PCLTest : public rclcpp::Node
{
  std::string incloudfile;
  std::string outcloudfile;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

  //Filter Variables
  float ptFilter_Ulimit_x;
  float ptFilter_Llimit_x;
  float ptFilter_Ulimit_y;
  float ptFilter_Llimit_y;
  float ptFilter_Ulimit_z;
  float ptFilter_Llimit_z;

  //Plane segmentation Variables
  int segmentation_max_iterations;
  float segmentation_distance_threshold;

  //Object clustering Variables
  float cluster_tolerance;
  int min_cluster_size;

  sensor_msgs::msg::PointCloud2 pointcloud2;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> object_cloud_vec;

public:
  PCLTest()
  : Node("pcl_node"),
    cloud(new pcl::PointCloud<pcl::PointXYZRGB>())
  {
    std::cout << "Start" << std::endl;
    incloudfile = "/home/rosi5/pcl_ws/src/pcl_test/data/objects-example.pcd";
    outcloudfile = "/home/rosi5/pcl_ws/src/pcl_test/data/objects-example-output.pcd";
    ptFilter_Ulimit_x = 0.50;
    ptFilter_Llimit_x = -0.50;
    ptFilter_Ulimit_y = 0.40;
    ptFilter_Llimit_y = -0.55;
    ptFilter_Ulimit_z = 1.5;
    ptFilter_Llimit_z = 0.0;

    segmentation_max_iterations = 50;
    segmentation_distance_threshold = 0.02;

    cluster_tolerance = 0.01;
    min_cluster_size = 750;


    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(incloudfile.c_str(), *cloud) == -1) { //* load the file
      PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    }
    std::cout << "Loaded " <<
      cloud->width * cloud->height <<
      " data points from " <<
      incloudfile <<
      std::endl;
    view_cloud();
    passthrough_filter();
    view_cloud();
    plane_segmentation();
    view_cloud();
    extract_objects();
    for (int i = 0; i < static_cast<int>(object_cloud_vec.size()); i++) {
      view_cloud(object_cloud_vec[i]);
    }
  }

  void view_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud)
  {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(target_cloud);
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPointCloud<pcl::PointXYZRGB>(target_cloud, rgb, "Main cloud");
    viewer->spin();
    viewer->close();
  }

  void view_cloud()
  {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Main cloud");
    viewer->spin();
    viewer->close();
  }
  bool passthrough_filter()
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

  bool plane_segmentation()
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
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
      PCL_ERROR("Could not estimate a planar model for the given dataset.");
      return false;
    }

    // Remove the planar inliers, extract the rest
    pcl::ExtractIndices<pcl::PointXYZRGB> indExtractor;
    indExtractor.setInputCloud(cloud);
    indExtractor.setIndices(inliers);
    indExtractor.setNegative(false);

    // Get the points associated with the planar surface
    //indExtractor.filter(*cloudPlane);

    // Remove the planar inliers, extract the rest
    indExtractor.setNegative(true);
    indExtractor.filter(*cloud);
    return true;
  }

  bool extract_objects()
  {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecExtractor;
    ecExtractor.setClusterTolerance(cluster_tolerance);
    ecExtractor.setMinClusterSize(min_cluster_size);
    ecExtractor.setSearchMethod(tree);
    ecExtractor.setInputCloud(cloud);
    ecExtractor.extract(clusterIndices);

    if (clusterIndices.empty()) {
      return false;
    } else {
      std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();
      int objectNumber = 0;
      for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (std::vector<int>::const_iterator pit = it->indices.begin();
          pit != it->indices.end(); ++pit)
        {
          objectCloud->points.push_back(cloud->points[*pit]);
        }
        objectCloud->width = objectCloud->points.size();
        objectCloud->height = 1;
        objectCloud->is_dense = true;
        object_cloud_vec.push_back(objectCloud);
      }
    }
    return true;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLTest>());
  rclcpp::shutdown();
  return 0;
}
