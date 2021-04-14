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

//Other Libraries
#include <cmath>

//Custom library
#include "pointcloud_functions.hpp"

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud viewer"));

class PCLTest : public rclcpp::Node
{
public:
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

  const float cloud_normal_radius = 0.03;
  const float kGraspPlaneApprox = 0.007;


  sensor_msgs::msg::PointCloud2 pointcloud2;

  //ROS Pub/Sub
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

  //Object Vector Variables
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects_cloud;
  std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> objects_normal;
  std::vector<Eigen::Vector4f> objects_cutting_plane;
  std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> objects_cutting_plane_points;
  std::vector<Eigen::Vector4f> objects_centerpoints;
  std::vector<pcl::PointXYZ> objects_axis;

  //Functions
  void PCLTest::view_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud);
  void PCLTest::view_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud);
  void PCLTest::center_camera(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud)
  void PCLTest::SensorMsgtoPCLPointCloud2(
    const sensor_msgs::msg::PointCloud2 & pc2,
    pcl::PCLPointCloud2 & pcl_pc2);
  void PCLTest::compute_cloud_normal(int pos);
  void compute_cloud_normal(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & inputCloud,
    const float & searchRadius,
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals)
  void extract_objects(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    float cluster_tolerance,
    int min_cluster_size);
  pcl::PointXYZ get_object_axis(int pos);
  void remove_all_zeros(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  bool passthrough_filter(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    float ptFilter_Ulimit_x,
    float ptFilter_Llimit_x,
    float ptFilter_Ulimit_y,
    float ptFilter_Llimit_y,
    float ptFilter_Ulimit_z,
    float ptFilter_Llimit_z);
  bool plane_segmentation(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    int segmentation_max_iterations,
    float segmentation_distance_threshold);


  PCLTest()
  : Node("pcl_node"),
    cloud(new pcl::PointCloud<pcl::PointXYZRGB>())
  {
    std::cout << "Start" << std::endl;
    ptFilter_Ulimit_x = 0.50;
    ptFilter_Llimit_x = -0.50;
    ptFilter_Ulimit_y = 0.40;
    ptFilter_Llimit_y = -0.55;
    ptFilter_Ulimit_z = 0.6;
    ptFilter_Llimit_z = 0.01;

    segmentation_max_iterations = 50;
    segmentation_distance_threshold = 0.02;

    cluster_tolerance = 0.01;
    min_cluster_size = 750;

    std::cout << "Waiting for topic...." << std::endl;
    cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/pointcloud", 10, std::bind(&PCLTest::planning_init, this, std::placeholders::_1));
  }

  void planning_init(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PCLPointCloud2 * pcl_pc2(new pcl::PCLPointCloud2);
    SensorMsgtoPCLPointCloud2(*msg, *pcl_pc2);
    pcl::fromPCLPointCloud2(*pcl_pc2, *cloud);
    std::cout << "[Raw Cloud] Number of points: " << cloud->points.size() << std::endl;
    std::cout << "[Raw Cloud] Width = " << cloud->width << std::endl;
    std::cout << "[Raw Cloud] Height = " << cloud->height << std::endl;

    view_cloud(cloud);
    passthrough_filter(
      cloud,
      ptFilter_Ulimit_x,
      ptFilter_Llimit_x,
      ptFilter_Ulimit_y,
      ptFilter_Llimit_y,
      ptFilter_Ulimit_z,
      ptFilter_Llimit_z);
    center_camera(cloud);
    std::cout << "[Passthrough Filter] Number of points: " << cloud->points.size() << std::endl;
    std::cout << "[Passthrough Filter] Width = " << cloud->width << std::endl;
    std::cout << "[Passthrough Filter] Height = " << cloud->height << std::endl;
    view_cloud(cloud);
    plane_segmentation(cloud, segmentation_max_iterations, segmentation_distance_threshold);
    std::cout << "[Plane Segmentation] Number of points: " << cloud->points.size() << std::endl;
    std::cout << "[Plane Segmentation] Width = " << cloud->width << std::endl;
    std::cout << "[Plane Segmentation] Height = " << cloud->height << std::endl;

    remove_all_zeros(cloud);
    view_cloud(cloud);
    extract_objects(cloud, cluster_tolerance, min_cluster_size);
    // center_camera(viewer, object_cloud_vec[0]);
    for (int i = 0; i < static_cast<int>(object_cloud_vec.size()); i++) {
      get_object_axis(objects_cloud[i]);
      std::vector<pcl::PointXYZ> objects_axis;
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLTest>());
  std::cout << "Shutting Down" << std::endl;
  rclcpp::shutdown();
  return 0;
}
