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


//ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

class PCLPub : public rclcpp::Node
{
  std::string incloudfile;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

  sensor_msgs::msg::PointCloud2 pointcloud2;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> object_cloud_vec;

  //ROS Pub/Sub
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub;

public:
  PCLTest() : Node("pcl_node"),
    cloud(new pcl::PointCloud<pcl::PointXYZRGB>())
  {
    output_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", 10);
    std::cout << "Start" << std::endl;
    incloudfile = "/home/rosi5/pcl_ws/src/pcl_test/data/objects-example.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(incloudfile.c_str(), *cloud) == -1) { //* load the file
      PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    }
    std::cout << "Loaded " <<
      cloud->width * cloud->height <<
      " data points from " <<
      incloudfile <<
      std::endl;

    sensor::msg::PointCloud2 cloud_output;
  }

  sensor::msg::

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLPub>());
  rclcpp::shutdown();
  return 0;
}
