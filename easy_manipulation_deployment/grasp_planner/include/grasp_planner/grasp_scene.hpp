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

#ifndef GRASP_PLANNER__GRASP_SCENE_HPP_
#define GRASP_PLANNER__GRASP_SCENE_HPP_

// Main PCL files
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// ROS2 Libraries
#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>

#include <message_filters/subscriber.h>

// Other libraries
#include <emd_msgs/msg/grasp_target.hpp>
#include <emd_msgs/msg/grasp_task.hpp>
#include <epd_msgs/msg/epd_object_localization.hpp>
#include <epd_msgs/msg/localized_object.hpp>

// Temp
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// EndTemp

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <limits>

// For uuid
#include <random>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

// Custom Libraries
#include "grasp_planner/end_effectors/finger_gripper.hpp"
#include "grasp_planner/end_effectors/suction_gripper.hpp"
#include "grasp_planner/grasp_object.hpp"
#include "grasp_planner/common/conversions.hpp"
#include "grasp_planner/common/pcl_functions.hpp"
#include "grasp_planner/common/fcl_functions.hpp"


class GraspScene: public rclcpp::Node
{
public:
  void getCameraPosition();
  // void planning_init(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  std::vector < std::shared_ptr < GraspObject >> extractObjects(
    std::string camera_frame,
    float cloud_normal_radius,
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud,
    float cluster_tolerance,
    int min_cluster_size);
  std::vector < std::shared_ptr < GraspObject >> processEPDObjects(
    std::vector < epd_msgs::msg::LocalizedObject > objects,
    std::string camera_frame,
    float cloud_normal_radius);
  void printPose(const geometry_msgs::msg::PoseStamped & _pose);
  void printPose(const geometry_msgs::msg::Pose & _pose);
  void planning_init(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void loadEndEffectors();
  void processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void createWorldCollision(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void planning_init_epd(const epd_msgs::msg::EPDObjectLocalization::ConstSharedPtr & msg);
  void EPDCreateWorldCollisionObject(
    const epd_msgs::msg::EPDObjectLocalization::ConstSharedPtr & msg);

  // Filter Variables
  const float ptFilter_Ulimit_x;
  const float ptFilter_Llimit_x;
  const float ptFilter_Ulimit_y;
  const float ptFilter_Llimit_y;
  const float ptFilter_Ulimit_z;
  const float ptFilter_Llimit_z;

  // Plane segmentation Variables
  const int segmentation_max_iterations;
  const float segmentation_distance_threshold;

  // Object clustering Variables
  const float cluster_tolerance;
  const int min_cluster_size;

  // const float cloud_normal_radius;
  // const float grasp_plane_dist_limit;

  // const float worldXAngleThreshold;
  // const float worldYAngleThreshold;
  // const float worldZAngleThreshold;

  std::string incloudfile;
  std::string outcloudfile;
  pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud;
  pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud_plane_removed;
  pcl::PointCloud < pcl::PointXYZRGB > ::Ptr org_cloud;  // REMOVE LATER
  pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud_table;

  pcl::ModelCoefficients::Ptr table_coeff;
  std::shared_ptr < fcl::CollisionObject < float >> world_collision_object;

  pcl::visualization::PCLVisualizer::Ptr viewer;

  sensor_msgs::msg::PointCloud2 pointcloud2;

  // For collision checking
  std::shared_ptr < tf2_ros::Buffer > buffer_;
  std::shared_ptr < tf2_ros::TransformListener > tf_listener;


  // ROS Pub/Sub
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  rclcpp::Publisher < emd_msgs::msg::GraspTask > ::SharedPtr output_pub;
  std::shared_ptr < message_filters::Subscriber < sensor_msgs::msg::PointCloud2 >> cloud_sub;
  std::shared_ptr < tf2_ros::MessageFilter < sensor_msgs::msg::PointCloud2 >> tf_cloud_sub;
  std::vector < std::shared_ptr < GraspObject >> grasp_objects;
  // std::vector<std::shared_ptr<emd_msgs::msg::GraspTask>> grasp_objects;

  std::shared_ptr < message_filters::Subscriber < epd_msgs::msg::EPDObjectLocalization >> epd_sub;
  std::shared_ptr < tf2_ros::MessageFilter < epd_msgs::msg::EPDObjectLocalization >> tf_epd_sub;

  std::vector < std::shared_ptr < EndEffector >> end_effectors;

  GraspScene()
    : Node(
      "grasp_planning_node",
      rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true)),
    ptFilter_Ulimit_x(
      static_cast < float > (this->get_parameter("point_cloud_params.passthrough_filter_limits_x").
      as_double_array()[1])),
    ptFilter_Llimit_x(
      static_cast < float > (this->get_parameter("point_cloud_params.passthrough_filter_limits_x").
      as_double_array()[0])),
    ptFilter_Ulimit_y(
      static_cast < float > (this->get_parameter("point_cloud_params.passthrough_filter_limits_y").
      as_double_array()[1])),
    ptFilter_Llimit_y(
      static_cast < float > (this->get_parameter("point_cloud_params.passthrough_filter_limits_y").
      as_double_array()[0])),
    ptFilter_Ulimit_z(
      static_cast < float > (this->get_parameter("point_cloud_params.passthrough_filter_limits_z").
      as_double_array()[1])),
    ptFilter_Llimit_z(
      static_cast < float > (this->get_parameter("point_cloud_params.passthrough_filter_limits_z").
      as_double_array()[0])),
    segmentation_max_iterations(
      this->get_parameter(
        "point_cloud_params.segmentation_max_iterations").as_int()),
    segmentation_distance_threshold(
      static_cast < float > (this->get_parameter(
        "point_cloud_params.segmentation_distance_threshold").as_double())),
    cluster_tolerance(
      static_cast < float > (this->get_parameter(
        "point_cloud_params.cluster_tolerance").as_double())),
    min_cluster_size(
      this->get_parameter(
        "point_cloud_params.min_cluster_size").as_int()),
    cloud(new pcl::PointCloud < pcl::PointXYZRGB > ()),
    cloud_plane_removed(new pcl::PointCloud < pcl::PointXYZRGB > ()),
    org_cloud(new pcl::PointCloud < pcl::PointXYZRGB > ()),
    cloud_table(new pcl::PointCloud < pcl::PointXYZRGB > ()),
    table_coeff(new pcl::ModelCoefficients),
    viewer(new pcl::visualization::PCLVisualizer("Cloud viewer"))
  {
    output_pub = this->create_publisher < emd_msgs::msg::GraspTask > ("/grasp_tasks", 10);
    rclcpp::Clock::SharedPtr clock = std::make_shared < rclcpp::Clock > (RCL_SYSTEM_TIME);
    this->buffer_ = std::make_shared < tf2_ros::Buffer > (clock);
    this->buffer_->setUsingDedicatedThread(true);
    this->tf_listener = std::make_shared < tf2_ros::TransformListener > (
      *buffer_, this, false);

    auto create_timer_interface = std::make_shared < tf2_ros::CreateTimerROS > (
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    this->buffer_->setCreateTimerInterface(create_timer_interface);
    this->cloud_sub = std::make_shared <
      message_filters::Subscriber < sensor_msgs::msg::PointCloud2 >> (
      this, "/camera/pointcloud");
    this->tf_cloud_sub = std::make_shared < tf2_ros::MessageFilter <
      sensor_msgs::msg::PointCloud2 >> (
      *buffer_, "base_link", 5,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      std::chrono::seconds(1));
    this->tf_cloud_sub->connectInput(*cloud_sub);
    this->tf_cloud_sub->registerCallback(
      std::bind(
        &GraspScene::planning_init, this,
        std::placeholders::_1));


    this->epd_sub = std::make_shared <
      message_filters::Subscriber < epd_msgs::msg::EPDObjectLocalization >> (
      this, "/processor/epd_localize_output");
    this->tf_epd_sub = std::make_shared < tf2_ros::MessageFilter <
      epd_msgs::msg::EPDObjectLocalization >> (
      *buffer_, "base_link", 5,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      std::chrono::seconds(1));
    this->tf_epd_sub->connectInput(*epd_sub);
    this->tf_epd_sub->registerCallback(
      std::bind(
        &GraspScene::planning_init_epd, this,
        std::placeholders::_1));
    std::cout << " waiting... " << std::endl;

    // cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    // this->get_parameter("perception_topic").as_string(),
    // 10, std::bind(&GraspScene::planning_init, this, std::placeholders::_1));
  }
  ~GraspScene();
};

#endif  // GRASP_PLANNER__GRASP_SCENE_HPP_
