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
#include <epd_msgs/msg/epd_object_tracking.hpp>
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
#include "grasp_planner/common/pcl_visualizer.hpp"
#include "grasp_planner/common/fcl_functions.hpp"

static const rclcpp::Logger & LOGGER = rclcpp::get_logger("GraspScene");

/*! \brief General Class for a grasp scene*/
class GraspScene : public rclcpp::Node
{
public:
  void getCameraPosition();
  std::vector<std::shared_ptr<GraspObject>> extractObjects(
    std::string camera_frame,
    float cloud_normal_radius,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    float cluster_tolerance,
    int min_cluster_size);
  std::vector<std::shared_ptr<GraspObject>> processEPDObjects(
    std::vector<epd_msgs::msg::LocalizedObject> objects,
    std::string camera_frame,
    float cloud_normal_radius);
  void printPose(const geometry_msgs::msg::PoseStamped & _pose);
  void printPose(const geometry_msgs::msg::Pose & _pose);
  void planning_init(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void loadEndEffectors();
  void processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void createWorldCollision(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  std::string generate_task_id();

  // void EPDCreateWorldCollisionObject(
  //   const epd_msgs::msg::EPDObjectLocalization::ConstSharedPtr & msg);
  template<typename T>
  void EPDCreateWorldCollisionObject(
    const T & msg);
  // void planning_init_epd(const epd_msgs::msg::EPDObjectLocalization::ConstSharedPtr & msg);
  template<typename U>
  void planningInit(const U & msg);
  void EPDTrackingCallback(const epd_msgs::msg::EPDObjectTracking::ConstSharedPtr & msg);
  void EPDLocalizationCallback(const epd_msgs::msg::EPDObjectLocalization::ConstSharedPtr & msg);

  ~GraspScene();
  GraspScene();

  // Filter Variables
  /*! \brief Upper limit for passthrough filter in the x direction */
  const float ptFilter_Ulimit_x;
  /*! \brief Lower limit for passthrough filter in the x direction */
  const float ptFilter_Llimit_x;
  /*! \brief Upper limit for passthrough filter in the y direction */
  const float ptFilter_Ulimit_y;
  /*! \brief Lower limit for passthrough filter in the y direction */
  const float ptFilter_Llimit_y;
  /*! \brief Upper limit for passthrough filter in the z direction */
  const float ptFilter_Ulimit_z;
  /*! \brief Lower limit for passthrough filter in the z direction */
  const float ptFilter_Llimit_z;

  // Plane segmentation Variables
  /*! \brief Max iteration for Pointcloud Plane Segmentation */
  const int segmentation_max_iterations;
  /*! \brief Distance threshhold for Pointcloud Plane Segmentation */
  const float segmentation_distance_threshold;

  // Object clustering Variables
  /*! \brief Tolerance when doing point cloud clustering for object segmentation */
  const float cluster_tolerance;
  /*! \brief Minimum size of a cluster to constitute as an object cluster */
  const int min_cluster_size;
  /*! \brief Voxel size for fcl collision object generation */
  const float fcl_voxel_size;


  /*! \brief  */
  std::string incloudfile;
  /*! \brief  */
  std::string outcloudfile;
  /*! \brief Input cloud */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  /*! \brief Input cloud without the plane */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_removed;
  /*! \brief  */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr org_cloud;
  /*! \brief Point cloud representing the surface on which the object is placed on */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_table;
  /*! \brief Coefficient of plane representing surface containing objects */
  pcl::ModelCoefficients::Ptr table_coeff;
  /*! \brief Collision object represented by the input cloud (all in scene) */
  std::shared_ptr<grasp_planner::collision::CollisionObject> world_collision_object;
  /*! \brief PCL Visualizer  */
  pcl::visualization::PCLVisualizer::Ptr viewer;
  /*! \brief Intermediate message type for conversion to PointCloud2 message */
  sensor_msgs::msg::PointCloud2 pointcloud2;

  // For collision checking
  /*! \brief Pointer Buffer */
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  /*! \brief Tf listener to listen for frame transforms */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  /*! \brief Publisher that provides the GraspTask information for the Grasp execution component */
  rclcpp::Publisher<emd_msgs::msg::GraspTask>::SharedPtr output_pub;
  /*! \brief Subscriber that subscribes to the camera output */
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub;
  /*! \brief Message filter for pointcloud message */
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_cloud_sub;
  /*! \brief Vector of objects in the scene to be picked */
  std::vector<std::shared_ptr<GraspObject>> grasp_objects;
  /*! \brief Subscriber that subscribes to the EPD precision 2 output */
  std::shared_ptr<message_filters::Subscriber<epd_msgs::msg::EPDObjectLocalization>>
  epd_localize_sub;
  /*! \brief Message filter for EPD precision 2 message */
  std::shared_ptr<tf2_ros::MessageFilter<epd_msgs::msg::EPDObjectLocalization>> tf_epd_localize_sub;
  /*! \brief Subscriber that subscribes to the EPD precision 3 output */
  std::shared_ptr<message_filters::Subscriber<epd_msgs::msg::EPDObjectTracking>> epd_tracking_sub;
  /*! \brief Message filter for EPD precision 3 message */
  std::shared_ptr<tf2_ros::MessageFilter<epd_msgs::msg::EPDObjectTracking>> tf_epd_tracking_sub;
  /*! \brief Vector of End effectors available */
  std::vector<std::shared_ptr<EndEffector>> end_effectors;


};

#endif  // GRASP_PLANNER__GRASP_SCENE_HPP_
