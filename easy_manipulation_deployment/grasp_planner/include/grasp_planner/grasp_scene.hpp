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

// Marker Array library
#include "visualization_msgs/msg/marker.hpp"

// Other libraries
#include <emd_msgs/msg/grasp_target.hpp>
#include <emd_msgs/msg/grasp_task.hpp>
#include <epd_msgs/msg/epd_object_localization.hpp>
#include <epd_msgs/msg/epd_object_tracking.hpp>
#include <epd_msgs/msg/localized_object.hpp>
#include <emd_msgs/srv/grasp_request.hpp>


// Temp
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// EndTemp

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"

// Custom Libraries
#include "grasp_planner/end_effectors/finger_gripper.hpp"
#include "grasp_planner/end_effectors/suction_gripper.hpp"
#include "grasp_planner/grasp_object.hpp"
#include "grasp_planner/common/conversions.hpp"
#include "grasp_planner/common/pcl_functions.hpp"
#include "grasp_planner/common/fcl_functions.hpp"

static const rclcpp::Logger & LOGGER = rclcpp::get_logger("GraspScene");
namespace grasp_planner
{
template<class T>
/*! \brief General Class for a grasp scene*/
class GraspScene
{
public:
  /*! \brief GraspScene Set Up */
  void setup(std::string topic_name);

  /*! \brief GraspScene Set Up */
  void startPlanning(const typename T::ConstSharedPtr & msg);

  /*! \brief Method to process direct Point Clouds */
  void processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  /*! \brief Method to create collision objects from Point Clouds */
  void createWorldCollision(const typename T::ConstSharedPtr & msg);

  /*! \brief General method to extract grasp objects from Point Clouds */
  void extractObjects(const typename T::ConstSharedPtr & msg);

  /*! \brief Method to extract grasp objects from Point Clouds for Direct Camera workflow */
  void extractObjectsDirect();

  /*! \brief Method to extract grasp objects from Point Clouds for EPD-EMD workflow */
  void extractObjectsEPD(const std::vector<epd_msgs::msg::LocalizedObject> & objects);

  /*! \brief Method to load existing end effectors */
  void loadEndEffectors();

  /*! \brief Method to generate Grasp Task for Grasp Execution tasks */
  emd_msgs::msg::GraspTask generateGraspTask();

  /*! \brief Method to make a request to the Grasp Execution Service */
  void sendToExecution(const emd_msgs::msg::GraspTask & grasp_task);

  /*! \brief Grasp object pose rectification due to Point Cloud limitations */
  void objectPoseRectification(emd_msgs::msg::GraspTask & grasp_task);

  /*! \brief Method to print PoseStamped variables */
  void printPose(const geometry_msgs::msg::PoseStamped & _pose);

  /*! \brief Method to print Pose variables */
  void printPose(const geometry_msgs::msg::Pose & _pose);

  /*! \brief Not used */
  void getCameraPosition();

  /*! \brief GraspScene Constructor */
  GraspScene(const rclcpp::Node::SharedPtr & node_)
  : cloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
    cloud_plane_removed(new pcl::PointCloud<pcl::PointXYZRGB>()),
    org_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
    cloud_table(new pcl::PointCloud<pcl::PointXYZRGB>()),
    table_coeff(new pcl::ModelCoefficients),
    viewer(new pcl::visualization::PCLVisualizer("Cloud viewer")),
    node(node_)
  {
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    this->buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
    this->buffer_->setUsingDedicatedThread(true);
    this->tf_listener = std::make_shared<tf2_ros::TransformListener>(
      *buffer_, node, false);

    auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(),
      node->get_node_timers_interface());

    this->buffer_->setCreateTimerInterface(create_timer_interface);
    // setup(topic_name);
  }

  /*! \brief GraspScene Destructor */
  ~GraspScene() {}

  /*! \brief Subscriber that subscribes to perception output */
  std::shared_ptr<message_filters::Subscriber<T>> perception_sub;
  /*! \brief Message filter for perception output */
  std::shared_ptr<tf2_ros::MessageFilter<T>> tf_perception_sub;
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
  /*! \brief Client that provides the GraspRequest information for the Grasp execution component */
  rclcpp::Client<emd_msgs::srv::GraspRequest>::SharedPtr output_client;
  /*! \brief Futures for GraspRequest request */
  std::shared_future<rclcpp::Client<emd_msgs::srv::GraspRequest>::SharedResponse> result_future;
  /*! \brief Vector of objects in the scene to be picked */
  std::vector<std::shared_ptr<GraspObject>> grasp_objects;
  /*! \brief Vector of End effectors available */
  std::vector<std::shared_ptr<EndEffector>> end_effectors;

  rclcpp::Node::SharedPtr node;
};
}

#endif  // GRASP_PLANNER__GRASP_SCENE_HPP_
