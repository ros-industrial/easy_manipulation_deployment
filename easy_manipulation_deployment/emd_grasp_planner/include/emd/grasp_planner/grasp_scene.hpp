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

#ifndef EMD__GRASP_PLANNER__GRASP_SCENE_HPP_
#define EMD__GRASP_PLANNER__GRASP_SCENE_HPP_

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
#include <emd_msgs/srv/grasp_request.hpp>

#if EPD_ENABLED == 1
  #include <epd_msgs/msg/epd_object_localization.hpp>
  #include <epd_msgs/msg/epd_object_tracking.hpp>
  #include <epd_msgs/msg/localized_object.hpp>
  #include "epd_msgs/srv/perception.hpp"
#endif

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
#include "emd/grasp_planner/end_effectors/finger_gripper.hpp"
#include "emd/grasp_planner/end_effectors/suction_gripper.hpp"
#include "emd/grasp_planner/grasp_object.hpp"
#include "emd/common/conversions.hpp"
#include "emd/common/pcl_functions.hpp"
#include "emd/common/fcl_functions.hpp"

static const rclcpp::Logger & LOGGER = rclcpp::get_logger("GraspScene");
namespace grasp_planner
{
template<class T>
/*! \brief General Class for a grasp scene*/
class GraspScene
{
public:
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

/**
 * Method to set up all communication methods with perception system for Direct Input
 */
  void setup(std::string topic_name);

/**
 * Function that prints PoseStamped
 * \param[in] _pose target PoseStamped to print
 */
  void print_pose(const geometry_msgs::msg::PoseStamped & _pose);

/**
 * Function that prints Pose messages
 * \param[in] _pose target Pose pose to print
 */
  void print_pose(const geometry_msgs::msg::Pose & _pose);

/*! \brief Pointer of Node Instance*/
  rclcpp::Node::SharedPtr node;

protected:
/**
 * General Callback function for Direct Point Cloud pipeline for tracking and localization
 * \param[in] msg Input message
 */
  void start_planning(const typename T::ConstSharedPtr & msg);

/**
 * Function that processes an input sensor_msgs pointcloud2 message.
 * Includes Conversion to PCL Pointcloud2 type, undergoing passthrough filtering,
 * Removing statistical outlier, downsampling and plane segmentation.
 * \param[in] msg Pointcloud input
 */
  void process_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

/**
 * Function that converts a sensor_msg pointcloud2 message into an FCL compatible
 * collision object.
 * \param[in] msg Pointcloud input
 */
  void create_world_collision(const typename T::ConstSharedPtr & msg);

/**
 * Generic method to Extract Grasp Objects
 * \param[in] msg Input message
 */
  void extract_objects(const typename T::ConstSharedPtr & msg);

/**
 * Function that processes the Objects in a point cloud scene and outputs a vector
 * of GraspObjects. Method to extract grasp objects from Point Clouds for Direct Camera workflow
 */
  void extract_objects_direct();

/**
 * Method that loads all available end effector based on the parameter files
 */
  void load_end_effectors();

/**
 * Method to generate Grasp Tasks for manipulation
 * \param[in] msg Input message
 */
  emd_msgs::msg::GraspTask generate_grasp_task();

/**
 * Function that calls the grasp execution service after all grasp plans are
 * generated.
 * \param[in] grasp_task Grasp Task to send
 */
  void send_to_execution(const emd_msgs::msg::GraspTask & grasp_task);

  #if EPD_ENABLED == 1

  /*! \brief */

/**
 * Function that processes the Objects detected by EPD and outputs a vector
 * of GraspObjects. Method to extract grasp objects from Point Clouds for EPD-EMD workflow
 * \param[in] objects EPD detected objects
 */
  void extract_objects_epd(const std::vector<epd_msgs::msg::LocalizedObject> & objects);

  /*! \brief Method to request service to trigger epd pipeline */
  void trigger_epd_pipeline();

  #endif

  /*! \brief Grasp object pose rectification due to Point Cloud limitations */
  void object_pose_rectification(emd_msgs::msg::GraspTask & grasp_task);

/**
 * Not used
 */
  void get_camera_position();

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

  #if EPD_ENABLED == 1
  /*! \brief Client that triggers the EPD workflow */
  rclcpp::Client<epd_msgs::srv::Perception>::SharedPtr epd_client;
  /*! \brief Futures for EPD service request */
  std::shared_future<rclcpp::Client<epd_msgs::srv::Perception>::SharedResponse> epd_result_future;
  /*! \brief Vector of objects in the scene to be picked */
  #endif

  /*! \brief Vector of Grasp Objects available */
  std::vector<GraspObject> grasp_objects;
  /*! \brief Vector of End effectors available */
  // std::vector<std::shared_ptr<EndEffector>> end_effectors;
  std::vector<EndEffector> end_effectors;
};
}

#endif  // EMD__GRASP_PLANNER__GRASP_SCENE_HPP_
