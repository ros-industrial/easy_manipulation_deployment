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

#ifndef GRASP_PLANNER__END_EFFECTORS__SUCTION_GRIPPER_HPP_
#define GRASP_PLANNER__END_EFFECTORS__SUCTION_GRIPPER_HPP_

// Main PCL files
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/search.h>

// For Plane Segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// For Object Segmentation
#include <pcl/segmentation/extract_clusters.h>

// For Cloud Filtering
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

// For Visualization
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// ROS2 Libraries
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <emd_msgs/msg/grasp_method.hpp>

// Other Libraries
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>
#include <string>
#include <memory>
#include <vector>

// For Multithreading
#include <future>

// EMD libraries
#include "grasp_planner/common/pcl_functions.hpp"
#include "grasp_planner/common/fcl_functions.hpp"
#include "grasp_planner/common/math_functions.hpp"
#include "grasp_planner/end_effectors/end_effector.hpp"


/*! \brief Struct for a single suction cup */
struct singleSuctionCup
{
  /*! \brief Total number of contact points for the suction cup */
  int contact_points;
  /*! \brief Weighted value contact points based on user config */
  int weighted_contact_points;
  /*! \brief Average curvature of the points in the suction cup */
  float average_curvature;
  /*! \brief Sum of all curvature values of points in contact with the suction cup */
  float curvature_sum;
  /*! \brief 3D coordinates of the suction cup */
  pcl::PointXYZ cup_center;
  singleSuctionCup(
    pcl::PointXYZ cup_center_,
    float curvature_sum_,
    float contact_points_,
    float weighted_contact_points_)
  {
    cup_center = cup_center_;
    contact_points = contact_points_;
    weighted_contact_points = weighted_contact_points_;
    curvature_sum = curvature_sum_;
    average_curvature = curvature_sum_ / contact_points_;
  }
};

/*! \brief General Struct for Suction array grasp sample  */
struct suctionCupArray
{
  /*! \brief 2D matrix representing a suction cup array */
  std::vector<std::vector<std::shared_ptr<singleSuctionCup>>> cup_array;
  /*! \brief Center of entire suction cup array */
  pcl::PointXYZ gripper_center;
  /*! \brief Distance from gripper center to the center of object */
  float center_dist;
  /*! \brief Grasp quality of this suction array */
  float rank;
  /*! \brief Total contact points of all the suction cups on this suction array */
  int total_contact_points;
  /*! \brief Total Curvature of all points of the suction cups on this suction array */
  float total_curvature;
  /*! \brief Average Curvature of all points of the suction cups on this suction array */
  float average_curvature;
  /*! \brief Angle of grasp sample */
  float grasp_angle;
  /*! \brief Vector representing the row direction */
  Eigen::Vector3f row_direction;
  /*! \brief Vector representing the col direction */
  Eigen::Vector3f col_direction;

  suctionCupArray(
    pcl::PointXYZ gripper_center_,
    Eigen::Vector3f row_direction_,
    Eigen::Vector3f col_direction_)
  {
    row_direction = row_direction_;
    col_direction = col_direction_;
    gripper_center = gripper_center_;
    total_contact_points = 0;
    rank = 0;
  }
};

/*! \brief General Class for Suction Gripper grasp planning  */
class SuctionGripper : public EndEffector
{
public:
  SuctionGripper(
    std::string id_,
    const int & num_cups_length_,
    const int & num_cups_breadth_,
    const float & dist_between_cups_length_,
    const float & dist_between_cups_breadth_,
    const float & cup_radius_,
    const float & cup_height_,
    const int & num_sample_along_axis_,
    const float & search_resolution_,
    const int & search_angle_resolution_,
    const float & cloud_normal_radius_,
    const float & curvature_weight_,
    const float & grasp_center_distance_weight_,
    const float & num_contact_points_weight_,
    std::string length_direction_,
    std::string breadth_direction_,
    std::string grasp_approach_direction_);

  void generateGripperAttributes();

  void planGrasps(
    std::shared_ptr<GraspObject> object,
    emd_msgs::msg::GraspMethod * grasp_method,
    std::shared_ptr<CollisionObject> world_collision_object);

  std::string getID() {return id;}

  void visualizeGrasps(pcl::visualization::PCLVisualizer::Ptr viewer);

  int getCentroidIndex(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);

  void getAllPossibleGrasps(
    const std::shared_ptr<GraspObject> & object,
    const pcl::PointXYZ & object_center,
    const pcl::PointXYZRGB & top_point);

  bool getCupContactCloud(
    pcl::PointXYZRGB contact_point,
    float radius, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output);

  std::vector<int> getCupContactIndices(
    pcl::PointXYZRGB contact_point,
    float radius, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input);

  pcl::PointXYZRGB findHighestPoint(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
    const char & height_axis,
    const bool & is_positive);

  void getStartingPlane(
    pcl::ModelCoefficients::Ptr plane_coefficients,
    const Eigen::Vector3f & axis,
    const Eigen::Vector4f & object_centerpoint,
    const pcl::PointXYZRGB & top_point,
    const char & height_axis);

  singleSuctionCup generateSuctionCup(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & projected_cloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & sliced_cloud_normal,
    const Eigen::Vector3f & suction_cup_center,
    const pcl::PointXYZ & object_center,
    const float & object_max_dim);

  void getSlicedCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & input_cloud_normal,
    const float & top_limit,
    const float & bottom_limit,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal,
    const char & height_axis);

  void projectCloudToPlane(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
    const pcl::ModelCoefficients::Ptr & plane_coefficients,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud);

  std::vector<int> createDiskFromCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal,
    pcl::PointXYZ centerpoint,
    float radius,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr disk_cloud,
    float * curvature_sum);

  pcl::PointXYZ getGripperCenter(
    const Eigen::Vector3f & object_axis,
    const float & offset,
    const pcl::PointXYZRGB & slice_centroid);

  int getContactPoints(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & sliced_cloud_normal,
    const pcl::PointXYZ & centerpoint,
    float * curvature_sum);

  void updateMaxMinValues(
    const int & num_contact_points,
    const float & average_curvature,
    const float & center_dist);

  void getAllGraspRanks(
    emd_msgs::msg::GraspMethod * grasp_method,
    const std::shared_ptr<GraspObject> & object);

  std::vector<double> getPlanarRPY(
    const Eigen::Vector3f & col_vector,
    const Eigen::Vector3f & row_vector);

  geometry_msgs::msg::PoseStamped getGraspPose(
    const std::shared_ptr<suctionCupArray> & grasp,
    const std::shared_ptr<GraspObject> & object);

  suctionCupArray generateGraspSample(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & projected_cloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & sliced_cloud_normal,
    const pcl::PointXYZ & sample_gripper_center,
    const pcl::PointXYZ & object_center,
    const Eigen::Vector3f & grasp_direction,
    const Eigen::Vector3f & object_direction,
    const float & object_max_dim);

  int generateWeightedContactPoints(
    const int & contact_points,
    const float & cup_to_center_dist,
    const float & object_max_dim);

  float getGap(
    const int & itr,
    const bool & is_even,
    const float & initial_gap,
    const float & dist_between_cups);

  /*! \brief Gripper ID */
  std::string id;
  /*! \brief User Defined: Number of suction cups in the length direction */
  const int num_cups_length;
  /*! \brief User Defined: Number of suction cups in the breadth direction */
  const int num_cups_breadth;
  /*! \brief User Defined: Distance between the suction cups in the length direction */
  const float dist_between_cups_length;
  /*! \brief User Defined: Distance between the suction cups in the breadth direction */
  const float dist_between_cups_breadth;
  /*! \brief User Defined: Radius of each suction cup*/
  const float cup_radius;
  /*! \brief User Defined: Height of each suction cup*/
  const float cup_height;
  /*! \brief User Defined: Number of grasp samples to be sampled along the axis for
  each search iteration*/
  const int num_sample_along_axis;
  /*! \brief User Defined: Step size for each search iteration*/
  const float search_resolution;
  /*! \brief User Defined: Angle resolution for each search iteration.
  Default is 3 for a single 90 degree angle iteration. Min 3, Max 9. */
  const int search_angle_resolution;
  /*! \brief User Defined: Radius used for creating Normal Point Cloud of grasp samples*/
  const float cloud_normal_radius;
  /*! \brief User Defined: Weights to determine importance of curvature.  Default is 1.0 */
  float curvature_weight;
  /*! \brief User Defined: Weights to determine importance of Center Distance. Default is 1.0 */
  float grasp_center_distance_weight;
  /*! \brief User Defined: Weights to determine importance of the number of
  contact points. Default is 1.0 */
  float num_contact_points_weight;

  /*! \brief Axis in the direction of the length vector */
  const char length_direction;
  /*! \brief Axis in the direction of the breadth vector */
  const char breadth_direction;
  /*! \brief Axis in which the gripper approaches the object */
  const char grasp_approach_direction;


  /*! \brief Actual breadth dimensions of the entire suction gripper*/
  float breadth_dim;
  /*! \brief Actual length dimensions of the entire suction gripper*/
  float length_dim;

  /*! \brief True if number of cups is even in the column direction */
  bool col_is_even;
  /*! \brief Number of iterations to generate full suction cup array in the column direction */
  float col_itr;
  /*! \brief Distance between cups in the column direction */
  float col_dist_between_cups;
  /*! \brief Initial distance between center of suction array gripper to the first suction cup in
    the column direction */
  float col_initial_gap;
  /*! \brief Axis in the direction of the col vector */
  char col_direction;

  /*! \brief True if number of cups is even in the row direction */
  bool row_is_even;
  /*! \brief Number of iterations to generate full suction cup array in the row direction */
  float row_itr;
  /*! \brief Distance between cups in the row direction */
  float row_dist_between_cups;
  /*! \brief Initial distance between center of suction array gripper to the first suction cup in
    the row direction */
  float row_initial_gap;
  /*! \brief Axis in the direction of the breadth vector */
  char row_direction;

  // For grasp planning
  /*! \brief Maximum average curvature value, comparing all the grasp samples sampled */
  float max_curvature;
  /*! \brief Maximum average curvature value, comparing all the grasp samples sampled */
  float min_curvature;
  /*! \brief Maximum total contact points, comparing all the grasp samples sampled */
  int max_contact_points;
  /*! \brief Minimum total contact points, comparing all the grasp samples sampled */
  int min_contact_points;
  /*! \brief Maximum distance from the center of grasp to the object center,
  comparing all the grasp samples sampled */
  float max_center_dist;
  /*! \brief Minimum total contact points, comparing all the grasp samples sampled */
  float min_center_dist;
  /*! \brief All sampled grasp array grasps */
  std::vector<std::shared_ptr<suctionCupArray>> cup_array_samples;
};

#endif  // GRASP_PLANNER__END_EFFECTORS__SUCTION_GRIPPER_HPP_
