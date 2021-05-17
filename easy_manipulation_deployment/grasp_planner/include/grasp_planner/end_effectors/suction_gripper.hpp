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
    float weighted_contact_points_){
    cup_center = cup_center_;
    contact_points = contact_points_;
    weighted_contact_points = weighted_contact_points_;
    curvature_sum = curvature_sum_;
    average_curvature = curvature_sum_ / contact_points_;
  }
};
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

  suctionCupArray(pcl::PointXYZ gripper_center_){
    gripper_center = gripper_center_;
    total_contact_points = 0;
    rank = 0;
  }
};

class SuctionGripper : public EndEffector
{
public:
  // Gripper predefined Attributes
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
  /*! \brief Actual breadth dimensions of the entire suction gripper*/
  float breadth_dim;
  /*! \brief Actual length dimensions of the entire suction gripper*/
  float length_dim;

  // Gripper Derived Attributes
  bool col_is_even;
  float col_itr;
  float col_dist_between_cups;
  float col_initial_gap;

  bool row_is_even;
  float row_itr;
  float row_dist_between_cups;
  float row_initial_gap;

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


  Eigen::Matrix4f affine_matrix_test;
  Eigen::Vector3f row_cen_point;

  SuctionGripper(
    std::string id,
    const int & num_cups_length_,
    const int & num_cups_breadth_,
    const float & dist_between_cups_length_,
    const float & dist_between_cups_breadth_,
    const float & cup_radius_,
    const float & cup_height_,
    const int num_sample_along_axis,
    const float search_resolution_,
    const int search_angle_resolution_,
    const float cloud_normal_radius_);

  SuctionGripper(
    std::string id_,
    const int & num_cups_length_,
    const int & num_cups_breadth_,
    const float & dist_between_cups_length_,
    const float & dist_between_cups_breadth_,
    const float & cup_radius_,
    const float & cup_height_,
    const int num_sample_along_axis_,
    const float search_resolution_,
    const int search_angle_resolution_,
    const float cloud_normal_radius_,
    const float curvature_weight_,
    const float grasp_center_distance_weight_,
    const float num_contact_points_weight_);

  void generateGripperAttributes();
  void planGrasps(
    std::shared_ptr<GraspObject> object,
    emd_msgs::msg::GraspMethod * grasp_method,
    std::shared_ptr<CollisionObject> world_collision_object,
    pcl::visualization::PCLVisualizer::Ptr viewer);

  std::string getID() {return id;}

  void visualizeGrasps(pcl::visualization::PCLVisualizer::Ptr viewer);

  int getCentroidIndex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  void getAllPossibleGrasps(
    std::shared_ptr<GraspObject> object,
    pcl::PointXYZ object_center,
    pcl::PointXYZRGB top_point);

  bool getCupContactCloud(
    pcl::PointXYZRGB contact_point,
    float radius, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output);

  std::vector<int> getCupContactIndices(
    pcl::PointXYZRGB contact_point,
    float radius, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input);

  pcl::PointXYZRGB findHighestPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, char height_axis, bool is_positive);

  void getStartingPlane(
    pcl::ModelCoefficients::Ptr plane_coefficients,
    Eigen::Vector3f axis,
    Eigen::Vector4f object_centerpoint,
    pcl::PointXYZRGB top_point,
    char height_axis);

  singleSuctionCup generateSuctionCup(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal,
    Eigen::Vector3f suction_cup_center,
    Eigen::Vector3f object_direction,
    pcl::PointXYZ object_center,
    float object_max_dim);

  void getSlicedCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud_normal,
    float top_limit, float bottom_limit, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal, char height_axis);

  void projectCloudToPlane(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    pcl::ModelCoefficients::Ptr plane_coefficients,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud);

  std::vector<int> createDiskFromCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal,
    pcl::PointXYZ centerpoint, float radius,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr disk_cloud,
    float * curvature_sum);
  
  pcl::PointXYZ getGripperCenter(Eigen::Vector3f object_axis,
    float offset,
    pcl::PointXYZRGB slice_centroid,
    float slice_height,
    char height_axis);
  // Eigen::Vector3f getGraspDirection(Eigen::Vector3f grasp_axis, float angle);
  // Eigen::Vector3f getObjectDirection(Eigen::Vector3f object_axis, float angle);

  int getContactPoints(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal,
    pcl::PointXYZ centerpoint,
    float * curvature_sum);
  

  void updateMaxMinValues(int num_contact_points, float average_curvature, float center_dist);
  void getAllGraspRanks(
    emd_msgs::msg::GraspMethod * grasp_method,
    std::shared_ptr<GraspObject> object);

  geometry_msgs::msg::PoseStamped getGraspPose(
    std::shared_ptr<suctionCupArray> grasp,
    std::shared_ptr<GraspObject> object);

  suctionCupArray generateGraspSample(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal,
    pcl::PointXYZ sample_gripper_center,
    pcl::PointXYZ object_center,
    Eigen::Vector3f grasp_direction,
    Eigen::Vector3f object_direction,
    float object_max_dim);

  int generateWeightedContactPoints(
    int contact_points,
    float cup_to_center_dist,
    float object_max_dim);

  float getGap(
    int itr,
    bool is_even,
    float initial_gap,
    float dist_between_cups);
};

#endif  // GRASP_PLANNER__END_EFFECTORS__SUCTION_GRIPPER_HPP_
