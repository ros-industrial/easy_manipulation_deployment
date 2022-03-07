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

#ifndef EMD__GRASP_PLANNER__END_EFFECTORS__SUCTION_GRIPPER_HPP_
#define EMD__GRASP_PLANNER__END_EFFECTORS__SUCTION_GRIPPER_HPP_

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
#include "emd/common/pcl_functions.hpp"
#include "emd/common/fcl_functions.hpp"
#include "emd/common/math_functions.hpp"
#include "emd/common/pcl_visualizer.hpp"
#include "emd/grasp_planner/end_effectors/end_effector.hpp"


/*! \brief Struct for a single suction cup */
struct SingleSuctionCup
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
  SingleSuctionCup(
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
struct SuctionCupArray
{
  /*! \brief 2D matrix representing a suction cup array */
  std::vector<std::vector<std::shared_ptr<SingleSuctionCup>>> cup_array;
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
  /*! \brief Array of markers representing the grasp points of all suction cups  */
  visualization_msgs::msg::Marker marker;

  SuctionCupArray(
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
/**
 * Suction Gripper Constructor
 *
 * \param[in] id_ gripper id
 * \param[in] num_cups_length_ Number of cups in the length vector
 * \param[in] num_cups_breadth_ Number of cups in the breadth vector
 * \param[in] dist_between_cups_length_ Distance between cups in the length vector
 * \param[in] dist_between_cups_breadth_ Distance between cups in the breadth vector
 * \param[in] cup_radius_ Radius of each suction cup
 * \param[in] cup_height_ Height of suction cup
 * \param[in] num_sample_along_axis_ number of samples along object axis
 * \param[in] search_resolution_ How far of an offset along the object axis to search
 * \param[in] search_angle_resolution_ Angle step for rotation of each grasp sample
 * \param[in] cloud_normal_radius_ Radius of which to determine cloud normals
 * \param[in] curvature_weight_ Weights for the curvature sum of the grasp
 * \param[in] grasp_center_distance_weight_ Weights for the grasp center distance
 * \param[in] num_contact_points_weight_ Weights for the number of contact points
 * \param[in] length_direction_ Axis in the direction of the length vector
 * \param[in] breadth_direction_ Axis in the direction of the breadth vector
 * \param[in] grasp_approach_direction_ Axis in which the gripper approaches the object
 ******/
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

/**
 * Method that generates the relevant gripper attributes before grasp planning
 */
  void generate_gripper_attributes();

/**
 * Inherited method that plans the required grasps
 *
 * \param[in] object Grasp Object
 * \param[in] grasp_method Grasp method output for all possible grasps
 * \param[in] world_collision_object FCL collision object of the world
 * \param[in] camera_frame tf frame representing camera
 */
  void plan_grasps(
    const GraspObject & object,
    emd_msgs::msg::GraspMethod & grasp_method,
    std::shared_ptr<CollisionObject> world_collision_object,
    std::string camera_frame);

  std::string get_id() {return id;}

/**
 * Inherited method that visualizes the required grasps
 *
 * \param[in] viewer Projected Cloud Visualizer
 * \param[in] object Grasp Object to visualize
 */
  void visualize_grasps(
    pcl::visualization::PCLVisualizer::Ptr viewer,
    const GraspObject & object);

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

protected:
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
  std::vector<std::shared_ptr<SuctionCupArray>> cup_array_samples;

  /**
   * Method that gets the index of the centroid of the projected cloud
   *
   * \param[in] cloud Projected Cloud
   */
  int get_centroid_index(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);

  /**
   * Inherited method that gets all possible grasp samples
   *
   * \param[in] object Grasp Object
   * \param[in] object_center PCL centroid point of the object
   * \param[in] top_point Highest point on object
   * \param[in] camera_frame tf frame representing camera
   */
  void get_all_possible_grasps(
    const GraspObject & object,
    const pcl::PointXYZ & object_center,
    const pcl::PointXYZRGB & top_point,
    std::string camera_frame);


  /**
   * Provides the cloud representing the contact with suction gripper
   * (Only for visualization purposes) (CURRENTLY NOT USED)
   * \param[in] contact_point Contact point
   * \param[in] radius Radius of contact point
   * \param[in] cloud_input Input cloud
   * \param[in] cloud_output Result cloud
   */
  bool get_cup_contact_cloud(
    pcl::PointXYZRGB contact_point,
    float radius, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output);

  /**
   * Provides the cloud point indices. For actual implementation.
   * (Only for visualization purposes) (CURRENTLY NOT USED)
   * \param[in] contact_point Contact point
   * \param[in] radius Radius of contact point
   * \param[in] cloud_input Input cloud
   */
  std::vector<int> get_cup_contact_indices(
    pcl::PointXYZRGB contact_point,
    float radius, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input);

  /**
   * Method that defines the highest point of the object
   *
   * \param[in] cloud Object Cloud
   * \param[in] height_axis Closest world axes to the minor axis of the object (WIP)
   * \param[in] is_positive True if aligned to positive closest world axis (WIP)
   */
  pcl::PointXYZRGB find_highest_point(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
    const char & height_axis,
    const bool & is_positive);

  /**
   * Method that defines the plane on which to project the point cloud points on
   *
   * \param[in] plane_coefficients Coefficients of the plane
   * \param[in] axis Major axis of the grasp object
   * \param[in] object_centerpoint Center of grasp object
   * \param[in] top_point Highest point of the grasp object
   * \param[in] height_axis Closest world axes to the minor axis of the object (WIP)
   */
  void get_starting_plane(
    pcl::ModelCoefficients::Ptr plane_coefficients,
    const Eigen::Vector3f & axis,
    const Eigen::Vector4f & object_centerpoint,
    const pcl::PointXYZRGB & top_point,
    const char & height_axis);

  /**
   * Function that generates a single suction cup in an array
   *
   * \param[in] projected_cloud Projected cloud slice on a plane
   * \param[in] sliced_cloud_normal Normals of the sliced cloud
   * \param[in] suction_cup_center Center of Suction Cup
   * \param[in] object_center Center point of object
   * \param[in] object_max_dim Maximum dimensions of obejct
   */
  SingleSuctionCup generate_suction_cup(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & projected_cloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & sliced_cloud_normal,
    const Eigen::Vector3f & suction_cup_center,
    const pcl::PointXYZ & object_center,
    const float & object_max_dim);

  /**
   * Function that slices a cloud to the required limit using a passthrough filter.
   * This function assumes that the direction of filtering is in the z direction,
   * which is parallel to the height vector of the object

  * \param[in] input_cloud Input cloud to be sliced
  * \param[in] input_cloud_normal Input cloud normal to be sliced
  * \param[in] top_limit Top limit for passthrough filter
  * \param[in] bottom_limit Bottom limit for passthrough filter
  * \param[in] sliced_cloud Resultant sliced cloud
  * \param[in] sliced_cloud_normal Resultant sliced cloud
  * \param[in] height_axis Current axis on which the height axis represents(WIP)
  */
  void get_sliced_cloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & input_cloud_normal,
    const float & top_limit,
    const float & bottom_limit,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal,
    const char & height_axis);

  /**
   * Function that projects a cloud to the required plane. This function is used
   * to approximate a relatively flat point cloud onto a plane to start grasp sampling

  * \param[in] input_cloud Input cloud to be sliced
  * \param[in] plane_coefficients Input cloud normal to be sliced
  * \param[in] projected_cloud Top limit for passthrough filter
  */
  void project_cloud_to_plane(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
    const pcl::ModelCoefficients::Ptr & plane_coefficients,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud);

  std::vector<int> create_disk_from_cloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal,
    pcl::PointXYZ centerpoint,
    float radius,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr disk_cloud,
    float & curvature_sum);

  /**
   * Method that calculates center of the suction cup array
   *
   * \param[in] object_axis Vector representing the object
   * \param[in] offset Offset from centroid of sliced cloud
   * \param[in] slice_centroid Center point of sliced cloud
   */
  pcl::PointXYZ get_gripper_center(
    const Eigen::Vector3f & object_axis,
    const float & offset,
    const pcl::PointXYZRGB & slice_centroid);

  int get_contact_points(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & sliced_cloud_normal,
    const pcl::PointXYZ & centerpoint,
    float & curvature_sum);

  /**
   * Function that updates the maximum and minimum values of the attributes
   * required for calculation of ranks, for normalization later on
   *
   * \param[in] num_contact_points Weighted number of contact points on the object
   * \param[in] average_curvature Average Curvature of the points on the gripper
   * \param[in] center_dist Center distance of the gripper to object centroid
   */
  void update_max_min_values(
    const int & num_contact_points,
    const float & average_curvature,
    const float & center_dist);

  /**
   * Method that calculate the ranks of all grasp samples
   *
   * \param[in] grasp_method Output Grasp Method to be used for Grasp Execution
   * \param[in] object Target Grasp Object
   */
  void get_all_grasp_ranks(
    emd_msgs::msg::GraspMethod & grasp_method,
    const GraspObject & object);

  /**
   * Function that returns the roll, pitch and yaw from coplanar points. This method
   * accounts for coodinate systems where the z axis may not be the "downward direction" for
   * grasp approach
   *
   * \param[in] col_vector Vector representing the direction of the column array
   * \param[in] row_vector Vector representing the direction of the row array
   */
  std::vector<double> get_planar_rpy(
    const Eigen::Vector3f & col_vector,
    const Eigen::Vector3f & row_vector);

  /**
   * Method that calculate the pose of the grasp sample
   *
   * \param[in] grasp Target Grasp
   * \param[in] object Target Grasp Object
   */
  geometry_msgs::msg::PoseStamped get_grasp_pose(
    const std::shared_ptr<SuctionCupArray> & grasp,
    const GraspObject & object);

  /**
   * Function that generates a single grasp sample of the user defined end effector
   *
   * \param[in] projected_cloud Projected cloud slice on a plane
   * \param[in] sliced_cloud_normal Normals of the sliced cloud
   * \param[in] sample_gripper_center Center of Suction Array
   * \param[in] object_center Center point of object
   * \param[in] row_direction Vector representing the suction array row direction
   * \param[in] col_direction Vector representing the suction array col direction
   * \param[in] object_max_dim Maximum dimensions of object
   * \param[in] camera_frame tf frame representing camera
   */
  SuctionCupArray generate_grasp_sample(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & projected_cloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & sliced_cloud_normal,
    const pcl::PointXYZ & sample_gripper_center,
    const pcl::PointXYZ & object_center,
    const Eigen::Vector3f & grasp_direction,
    const Eigen::Vector3f & object_direction,
    const float & object_max_dim,
    std::string camera_frame);

  /**
   * Method that calculates the weighted value of the number of contact points
   *
   * \param[in] contact_points Actual number of contact points
   * \param[in] cup_to_center_dist True if the row/column has even cups
   * \param[in] object_max_dim Maximum dimensions of grasp object
   */
  int generate_weighted_contact_points(
    const int & contact_points,
    const float & cup_to_center_dist,
    const float & object_max_dim);

  /**
   * Function that calculates the distance between a suction cup and the gripper
   * array center
   *
   * \param[in] itr Current iteration in the suction cup array iteration
   * \param[in] is_even True if the row/column has even cups
   * \param[in] initial_gap Initial gap between the center of array and the first cup
   * \param[in] dist_between_cups Distance between the suction cups
   */
  float get_gap(
    const int & itr,
    const bool & is_even,
    const float & initial_gap,
    const float & dist_between_cups);
};

#endif  // EMD__GRASP_PLANNER__END_EFFECTORS__SUCTION_GRIPPER_HPP_
