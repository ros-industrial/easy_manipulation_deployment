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

#ifndef EMD__GRASP_PLANNER__END_EFFECTORS__FINGER_GRIPPER_HPP_
#define EMD__GRASP_PLANNER__END_EFFECTORS__FINGER_GRIPPER_HPP_

// Main PCL files
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// For Visualization
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include "visualization_msgs/msg/marker.hpp"

// ROS2 Libraries
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Custom msgs
#include <emd_msgs/msg/grasp_method.hpp>

// Other Libraries
#include <math.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <limits>

// For Multithreading
#include <future>

// EMD libraries
#include "emd/common/pcl_functions.hpp"
#include "emd/common/fcl_functions.hpp"
#include "emd/common/math_functions.hpp"
#include "emd/common/pcl_visualizer.hpp"
#include "emd/grasp_planner/end_effectors/end_effector.hpp"


/*! \brief General Struct for a single finger in a gripper sample  */
struct SingleFinger
{
  /*! \brief 3D coordinate of point */
  pcl::PointNormal finger_point;
  /*! \brief Distance between finger to center point of object */
  float centroid_dist;
  /*! \brief Distance between point to the finger's grasp plane */
  float grasp_plane_dist;
  /*! \brief Curvature at this Point */
  float curvature;
  /*! \brief Index of plane this finger belongs to with respect to the plane vector */
  int plane_index;
  /*! \brief Cosine of angle of Finger normal with respect to the plane normal vector */
  float angle_cos;

  /**
* SingleFinger Constructor
*
* \param[in] finger_point_ Coordinate of finger point
* \param[in] centroid_dist_ Distance between Finger point and center of gripper
* \param[in] grasp_plane_dist_ Distance between Finger point and grasp plane
* \param[in] curvature_  Curvature of that particular point
* \param[in] plane_index_ Plane this point is in with respect to the plane vector
  */

  SingleFinger(
    pcl::PointNormal finger_point_,
    float centroid_dist_,
    float grasp_plane_dist_,
    float curvature_,
    int plane_index_)
  : finger_point(finger_point_),
    centroid_dist(centroid_dist_),
    grasp_plane_dist(grasp_plane_dist_),
    curvature(curvature_),
    plane_index(plane_index_)
  {}
  SingleFinger() = default;
};

/*! \brief General Struct for a finger grasp sample  */
struct MultiFingerGripper
{
  /*! \brief Distance between the closed fingers when grasping object */
  float closed_finger_stroke;
  /*! \brief Closed finger configuration on side 1 */
  std::vector<std::shared_ptr<SingleFinger>> closed_fingers_1;
  /*! \brief Closed finger configuration on side 2 */
  std::vector<std::shared_ptr<SingleFinger>> closed_fingers_2;
  /*! \brief Currently not used (I think) */
  std::vector<int> closed_fingers_1_index;
  /*! \brief Currently not used (I think) */
  std::vector<int> closed_fingers_2_index;
  /*! \brief 3D coordinates of the open configuration for finger gripper on side 1 */
  std::vector<Eigen::Vector3f> open_fingers_1;
  /*! \brief 3D coordinates of the open configuration for finger gripper on side 2*/
  std::vector<Eigen::Vector3f> open_fingers_2;
  /*! \brief Angle of multifinger gripper with respect to the object axis*/
  float grasp_plane_angle_cos;
  /*! \brief Rank of current Gripper*/
  float rank;
  /*! \brief Pose of gripper*/
  geometry_msgs::msg::PoseStamped pose;
  /*! \brief Marker representing the grasp points of open configuration on both sides */
  visualization_msgs::msg::Marker marker;
  /*! \brief Centerpoint of gripper */
  pcl::PointXYZ gripper_palm_center;
  /*! \brief Middle finger on side 1 */
  std::shared_ptr<SingleFinger> base_point_1;
  /*! \brief Middle finger on side 2 */
  std::shared_ptr<SingleFinger> base_point_2;
  /*! \brief True if colliding with world */
  bool collides_with_world;
  /*! \brief Unit vector of grasping direction */
  Eigen::Vector3f grasping_direction;
  /*! \brief Unit vector perpendicular to grasping direction */
  Eigen::Vector3f grasping_normal_direction;
  /*! \brief Unit vector of approach direction of gripper */
  Eigen::Vector3f grasp_approach_direction;

  /**
    * MultiFingerGripper Constructor
    *
    * \param[in] base_point_1_ middle single finger on side 1
    * \param[in] base_point_2_ middle single finger on side 2
    * \param[in] grasping_direction_ vector representing the grasping direction
    * \param[in] grasping_normal_direction_ vector perpendicular to the grasping direction
    */
  MultiFingerGripper(
    std::shared_ptr<SingleFinger> base_point_1_,
    std::shared_ptr<SingleFinger> base_point_2_,
    Eigen::Vector3f grasping_direction_,
    Eigen::Vector3f grasping_normal_direction_)
  : base_point_1(base_point_1_),
    base_point_2(base_point_2_)
  {
    grasping_direction = grasping_direction_.normalized();
    grasping_normal_direction = grasping_normal_direction_.normalized();
    //grasp_approach_direction = grasping_direction.cross(grasping_normal_direction);
    grasp_approach_direction = grasping_normal_direction.cross(grasping_direction); // Next time check which cross which
    gripper_palm_center.x = (base_point_1_->finger_point.x + base_point_2_->finger_point.x) / 2;
    gripper_palm_center.y = (base_point_1_->finger_point.y + base_point_2_->finger_point.y) / 2;
    gripper_palm_center.z = (base_point_1_->finger_point.z + base_point_2_->finger_point.z) / 2;
    closed_finger_stroke = pcl::geometry::squaredDistance(
      base_point_1_->finger_point,
      base_point_2_->finger_point);
    rank = 0;
  }
};

/*! \brief General Struct for a Finger Cloud Sample  */
struct FingerCloudSample
{
  /*! \brief Point cloud for current finger sample */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr finger_cloud;
  /*! \brief Point cloud normals for current finger sample */
  pcl::PointCloud<pcl::PointNormal>::Ptr finger_ncloud;
  /*! \brief Downsampled Point cloud normals for current finger sample */
  pcl::PointCloud<pcl::PointNormal>::Ptr finger_nvoxel;
  /*! \brief Maximum distance of a point in the cloud sample to the center of object */
  float centroid_dist_min;
  /*! \brief Minimum distance of a point in the cloud sample to the center of object */
  float centroid_dist_max;
  /*! \brief Minimum distance of a point in the cloud sample to the grasp plane */
  float grasp_plane_dist_min;
  /*! \brief Maximum distance of a point in the cloud sample to the grasp plane */
  float grasp_plane_dist_max;
  /*! \brief Minimum Curvature of point in cloud sample */
  float curvature_min;
  /*! \brief Maximum Curvature of point in cloud sample */
  float curvature_max;
  int start_index;
  /*! \brief Finger Samples derived from current finger cloud */
  std::vector<std::shared_ptr<SingleFinger>> finger_samples;

  /**
* FingerCloudSample Constructor
  */
  FingerCloudSample()
  : finger_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
    finger_ncloud(new pcl::PointCloud<pcl::PointNormal>),
    finger_nvoxel(new pcl::PointCloud<pcl::PointNormal>)
  {
    centroid_dist_min = std::numeric_limits<float>::max();
    centroid_dist_max = std::numeric_limits<float>::min();
    grasp_plane_dist_min = curvature_min = centroid_dist_min;
    grasp_plane_dist_max = curvature_max = centroid_dist_max;
    start_index = -1;
  }
};

/*! \brief General Struct for a grasp plane  */
struct GraspPlaneSample
{
  /*! \brief Index of this current plane with respect to plane vector */
  int plane_index;
  /*! \brief True if cutting plane intersects object */
  bool plane_intersects_object;
  /*! \brief Distance of this plane to the center plane */
  float dist_to_center_plane;
  /*! \brief Point cloud on plane containing points representing side 1 */
  std::shared_ptr<FingerCloudSample> sample_side_1;
  /*! \brief Point cloud on plane containing points representing side 2 */
  std::shared_ptr<FingerCloudSample> sample_side_2;
  /*! \brief Normal Point cloud strip for this grasp plane */
  pcl::PointCloud<pcl::PointNormal>::Ptr grasp_plane_ncloud;
  /*! \brief Coefficients for this current plane */
  pcl::ModelCoefficients::Ptr plane;
  /*! \brief Coefficients for this current plane as Eigen 4f */
  Eigen::Vector4f plane_eigen;

  /**
* GraspPlaneSample Constructor
  */
  GraspPlaneSample()
  : grasp_plane_ncloud(new pcl::PointCloud<pcl::PointNormal>),
    plane(new pcl::ModelCoefficients)
  {
    FingerCloudSample point_1;
    sample_side_1 = std::make_shared<FingerCloudSample>(point_1);
    FingerCloudSample point_2;
    sample_side_2 = std::make_shared<FingerCloudSample>(point_2);
  }
};

/*! \brief General Struct for Finger gripper grasp planning  */
class FingerGripper : public EndEffector
{
public:
  /**
   * Finger Gripper Constructor
   *
   * \param[in] id_ gripper id
   * \param[in] num_fingers_side_1_ Number of fingers on side 1
   * \param[in] num_fingers_side_2_ Number of fingers on side 2
   * \param[in] distance_between_fingers_1_ Distance between fingers at side 1
   * \param[in] distance_between_fingers_2_ Distance between fingers at side 2
   * \param[in] finger_thickness_ largest dimension of the finger
   * \param[in] gripper_stroke_ distance between opposing gripper fingers
   * \param[in] voxel_size_ Voxel size for downsampling
   * \param[in] grasp_quality_weight1_ weight 1 of grasp ranking
   * \param[in] grasp_quality_weight2_ weight 2 of grasp ranking
   * \param[in] grasp_plane_dist_limit_ Parameter for SAC search for points on plane
   * \param[in] cloud_normal_radius_ Radius of which to determine cloud normals
   * \param[in] worldXAngleThreshold_ Threshold after which the object is angled to the world X axis
   * \param[in] worldYAngleThreshold_ Threshold after which the object is angled to the world Y axis
   * \param[in] worldZAngleThreshold_ Threshold after which the object is angled to the world Z axis
   * \param[in] grasp_stroke_direction_ Axis in the same direction as the gripper stroke
   * \param[in] grasp_stroke_normal_direction_ Axis normal to the direction of the gripper stroke
   * \param[in] grasp_approach_direction_ Axis in which the gripper approaches the object
   */
  FingerGripper(
    std::string id_,
    const int & num_fingers_side_1_,
    const int & num_fingers_side_2_,
    const float & distance_between_fingers_1_,
    const float & distance_between_fingers_2_,
    const float & finger_thickness_,
    const float & gripper_stroke_,
    const float & voxel_size_,
    const float & grasp_quality_weight1_,
    const float & grasp_quality_weight2_,
    const float & grasp_plane_dist_limit_,
    const float & cloud_normal_radius_,
    const float & worldXAngleThreshold_,
    const float & worldYAngleThreshold_,
    const float & worldZAngleThreshold_,
    std::string grasp_stroke_direction_,
    std::string grasp_stroke_normal_direction_,
    std::string grasp_approach_direction_);


  /// Get the derived gripper attributes that will be used in the grasp samples generation
  void generate_gripper_attributes();

  /// Inherited method that visualizes the required grasps
  /**
   * \param[in] object Grasp Object
   * \param[in] grasp_method Grasp method output for all possible grasps
   * \param[in] world_collision_object FCL collision object of the world
   * \return average collision checking duration
   */
  void plan_grasps(
    GraspObject & object,
    emd_msgs::msg::GraspMethod & grasp_method,
    std::shared_ptr<CollisionObject> world_collision_object,
    std::string camera_frame);

  /// Reset Gripper Variables
  void reset_variables();

  /**
   * Inherited method that visualizes the required grasps
   *
   * \param[in] viewer Projected Cloud Visualizer
   */
  void visualize_grasps(
    pcl::visualization::PCLVisualizer::Ptr viewer,
    const GraspObject & object);

  std::string get_id() {return id;}

  /*! \brief Gripper ID*/
  std::string id;
  /*! \brief Total Number of finger grippers*/
  int num_fingers_total;
  /*! \brief Number of fingers on side 1*/
  const int num_fingers_side_1;
  /*! \brief Number of fingers on side 2*/
  const int num_fingers_side_2;
  /*! \brief Distance between fingers on side 1*/
  const float distance_between_fingers_1;
  /*! \brief Distance between fingers on side 2*/
  const float distance_between_fingers_2;
  /*! \brief Largest thickness dimension of a finger*/
  const float finger_thickness;
  /*! \brief Distance between fingers of opposing sides*/
  const float gripper_stroke;

protected:
  /*! \brief Voxel Size for downsampling of point cloud */
  const float voxel_size;
  /*! \brief weight 1 of grasp ranking */
  const float grasp_quality_weight1;
  /*! \brief weight 2 of grasp ranking */
  const float grasp_quality_weight2;
  /*! \brief Parameter for SAC search for points on plane */
  const float grasp_plane_dist_limit;
  /*! \brief Radius of which to determine cloud normals */
  const float cloud_normal_radius;
  /*! \brief Threshold after which the object is angled to the world X axis */
  const float worldXAngleThreshold; // Remove
  /*! \brief Threshold after which the object is angled to the world Y axis */
  const float worldYAngleThreshold; // Remove
  /*! \brief Threshold after which the object is angled to the world Z axis */
  const float worldZAngleThreshold; // Remove
  /*! \brief Axis in the same direction as the gripper stroke */
  const char grasp_stroke_direction;
  /*! \brief Axis normal to the direction of the gripper stroke */
  const char grasp_stroke_normal_direction;
  /*! \brief Axis in which the gripper approaches the object */
  const char grasp_approach_direction;

  /*! \brief Coefficients of the cutting plane through the object */
  Eigen::Vector4f center_cutting_plane;  // grasp Plane vector coeff: a, b, c ,d
  /*! \brief Normal vector of the cutting plane */
  Eigen::Vector3f center_cutting_plane_normal;
  /*! \brief List of finger grasp samples */
  std::vector<std::shared_ptr<GraspPlaneSample>> grasp_samples;
  /*! \brief Vector containing distance of each cutting plane to the center */
  std::vector<float> cutting_plane_distances;
  /*! \brief Vector containing the indexes of the cutting planes belonging to side 1 */
  std::vector<int> plane_1_index;
  /*! \brief Vector containing the indexes of the cutting planes belonging to side 2 */
  std::vector<int> plane_2_index;
  /*! \brief  */
  std::vector<std::vector<std::shared_ptr<SingleFinger>>> gripper_clusters;
  /*! \brief Vector containing grasp samples sorted by ranks */
  std::vector<std::shared_ptr<MultiFingerGripper>> sorted_gripper_configs;
  /*! \brief True if number of fingers in side 1 is even */
  bool is_even_1;
  /*! \brief True if number of fingers in side 2 is even */
  bool is_even_2;
  /*! \brief Initial gap between the center of finger gripper and the next gripper in side 1 */
  float initial_gap_1;
  /*! \brief Initial gap between the center of finger gripper and the next gripper in side 2 */
  float initial_gap_2;
  /*! \brief Number of iterations for Finger gripper to be populated in side 1 */
  int num_itr_1;
  /*! \brief Number of iterations for Finger gripper to be populated in side 2 */
  int num_itr_2;

  /**
   * Function that determines the start points on the center plane of the object depending
   * on the angle of the object with respect to the world. This start point will represent
   * the center of the finger cloud on that side of this plane.
   */
  bool get_initial_sample_points(const GraspObject & object);

  /**
   * Get a vector that is perpendicular to another vector that is on the same plane
   * Equations below is a culmination of simultaneous equations
   * \param[in] target_vector Target vector to be perpendicular to
   * \param[in] plane Target plane that both vectors need to be on.
   */
  Eigen::Vector3f get_perpendicular_vector_in_plane(
    Eigen::Vector3f target_vector,
    pcl::ModelCoefficients::Ptr plane);

  /**
   * Function that returns the roll, pitch and yaw from coplanar points. This method
   * accounts for coodinate systems where the z axis may not be the "downward direction" for
   * grasp approach
   *
   * \param[in] grasp_direction Vector representing the direction of gripper stroke
   * \param[in] grasp_direction_normal Vector representing the direction perpendicular to the stroke
   */
  std::vector<double> get_planar_rpy(
    const Eigen::Vector3f & grasp_direction,
    const Eigen::Vector3f & grasp_direction_normal);

  /**
   * Given the contact points of 2 fingers on the surface of an object, find the
   * coordinates of the initial open configuration of the fingers depending on the
   * stroke
   * \param[in] grasp_direction Vector representing the grasp direction
   * \param[in] finger_1 Coordinates of finger at side 1
   * \param[in] finger_2 Coordinates of finger at side 2
   */
  std::vector<Eigen::Vector3f> get_open_finger_coordinates(
    const Eigen::Vector3f & grasp_direction,
    const Eigen::Vector3f & finger_1,
    const Eigen::Vector3f & finger_2);

  /**
   * Method that generates the plane in which a particular gripper lies on
   * \param[in] finger_sample_1 Finger sample on side 1
   * \param[in] finger_sample_2 Finger sample on side 2
   * \param[in] grasp_direction Vector representing grasp direction
   * \param[in] object Grasp object
   */
  Eigen::Vector3f get_gripper_plane(
    std::shared_ptr<SingleFinger> & finger_sample_1,
    std::shared_ptr<SingleFinger> & finger_sample_2,
    const Eigen::Vector3f & grasp_direction,
    const GraspObject & object);

  /**
   * Function to add cutting planes if both sides of the end effector is odd or even.
   * This is because each plane contains finger points on both sides of the end effector
   * \param[in] centerpoint centerpoint of gripper
   * \param[in] plane_vector centerplane gripper
   * \param[in] both_sides_even True if both sides of the end effector is even
   */
  void add_cutting_planes_equal_aligned(
    const Eigen::Vector4f & centerpoint,
    const Eigen::Vector4f & plane_vector,
    const bool & both_sides_even);

  /**
   * Function to add cutting planes if both sides of the end effector is not both odd or even.
   * This is because each plane may or may not contain the finger on each side.
   * \param[in] centerpoint centerpoint of gripper
   * \param[in] plane_vector centerplane gripper
   * \param[in] num_itr_1 Number of iterations to find the cutting planes for all fingers in side 1. Each iteration finds planes above and below the center plane with a given gap
   * \param[in] num_itr_2 Number of iterations to find the cutting planes for all fingers in side 2. Each iteration finds planes above and below the center plane with a given gap
   * \param[in] initial_gap_1 Initial Gap from the center plane to the next cutting plane for side 1
   * \param[in] initial_gap_2 Initial Gap from the center plane to the next cutting plane for side 2
   */
  void add_cutting_planes(
    const Eigen::Vector4f & centerpoint,
    const Eigen::Vector4f & plane_vector,
    const int & num_itr_1,
    const int & num_itr_2,
    const float & initial_gap_1,
    const float & initial_gap_2);

  /**
   * Function check if the current plane created with a certain dist from the center plane
   * exists. This is to prevent multiple planes created if a finger on both sides share the same plane
   * Returns the index of the plane if it exists, if not, -1
   * \param[in] dist Distance from the center plane to the current plane checked
   */
  int check_plane_exists(const float & dist);

  /**
   * Function that creates a cutting plane and a GraspPlaneSample instance that will
   * be added to the gripper instance's vector of plane samples
   * \param[in] dist Distance from the center plane to the current plane checked
   * \param[in] centerpoint Centerpoint of the plane
   * \param[in] plane_vector Vector of the plane
   * \param[in] inside_1 True if a finger in side 1 is on this plane
   * \param[in] inside_2 True if a finger in side 2 is on this plane
   */
  void add_plane(
    const float & dist,
    const Eigen::Vector4f & centerpoint,
    const Eigen::Vector4f & plane_vector,
    const bool & inside_1,
    const bool & inside_2);

  void add_plane(
    float dist,
    Eigen::Vector4f centerpoint,
    Eigen::Vector4f plane_vector);

  /**
   * Given a vector of planes, find the index of the plane that represents a certain
   * distance from the center plane.
   * \param[in] target_distance Distance target plane is from the center plane
   */
  int get_nearest_plane_index(float target_distance);

  /**
   * Given a vector of planes, find the index of the plane that represents a certain
   * distance from the center plane.
   * \param[in] target_point target point of reference
   * \param[in] cloud cloud to search
   */
  int get_nearest_point_index(
    const pcl::PointNormal & target_point,
    const pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

  /**
   * Function to create the finger gripper configuration.
   * Returns the created finger gripper sample. Finger sample is created using the center part of the
   * gripper and then based on the grasp direction the finger will be expanded outwards.
   * \param[in] world_collision_object Collision object representing the world
   * \param[in] closed_center_finger_1 Finger representation of the closed configuration for the center finger in side 1.
   * \param[in] closed_center_finger_2 Finger representation of the closed configuration for the center finger in side 2.
   * \param[in] open_center_finger_1 Finger representation of the open configuration for the center finger in side 1.
   * \param[in] open_center_finger_2 Finger representation of the open configuration for the center finger in side 2.
   * \param[in] plane_normal_normalized Normal vector of the center plane.
   */
  std::shared_ptr<MultiFingerGripper> generate_gripper_open_config(
    const std::shared_ptr<CollisionObject> & world_collision_object,
    const std::shared_ptr<SingleFinger> & closed_center_finger_1,
    const std::shared_ptr<SingleFinger> & closed_center_finger_2,
    const Eigen::Vector3f & open_center_finger_1,
    const Eigen::Vector3f & open_center_finger_2,
    const Eigen::Vector3f & plane_normal_normalized,
    const Eigen::Vector3f & grasp_direction,
    std::string camera_frame);

  /**
   * Function to check the finger collision with the world
   * Returns true if the finger collides with the world.
   * \param[in] finger_point Coordinates for the current finger
   * \param[in] world_collision_object Collision object representing the world
   */
  bool check_finger_collision(
    const Eigen::Vector3f & finger_point,
    const std::shared_ptr<CollisionObject> & world_collision_object);

  /**
   * Method that generates the grasp sample for a particular plane vector
   * \param[in] plane_vector vector of plane cutting the object
   * \param[in] point_on_plane 3d point on plane
   * \param[in] dist_to_center_plane perpendicular distance from center cutting plane
   * \param[in] plane_index Index of plane with respect to the plane vector
   */
  std::shared_ptr<GraspPlaneSample> generate_grasp_samples(
    Eigen::Vector4f plane_vector,
    Eigen::Vector3f point_on_plane,
    float dist_to_center_plane,
    int plane_index);

  /**
   * Function that gets the cluster of point clouds based on a certain radius from
   * a certain start index that has already been defined in another method. This cluster
   * of point clouds will repreesnt the gripper finger point cloud at that point on the plane
   */
  void get_initial_sample_cloud(const GraspObject & object);

  /**
   * Function to create a grasp cloud. Thisis done through SAC which finds points within the
   * grasp_plane_dist_limit. This create a strip of point cloud about the grasp plane.
   * \param[in] object grasp object
   */
  bool get_grasp_cloud(const GraspObject & object);

  /**
   * Function find the plane that cuts the center of the Grasp Object
   * \param[in] object grasp object
   */
  void get_center_cutting_plane(const GraspObject & object);

  /**
   * Function to create cutting planes along the object. each cutting plane represents
   * the planar contact area for two fingers. For multi-fingered gripper, multiple planes are created,
   * \param[in] object grasp object
   */
  void get_cutting_planes(const GraspObject & object);

  /**
   * Function to voxelize a sample cloud
   * This is used to voxelize finger sample cloud for easier traversal
   */
  void voxelize_sample_cloud();

  /**
   * Get the current rank of a multifinger gripper. This is an extension of the implementation
   * by the research paper.
   * \param[in] gripper Target gripper
   */
  void get_gripper_rank(std::shared_ptr<MultiFingerGripper> gripper);

  /**
   * Function that gets the rank of each MultiFingerGripper instance, and sort them into a vector
   * of decreasing rank. This function also populates the GraspMethod message which will be sent
   * To the grasp execution
   * \param[in] input_vector Unsorted vector of gripper instances
   * \param[in] grasp_method GraspMethod output for grasp execution
   */
  std::vector<std::shared_ptr<MultiFingerGripper>> get_all_ranks(
    std::vector<std::shared_ptr<MultiFingerGripper>> input_vector,
    emd_msgs::msg::GraspMethod & grasp_method);

  /**
   * Now that we have generated the various FingerCloudSamples for each plane
   * (2 per plane), we now choose the correct finger clouds that corresponds to
   * the multifinger gripper. TODO:
   */
  void get_gripper_clusters();

  void get_gripper_samples_from_clusters();

  /**
   * Method that generates all possible finger grasp samples
   *
   * \param[in] object Grasp Object
   */
  void get_finger_samples(const GraspObject & object);

  /**
   * Function to create all possible gripper configurations. Returns a vector
   * containing the configurations.
   * \param[in] object Object to be grasped
   * \param[in] world_collision_object Collision object representing the world
   */
  std::vector<std::shared_ptr<MultiFingerGripper>> get_all_gripper_configs(
    const GraspObject & object,
    const std::shared_ptr<CollisionObject> & world_collision_object,
    std::string camera_frame);

  /**
   * Function that gets the maximum and minimum values of certain grasp ranking attributes within
   * a point cloud Each plane will have its own independant maximum and minimum values, we only
   * compare between the 2 voxelized clouds in each plane
   * \param[in] object Grasp object.
   */
  void get_max_min_values(const GraspObject & object);

  /**
   * Function that updates the max and min values of ranking attributes of
   * all finger samples at the particular section of the gripper. Used for
   * normalization later.

  * \param[in] sample FingerCloudSample for the current finger
  * \param[in] centroid_distance Distance from finger to object centroid
  * \param[in] grasp_plane_distance Distance from finger sample to grasp plane
  * \param[in] curvature Curvature of surface at finger point
  */
  void update_max_min_attributes(
    std::shared_ptr<FingerCloudSample> & sample,
    const float & centroid_distance,
    const float & grasp_plane_distance,
    const float & curvature);

  /**
   * Function that gets the grasp pose of a target gripper based on the object.
   * Current implementation of the gripper pose orientation is in relation to the
   * Object orientation, which should be changed in the future.
   * \param[in] gripper Target gripper
   * \param[in] object Target object
  */
  void get_grasp_pose(
    std::shared_ptr<MultiFingerGripper> gripper,
    const GraspObject & object);

  /**
   * Method that generates the grasp option for closed finger distance for
   * finger grippers
   * \param[in] gripper Finger gripper sample
   */
  emd_msgs::msg::Option add_closed_grasp_distance_option(
    std::shared_ptr<MultiFingerGripper> gripper);
};

#endif  // EMD__GRASP_PLANNER__END_EFFECTORS__FINGER_GRIPPER_HPP_
