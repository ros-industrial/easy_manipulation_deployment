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

#ifndef GRASP_PLANNER__END_EFFECTORS__FINGER_GRIPPER_HPP_
#define GRASP_PLANNER__END_EFFECTORS__FINGER_GRIPPER_HPP_

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
#include "grasp_planner/common/pcl_functions.hpp"
#include "grasp_planner/common/fcl_functions.hpp"
#include "grasp_planner/common/math_functions.hpp"
#include "grasp_planner/end_effectors/end_effector.hpp"
#include "grasp_planner/common/pcl_visualizer.hpp"

/*! \brief General Struct for a single finger in a gripper sample  */
struct singleFinger
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

  /***************************************************************************//**
  * singleFinger Constructor
  *
  * @param finger_point_ Coordinate of finger point
  * @param centroid_dist_ Distance between Finger point and center of gripper
  * @param grasp_plane_dist_ Distance between Finger point and grasp plane
  * @param curvature_  Curvature of that particular point
  * @param plane_index_ Plane this point is in with respect to the plane vector
  ******************************************************************************/

  singleFinger(
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
  singleFinger() = default;
};

/*! \brief General Struct for a finger grasp sample  */
struct multiFingerGripper
{
  /*! \brief Distance between the closed fingers when grasping object */
  float closed_finger_stroke;
  /*! \brief Closed finger configuration on side 1 */
  std::vector<std::shared_ptr<singleFinger>> closed_fingers_1;
  /*! \brief Closed finger configuration on side 2 */
  std::vector<std::shared_ptr<singleFinger>> closed_fingers_2;
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
  std::shared_ptr<singleFinger> base_point_1;
  /*! \brief Middle finger on side 2 */
  std::shared_ptr<singleFinger> base_point_2;
  /*! \brief True if colliding with world */
  bool collides_with_world;
  /*! \brief Unit vector of grasping direction */
  Eigen::Vector3f grasping_direction;
  /*! \brief Unit vector perpendicular to grasping direction */
  Eigen::Vector3f grasping_normal_direction;
  /*! \brief Unit vector of approach direction of gripper */
  Eigen::Vector3f grasp_approach_direction;

  /***************************************************************************//**
  * multiFingerGripper Constructor
  *
  * @param base_point_1_ middle single finger on side 1
  * @param base_point_2_ middle single finger on side 2
  * @param grasping_direction_ vector representing the grasping direction
  * @param grasping_normal_direction_ vector perpendicular to the grasping direction
  ******************************************************************************/
  multiFingerGripper(
    std::shared_ptr<singleFinger> base_point_1_,
    std::shared_ptr<singleFinger> base_point_2_,
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
struct fingerCloudSample
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
  std::vector<std::shared_ptr<singleFinger>> finger_samples;

  /***************************************************************************//**
  * fingerCloudSample Constructor
  ******************************************************************************/
  fingerCloudSample()
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
struct graspPlaneSample
{
  /*! \brief Index of this current plane with respect to plane vector */
  int plane_index;
  /*! \brief True if cutting plane intersects object */
  bool plane_intersects_object;
  /*! \brief Distance of this plane to the center plane */
  float dist_to_center_plane;
  /*! \brief Point cloud on plane containing points representing side 1 */
  std::shared_ptr<fingerCloudSample> sample_side_1;
  /*! \brief Point cloud on plane containing points representing side 2 */
  std::shared_ptr<fingerCloudSample> sample_side_2;
  /*! \brief Normal Point cloud strip for this grasp plane */
  pcl::PointCloud<pcl::PointNormal>::Ptr grasp_plane_ncloud;
  /*! \brief Coefficients for this current plane */
  pcl::ModelCoefficients::Ptr plane;
  /*! \brief Coefficients for this current plane as Eigen 4f */
  Eigen::Vector4f plane_eigen;

  /***************************************************************************//**
  * graspPlaneSample Constructor
  ******************************************************************************/
  graspPlaneSample()
  : grasp_plane_ncloud(new pcl::PointCloud<pcl::PointNormal>),
    plane(new pcl::ModelCoefficients)
  {
    fingerCloudSample point_1;
    sample_side_1 = std::make_shared<fingerCloudSample>(point_1);
    fingerCloudSample point_2;
    sample_side_2 = std::make_shared<fingerCloudSample>(point_2);
  }
};

/*! \brief General Struct for Finger gripper grasp planning  */
class FingerGripper : public EndEffector
{
public:
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

  void generateGripperAttributes();

  bool getInitialSamplePoints(const std::shared_ptr<GraspObject> & object);

  void getInitialSampleCloud(const std::shared_ptr<GraspObject> & object);

  bool getGraspCloud(const std::shared_ptr<GraspObject> & object);

  void getCenterCuttingPlane(const std::shared_ptr<GraspObject> & object);

  void getCuttingPlanes(const std::shared_ptr<GraspObject> & object);

  void voxelizeSampleCloud();

  void getBestGrasps(
    const std::shared_ptr<GraspObject> object,
    emd_msgs::msg::GraspMethod * grasp_method,
    const std::shared_ptr<CollisionObject> world_collision_object);

  void getMaxMinValues(const std::shared_ptr<GraspObject> & object);

  void updateMaxMinAttributes(
    std::shared_ptr<fingerCloudSample> & sample,
    const float & centroid_distance,
    const float & grasp_plane_distance,
    const float & curvature);

  void getGraspPose(
    std::shared_ptr<multiFingerGripper> gripper,
    const std::shared_ptr<GraspObject> & object);

  void planGrasps(
    std::shared_ptr<GraspObject> object,
    emd_msgs::msg::GraspMethod * grasp_method,
    std::shared_ptr<CollisionObject> world_collision_object,
    std::string camera_frame);

  void resetVariables();

  void addCuttingPlanesEqualAligned(
    const Eigen::Vector4f & centerpoint,
    const Eigen::Vector4f & plane_vector,
    const bool & both_sides_even);

  void addCuttingPlanes(
    const Eigen::Vector4f & centerpoint,
    const Eigen::Vector4f & plane_vector,
    const int & num_itr_1,
    const int & num_itr_2,
    const float & initial_gap_1,
    const float & initial_gap_2);

  int checkPlaneExists(const float & dist);

  void addPlane(
    const float & dist,
    const Eigen::Vector4f & centerpoint,
    const Eigen::Vector4f & plane_vector,
    const bool & inside_1,
    const bool & inside_2);

  void getGripperClusters();

  void getGripperSamplesFromClusters();

  void getFingerSamples(const std::shared_ptr<GraspObject> & object);
  std::vector<std::shared_ptr<multiFingerGripper>> getAllGripperConfigs(
    const std::shared_ptr<GraspObject> & object,
    const std::shared_ptr<CollisionObject> & world_collision_object,
    std::string camera_frame);

  std::shared_ptr<multiFingerGripper> generateGripperOpenConfig(
    const std::shared_ptr<CollisionObject> & world_collision_object,
    const std::shared_ptr<singleFinger> & closed_center_finger_1,
    const std::shared_ptr<singleFinger> & closed_center_finger_2,
    const Eigen::Vector3f & open_center_finger_1,
    const Eigen::Vector3f & open_center_finger_2,
    const Eigen::Vector3f & plane_normal_normalized,
    const Eigen::Vector3f & grasp_direction,
    std::string camera_frame);

  bool checkFingerCollision(
    const Eigen::Vector3f & finger_point,
    const std::shared_ptr<CollisionObject> & world_collision_object);

  void addPlane(float dist, Eigen::Vector4f centerpoint, Eigen::Vector4f plane_vector);

  void visualizeGrasps(
    pcl::visualization::PCLVisualizer::Ptr viewer,
    std::shared_ptr<GraspObject> object);

  Eigen::Vector3f getPerpendicularVectorInPlane(
    Eigen::Vector3f target_vector,
    pcl::ModelCoefficients::Ptr plane);

  void getGripperRank(std::shared_ptr<multiFingerGripper> gripper);

  std::vector<std::shared_ptr<multiFingerGripper>> getAllRanks(
    std::vector<std::shared_ptr<multiFingerGripper>> input_vector,
    emd_msgs::msg::GraspMethod * grasp_method);

  std::shared_ptr<graspPlaneSample> generateGraspSamples(
    Eigen::Vector4f plane_vector,
    Eigen::Vector3f point_on_plane,
    float dist_to_center_plane,
    int plane_index);

  std::vector<Eigen::Vector3f> getOpenFingerCoordinates(
    const Eigen::Vector3f & grasp_direction,
    const Eigen::Vector3f & finger_1,
    const Eigen::Vector3f & finger_2);

  int getNearestPlaneIndex(float target_distance);

  int getNearestPointIndex(
    const pcl::PointNormal & target_point,
    const pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

  Eigen::Vector3f getGripperPlane(
    std::shared_ptr<singleFinger> & finger_sample_1,
    std::shared_ptr<singleFinger> & finger_sample_2,
    const Eigen::Vector3f & grasp_direction,
    const std::shared_ptr<GraspObject> & object);

  std::string getID() {return id;}

  std::vector<double> getPlanarRPY(
    const Eigen::Vector3f & grasp_direction,
    const Eigen::Vector3f & grasp_direction_normal);

  emd_msgs::msg::Option addClosedGraspDistanceOption(
    std::shared_ptr<multiFingerGripper> gripper);

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
  const float worldXAngleThreshold;
  /*! \brief Threshold after which the object is angled to the world Y axis */
  const float worldYAngleThreshold;
  /*! \brief Threshold after which the object is angled to the world Z axis */
  const float worldZAngleThreshold;
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
  std::vector<std::shared_ptr<graspPlaneSample>> grasp_samples;
  /*! \brief Vector containing distance of each cutting plane to the center */
  std::vector<float> cutting_plane_distances;
  /*! \brief Vector containing the indexes of the cutting planes belonging to side 1 */
  std::vector<int> plane_1_index;
  /*! \brief Vector containing the indexes of the cutting planes belonging to side 2 */
  std::vector<int> plane_2_index;
  /*! \brief  */
  std::vector<std::vector<std::shared_ptr<singleFinger>>> gripper_clusters;
  /*! \brief Vector containing grasp samples sorted by ranks */
  std::vector<std::shared_ptr<multiFingerGripper>> sorted_gripper_configs;
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
};

#endif  // GRASP_PLANNER__END_EFFECTORS__FINGER_GRIPPER_HPP_
