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

#ifndef FINGER_GRIPPER_HPP
#define FINGER_GRIPPER_HPP

// Main PCL files
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>


// For Visualization
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

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

// For Collision Detection
#include <fcl/narrowphase/collision_result.h> //NOLINT
#include <fcl/narrowphase/collision_request.h>  //NOLINT
#include <fcl/narrowphase/contact.h>  //NOLINT
#include <fcl/narrowphase/collision.h>  //NOLINT
#include "fcl/geometry/shape/sphere.h"

// Custom Libraries
#include "pcl_functions.hpp"
#include "fcl_functions.hpp"
#include "end_effector.hpp"


struct singleFinger
{
  pcl::PointNormal finger_point;
  float centroid_dist;
  float grasp_plane_dist;
  float curvature;
  int plane_index;
  float angle_cos;

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
    plane_index(plane_index_) {
    // std::cout << "New finger: " << std::endl;
    // std::cout << " centroid_dist: " << centroid_dist << std::endl;
    // std::cout << " grasp_plane_dist_: " << grasp_plane_dist << std::endl;
    // std::cout << " curvature: " << curvature << std::endl;
  }
  singleFinger() = default;
};

struct multiFingerGripper
{
  std::vector < std::shared_ptr < singleFinger >> closed_fingers_1;
  std::vector < std::shared_ptr < singleFinger >> closed_fingers_2;
  std::vector < int > closed_fingers_1_index;
  std::vector < int > closed_fingers_2_index;
  std::vector < Eigen::Vector3f > open_fingers_1;
  std::vector < Eigen::Vector3f > open_fingers_2;
  float grasp_plane_angle_cos;
  float rank;
  geometry_msgs::msg::PoseStamped pose;
  pcl::PointXYZ gripper_palm_center;
  std::shared_ptr < singleFinger > base_point_1;
  std::shared_ptr < singleFinger > base_point_2;
  bool collides_with_world;

  multiFingerGripper(
    std::shared_ptr < singleFinger > base_point_1_,
    std::shared_ptr < singleFinger > base_point_2_)
    : base_point_1(base_point_1_),
    base_point_2(base_point_2_) {
    gripper_palm_center.x = (base_point_1_->finger_point.x + base_point_2_->finger_point.x) / 2;
    gripper_palm_center.y = (base_point_1_->finger_point.y + base_point_2_->finger_point.y) / 2;
    gripper_palm_center.z = (base_point_1_->finger_point.z + base_point_2_->finger_point.z) / 2;
  }
};


struct fingerCloudSample
{
  pcl::PointCloud < pcl::PointXYZRGB > ::Ptr finger_cloud;
  pcl::PointCloud < pcl::PointNormal > ::Ptr finger_ncloud;
  pcl::PointCloud < pcl::PointNormal > ::Ptr finger_nvoxel;
  float centroid_dist_min, centroid_dist_max;
  float grasp_plane_dist_min, grasp_plane_dist_max;
  float curvature_min, curvature_max;
  int start_index;
  std::vector < std::shared_ptr < singleFinger >> finger_samples;
  fingerCloudSample() : finger_cloud(new pcl::PointCloud < pcl::PointXYZRGB >),
    finger_ncloud(new pcl::PointCloud < pcl::PointNormal >),
    finger_nvoxel(new pcl::PointCloud < pcl::PointNormal >)
  {
    centroid_dist_min = std::numeric_limits < float > ::max();
    centroid_dist_max = std::numeric_limits < float > ::min();
    grasp_plane_dist_min = curvature_min = centroid_dist_min;
    grasp_plane_dist_max = curvature_max = centroid_dist_max;
  }
};

struct graspPlaneSample
{
  int plane_index;
  bool plane_intersects_object;
  float dist_to_center_plane;
  std::shared_ptr < fingerCloudSample > sample_side_1;
  std::shared_ptr < fingerCloudSample > sample_side_2;
  pcl::PointCloud < pcl::PointNormal > ::Ptr grasp_plane_ncloud;
  pcl::ModelCoefficients::Ptr plane;
  Eigen::Vector4f plane_eigen;
  graspPlaneSample() : grasp_plane_ncloud(new pcl::PointCloud < pcl::PointNormal >),
    plane(new pcl::ModelCoefficients)
  {
    fingerCloudSample point_1;
    sample_side_1 = std::make_shared < fingerCloudSample > (point_1);
    fingerCloudSample point_2;
    sample_side_2 = std::make_shared < fingerCloudSample > (point_2);
  }
};

class FingerGripper: public EndEffector
{
public:
  // pcl::PointCloud<pcl::PointNormal>::Ptr grasp_cloud;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr grasp_1_cloud;
  // pcl::PointCloud<pcl::PointNormal>::Ptr grasp_1_ncloud;
  // pcl::PointCloud<pcl::PointNormal>::Ptr grasp_1_nvoxel;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr grasp_2_cloud;
  // pcl::PointCloud<pcl::PointNormal>::Ptr grasp_2_ncloud;
  // pcl::PointCloud<pcl::PointNormal>::Ptr grasp_2_nvoxel;

  std::string id;
  int num_fingers_total;
  const int num_fingers_side_1;
  const int num_fingers_side_2;

  const float distance_between_fingers_1;
  const float distance_between_fingers_2;

  const float finger_thickness;
  const float gripper_stroke;
  const float voxel_size;
  const float grasp_quality_weight1;
  const float grasp_quality_weight2;

  const float grasp_plane_dist_limit;
  const float cloud_normal_radius;
  const float worldXAngleThreshold;
  const float worldYAngleThreshold;
  const float worldZAngleThreshold;

  // Object Specific
  Eigen::Vector4f center_cutting_plane;  // grasp Plane vector coeff: a, b, c ,d
  Eigen::Vector3f center_cutting_plane_normal;

  // pcl::PointNormal sample_point_1;

  // float centroid_dist_1_min, centroid_dist_1_max;
  // float grasp_plane_dist_1_min, grasp_plane_dist_1_max;
  // float curvature_1_min, curvature_1_max;

  // pcl::PointNormal sample_point_2;


  // float centroid_dist_2_min, centroid_dist_2_max;
  // float grasp_plane_dist_2_min, grasp_plane_dist_2_max;
  // float curvature_2_min, curvature_2_max;


  std::vector < float > centroid_dist_min_vec;
  std::vector < float > centroid_dist_max_vec;
  std::vector < float > grasp_plane_dist_min_vec;
  std::vector < float > grasp_plane_dist_max_vec;
  std::vector < float > curvature_min_vec;
  std::vector < float > curvature_max_vec;

  // std::vector<graspPlaneSample> grasp_samples;
  // std::vector<multiFingerGripper> gripper_samples;
  std::vector < std::shared_ptr < graspPlaneSample >> grasp_samples;
  std::vector < std::shared_ptr < multiFingerGripper >> gripper_samples;


  std::vector < float > cutting_plane_distances;
  std::vector < int > plane_1_index;
  std::vector < int > plane_2_index;
  std::vector < std::vector < std::shared_ptr < singleFinger >> > gripper_clusters;
  std::vector < std::vector < std::shared_ptr < singleFinger >> > gripper_clusters_flipped;

  std::vector < std::shared_ptr < multiFingerGripper >> sorted_gripper_configs;


  FingerGripper(
    std::string id_,
    const int num_fingers_side_1_,
    const int num_fingers_side_2_,
    const float distance_between_fingers_1_,
    const float distance_between_fingers_2_,
    const float finger_thickness_,
    const float gripper_stroke_,
    const float voxel_size_,
    const float grasp_quality_weight1_,
    const float grasp_quality_weight2_,
    const float grasp_plane_dist_limit_,
    const float cloud_normal_radius_,
    const float worldXAngleThreshold_,
    const float worldYAngleThreshold_,
    const float worldZAngleThreshold_);
  bool getInitialSamplePoints(const std::shared_ptr < GraspObject > object);
  void getInitialSampleCloud(const std::shared_ptr < GraspObject > object);
  void getGraspCloud(const std::shared_ptr < GraspObject > object);
  void getCuttingPlanes(const std::shared_ptr < GraspObject > object);
  void voxelizeSampleCloud();
  void getBestGrasps(
    const std::shared_ptr < GraspObject > object,
    emd_msgs::msg::GraspMethod * grasp_method,
    const std::shared_ptr < fcl::CollisionObject < float >> world_collision_object);
  void getMaxMinValues(std::shared_ptr < GraspObject > object);
  void getGraspPose(
    std::shared_ptr < multiFingerGripper > gripper,
    const std::shared_ptr < GraspObject > object);
  void planGrasps(
    std::shared_ptr < GraspObject > object,
    emd_msgs::msg::GraspMethod * grasp_method,
    std::shared_ptr < fcl::CollisionObject < float >> world_collision_object,
    pcl::visualization::PCLVisualizer::Ptr viewer);

  void resetVariables();

  void addCuttingPlanesEqualAligned(
    const Eigen::Vector4f centerpoint,
    const Eigen::Vector4f plane_vector, const bool both_sides_even);
  void addCuttingPlanes(
    const Eigen::Vector4f & centerpoint, const Eigen::Vector4f & plane_vector,
    int num_itr_1, int num_itr_2, float initial_gap_1, float initial_gap_2);

  int checkPlaneExists(float dist);
  void addPlane(
    float dist, Eigen::Vector4f centerpoint, Eigen::Vector4f plane_vector,
    bool inside_1, bool inside_2);

  void getGripperClusters();
  void getGripperSamplesFromClusters();
  void getFingerSamples(std::shared_ptr < GraspObject > object);
  std::vector < std::shared_ptr < multiFingerGripper >> getAllGripperConfigs(
    std::shared_ptr < GraspObject > object,
    std::shared_ptr < fcl::CollisionObject < float >> world_collision_object);
  std::shared_ptr < multiFingerGripper > generateGripperOpenConfig(
    std::shared_ptr < fcl::CollisionObject < float >> world_collision_object,
    std::shared_ptr < singleFinger > closed_center_finger_1,
    std::shared_ptr < singleFinger > closed_center_finger_2, Eigen::Vector3f open_center_finger_1,
    Eigen::Vector3f open_center_finger_2, Eigen::Vector3f plane_normal_normalized,
    Eigen::Vector3f grasp_direction);
  bool checkFingerCollision(
    Eigen::Vector3f finger_point,
    std::shared_ptr < fcl::CollisionObject < float >> world_collision_object);

  void addPlane(float dist, Eigen::Vector4f centerpoint, Eigen::Vector4f plane_vector);

  void visualizeGrasps(pcl::visualization::PCLVisualizer::Ptr viewer);
  Eigen::Vector3f getPerpendicularVectorInPlane(
    Eigen::Vector3f target_vector,
    pcl::ModelCoefficients::Ptr plane);
  void getGripperRank(std::shared_ptr < multiFingerGripper > gripper);
  std::vector < std::shared_ptr < multiFingerGripper >> getAllRanks(
    std::vector < std::shared_ptr < multiFingerGripper >> input_vector,
    emd_msgs::msg::GraspMethod * grasp_method);
  std::string getID() {return id;}
};

#endif  // FINGER_GRIPPER_HPP_
