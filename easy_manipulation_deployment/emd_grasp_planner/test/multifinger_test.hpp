// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//`
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef MULTIFINGER_TEST_HPP_
#define MULTIFINGER_TEST_HPP_

#include <gtest/gtest.h>

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

#include "emd/grasp_planner/end_effectors/finger_gripper.hpp"
#include "emd/common/pcl_functions.hpp"
#include "emd/common/fcl_types.hpp"
#include "emd/grasp_planner/grasp_object.hpp"

class FingerGripperTestFixture : public FingerGripper
{

public:
  explicit FingerGripperTestFixture(
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
    std::string grasp_approach_direction_

  )
  : FingerGripper(
      id_,
      num_fingers_side_1_,
      num_fingers_side_2_,
      distance_between_fingers_1_,
      distance_between_fingers_2_,
      finger_thickness_,
      gripper_stroke_,
      voxel_size_,
      grasp_quality_weight1_,
      grasp_quality_weight2_,
      grasp_plane_dist_limit_,
      cloud_normal_radius_,
      worldXAngleThreshold_,
      worldYAngleThreshold_,
      worldZAngleThreshold_,
      grasp_stroke_direction_,
      grasp_stroke_normal_direction_,
      grasp_approach_direction_
  ) {}

  Eigen::Vector4f center_cutting_plane_public;

  Eigen::Vector3f center_cutting_plane_normal_public;

  std::vector<std::shared_ptr<GraspPlaneSample>> grasp_samples_public;

  std::vector<float> cutting_plane_distances_public;

  std::vector<int> plane_1_index_public;

  std::vector<int> plane_2_index_public;

  std::vector<std::vector<std::shared_ptr<SingleFinger>>> gripper_clusters_public;

  bool is_even_1_public;

  bool is_even_2_public;

  float initial_gap_1_public;

  float initial_gap_2_public;

  int num_itr_1_public;

  int num_itr_2_public;

  bool get_initial_sample_points_public(const GraspObject & object)
  {
    return get_initial_sample_points(object);
  }
  Eigen::Vector3f get_perpendicular_vector_in_plane_public(
    Eigen::Vector3f target_vector,
    pcl::ModelCoefficients::Ptr plane)
  {
    return get_perpendicular_vector_in_plane(target_vector, plane);
  }

  std::vector<double> get_planar_rpy_public(
    const Eigen::Vector3f & grasp_direction,
    const Eigen::Vector3f & grasp_direction_normal)
  {
    return get_planar_rpy(grasp_direction, grasp_direction_normal);
  }

  std::vector<Eigen::Vector3f> get_open_finger_coordinates_public(
    const Eigen::Vector3f & grasp_direction,
    const Eigen::Vector3f & finger_1,
    const Eigen::Vector3f & finger_2)
  {
    return get_open_finger_coordinates(
      grasp_direction,
      finger_1,
      finger_2
    );
  }

  Eigen::Vector3f get_gripper_plane_public(
    std::shared_ptr<SingleFinger> & finger_sample_1,
    std::shared_ptr<SingleFinger> & finger_sample_2,
    const Eigen::Vector3f & grasp_direction,
    const GraspObject & object)
  {
    return get_gripper_plane(
      finger_sample_1,
      finger_sample_2,
      grasp_direction,
      object
    );
  }

  void add_cutting_planes_equal_aligned_public(
    const Eigen::Vector4f & centerpoint,
    const Eigen::Vector4f & plane_vector,
    const bool & both_sides_even)
  {
    add_cutting_planes_equal_aligned(
      centerpoint,
      plane_vector,
      both_sides_even
    );
  }

  void add_cutting_planes_public(
    const Eigen::Vector4f & centerpoint,
    const Eigen::Vector4f & plane_vector,
    const int & num_itr_1,
    const int & num_itr_2,
    const float & initial_gap_1,
    const float & initial_gap_2)
  {
    add_cutting_planes(
      centerpoint,
      plane_vector,
      num_itr_1,
      num_itr_2,
      initial_gap_1,
      initial_gap_2
    );
  }

  int check_plane_exists_public(const float & dist)
  {
    return check_plane_exists(dist);
  }

  void add_plane_public(
    const float & dist,
    const Eigen::Vector4f & centerpoint,
    const Eigen::Vector4f & plane_vector,
    const bool & inside_1,
    const bool & inside_2)
  {
    add_plane(
      dist,
      centerpoint,
      plane_vector,
      inside_1,
      inside_2
    );
  }

  void add_plane_public(
    float dist,
    Eigen::Vector4f centerpoint,
    Eigen::Vector4f plane_vector)
  {
    add_plane(
      dist,
      centerpoint,
      plane_vector
    );
  }

  int get_nearest_plane_index_public(float target_distance)
  {
    return get_nearest_plane_index(target_distance);
  }

  int get_nearest_point_index_public(
    const pcl::PointNormal & target_point,
    const pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
  {
    return get_nearest_point_index(target_point, cloud);
  }

  std::shared_ptr<MultiFingerGripper> generate_gripper_open_config_public(
    const std::shared_ptr<CollisionObject> & world_collision_object,
    const std::shared_ptr<SingleFinger> & closed_center_finger_1,
    const std::shared_ptr<SingleFinger> & closed_center_finger_2,
    const Eigen::Vector3f & open_center_finger_1,
    const Eigen::Vector3f & open_center_finger_2,
    const Eigen::Vector3f & plane_normal_normalized,
    const Eigen::Vector3f & grasp_direction,
    std::string camera_frame)
  {
    return generate_gripper_open_config(
      world_collision_object,
      closed_center_finger_1,
      closed_center_finger_2,
      open_center_finger_1,
      open_center_finger_2,
      plane_normal_normalized,
      grasp_direction,
      camera_frame
    );
  }

  bool check_finger_collision_public(
    const Eigen::Vector3f & finger_point,
    const std::shared_ptr<CollisionObject> & world_collision_object)
  {
    return check_finger_collision(
      finger_point,
      world_collision_object
    );
  }

  std::shared_ptr<GraspPlaneSample> generate_grasp_samples_public(
    Eigen::Vector4f plane_vector,
    Eigen::Vector3f point_on_plane,
    float dist_to_center_plane,
    int plane_index)
  {
    return generate_grasp_samples(
      plane_vector,
      point_on_plane,
      dist_to_center_plane,
      plane_index
    );
  }

  void get_initial_sample_cloud_public(const GraspObject & object)
  {
    get_initial_sample_cloud(object);
  }

  bool get_grasp_cloud_public(const GraspObject & object)
  {
    return get_grasp_cloud(object);
  }

  void get_center_cutting_plane_public(const GraspObject & object)
  {
    get_center_cutting_plane(object);
  }

  void get_cutting_planes_public(const GraspObject & object)
  {
    get_cutting_planes(object);
  }

  void voxelize_sample_cloud_public()
  {
    voxelize_sample_cloud();
  }

  void get_gripper_rank_public(std::shared_ptr<MultiFingerGripper> gripper)
  {
    get_gripper_rank(gripper);
  }

  std::vector<std::shared_ptr<MultiFingerGripper>> get_all_ranks_public(
    std::vector<std::shared_ptr<MultiFingerGripper>> input_vector,
    emd_msgs::msg::GraspMethod & grasp_method)
  {
    return get_all_ranks(
      input_vector,
      grasp_method
    );
  }


  void get_gripper_clusters_public()
  {
    get_gripper_clusters();
  }

  void get_gripper_samples_from_clusters_public()
  {
    get_gripper_samples_from_clusters();
  }

  void get_finger_samples_public(const GraspObject & object)
  {
    get_finger_samples(object);
  }

  std::vector<std::shared_ptr<MultiFingerGripper>> get_all_gripper_configs_public(
    const GraspObject & object,
    const std::shared_ptr<CollisionObject> & world_collision_object,
    std::string camera_frame)
  {
    return get_all_gripper_configs(
      object,
      world_collision_object,
      camera_frame
    );
  }

  void get_max_min_values_public(const GraspObject & object)
  {
    get_max_min_values(object);
  }

  void update_max_min_attributes_public(
    std::shared_ptr<FingerCloudSample> & sample,
    const float & centroid_distance,
    const float & grasp_plane_distance,
    const float & curvature)
  {
    update_max_min_attributes(
      sample,
      centroid_distance,
      grasp_plane_distance,
      curvature
    );
  }

  void get_grasp_pose_public(
    std::shared_ptr<MultiFingerGripper> gripper,
    const GraspObject & object)
  {
    get_grasp_pose(
      gripper,
      object
    );
  }

  emd_msgs::msg::Option add_closed_grasp_distance_option_public(
    std::shared_ptr<MultiFingerGripper> gripper)
  {
    return add_closed_grasp_distance_option(gripper);
  }

  void update_gripper_attributes()
  {

    center_cutting_plane_public = center_cutting_plane;

    center_cutting_plane_normal_public = center_cutting_plane_normal;

    grasp_samples_public = grasp_samples;

    cutting_plane_distances_public = cutting_plane_distances;

    plane_1_index_public = plane_1_index;

    plane_2_index_public = plane_2_index;

    gripper_clusters_public = gripper_clusters;

    is_even_1_public = is_even_1;

    is_even_2_public = is_even_2;

    initial_gap_1_public = initial_gap_1;

    initial_gap_2_public = initial_gap_2;

    num_itr_1_public = num_itr_1;

    num_itr_2_public = num_itr_2;

  }
};

class MultiFingerTest : public ::testing::Test
{
public:
  using CollisionObject = grasp_planner::collision::CollisionObject;
  // std::shared_ptr<GraspObject>  object;
  std::string id;
  int num_fingers_side_1;
  int num_fingers_side_2;
  float distance_between_fingers_1;
  float distance_between_fingers_2;
  float finger_thickness;
  float gripper_stroke;
  float voxel_size;
  float grasp_quality_weight1;
  float grasp_quality_weight2;
  float grasp_plane_dist_limit;
  float cloud_normal_radius;
  float worldXAngleThreshold;
  float worldYAngleThreshold;
  float worldZAngleThreshold;
  std::string grasp_stroke_direction;
  std::string grasp_stroke_normal_direction;
  std::string grasp_approach_direction;
  std::string camera_frame;

  std::shared_ptr<FingerGripperTestFixture> gripper;
  std::shared_ptr<grasp_planner::collision::CollisionObject> collision_object_ptr;

  // pcl::visualization::PCLVisualizer::Ptr viewer;

  MultiFingerTest();
  void reset_variables();
  void LoadGripper();
  GraspObject GenerateObjectHorizontal();
  GraspObject GenerateObjectVertical();
  void GenerateObjectCollision(float length, float breadth, float height);


  void SetUp(void)
  {
    std::cout << "Setup" << std::endl;
  }
  void TearDown(void)
  {
    std::cout << "Teardown" << std::endl;
  }
};

#endif  // MULTIFINGER_TEST_HPP_
