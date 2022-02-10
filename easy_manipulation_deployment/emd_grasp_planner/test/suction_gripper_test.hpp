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


#ifndef SUCTION_GRIPPER_TEST_HPP_
#define SUCTION_GRIPPER_TEST_HPP_

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

#include "emd/grasp_planner/end_effectors/suction_gripper.hpp"
#include "emd/grasp_planner/grasp_object.hpp"
#include "emd/common/pcl_functions.hpp"
#include "emd/common/fcl_types.hpp"


class SuctionGripperTestFixture : public SuctionGripper
{

public:
  explicit SuctionGripperTestFixture(
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
    std::string grasp_approach_direction_
  )
  : SuctionGripper(
      id_,
      num_cups_length_,
      num_cups_breadth_,
      dist_between_cups_length_,
      dist_between_cups_breadth_,
      cup_radius_,
      cup_height_,
      num_sample_along_axis_,
      search_resolution_,
      search_angle_resolution_,
      cloud_normal_radius_,
      curvature_weight_,
      grasp_center_distance_weight_,
      num_contact_points_weight_,
      length_direction_,
      breadth_direction_,
      grasp_approach_direction_
  ) {}

  int num_sample_along_axis_public;

  float search_resolution_public;

  int search_angle_resolution_public;

  float cloud_normal_radius_public;

  float curvature_weight_public;

  float grasp_center_distance_weight_public;

  float num_contact_points_weight_public;

  char length_direction_public;

  char breadth_direction_public;

  char grasp_approach_direction_public;

  float breadth_dim_public;

  float length_dim_public;

  bool col_is_even_public;

  float col_itr_public;

  float col_dist_between_cups_public;

  float col_initial_gap_public;

  char col_direction_public;

  bool row_is_even_public;

  float row_itr_public;

  float row_dist_between_cups_public;

  float row_initial_gap_public;

  char row_direction_public;

  float max_curvature_public;

  float min_curvature_public;

  int max_contact_points_public;

  int min_contact_points_public;

  float max_center_dist_public;

  float min_center_dist_public;

  std::vector<std::shared_ptr<SuctionCupArray>> cup_array_samples_public;

  int get_centroid_index_public(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
  {
    return get_centroid_index(cloud);
  }

  void get_all_possible_grasps_public(
    const GraspObject & object,
    const pcl::PointXYZ & object_center,
    const pcl::PointXYZRGB & top_point,
    std::string camera_frame)
  {
    return get_all_possible_grasps(
      object,
      object_center,
      top_point,
      camera_frame);
  }

  bool get_cup_contact_cloud_public(
    pcl::PointXYZRGB contact_point,
    float radius, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output)
  {
    return get_cup_contact_cloud(
      contact_point,
      radius,
      cloud_input,
      cloud_output
    );
  }

  std::vector<int> get_cup_contact_indices_public(
    pcl::PointXYZRGB contact_point,
    float radius, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input)
  {
    return get_cup_contact_indices(
      contact_point,
      radius,
      cloud_input
    );
  }

  pcl::PointXYZRGB find_highest_point_public(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
    const char & height_axis,
    const bool & is_positive)
  {
    return find_highest_point(
      cloud,
      height_axis,
      is_positive
    );
  }

  void get_starting_plane_public(
    pcl::ModelCoefficients::Ptr plane_coefficients,
    const Eigen::Vector3f & axis,
    const Eigen::Vector4f & object_centerpoint,
    const pcl::PointXYZRGB & top_point,
    const char & height_axis)
  {
    return get_starting_plane(
      plane_coefficients,
      axis,
      object_centerpoint,
      top_point,
      height_axis
    );
  }

  SingleSuctionCup generate_suction_cup_public(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & projected_cloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & sliced_cloud_normal,
    const Eigen::Vector3f & suction_cup_center,
    const pcl::PointXYZ & object_center,
    const float & object_max_dim)
  {
    return generate_suction_cup(
      projected_cloud,
      sliced_cloud_normal,
      suction_cup_center,
      object_center,
      object_max_dim
    );
  }

  void get_sliced_cloud_public(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & input_cloud_normal,
    const float & top_limit,
    const float & bottom_limit,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal,
    const char & height_axis)
  {
    return get_sliced_cloud(
      input_cloud,
      input_cloud_normal,
      top_limit,
      bottom_limit,
      sliced_cloud,
      sliced_cloud_normal,
      height_axis
    );
  }

  void project_cloud_to_plane_public(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
    const pcl::ModelCoefficients::Ptr & plane_coefficients,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud)
  {
    return project_cloud_to_plane(
      input_cloud,
      plane_coefficients,
      projected_cloud);
  }

  std::vector<int> create_disk_from_cloud_public(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal,
    pcl::PointXYZ centerpoint,
    float radius,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr disk_cloud,
    float & curvature_sum)
  {
    return create_disk_from_cloud(
      input_cloud,
      sliced_cloud_normal,
      centerpoint,
      radius,
      disk_cloud,
      curvature_sum
    );
  }

  pcl::PointXYZ get_gripper_center_public(
    const Eigen::Vector3f & object_axis,
    const float & offset,
    const pcl::PointXYZRGB & slice_centroid)
  {
    return get_gripper_center(
      object_axis,
      offset,
      slice_centroid
    );
  }

  int get_contact_points_public(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & sliced_cloud_normal,
    const pcl::PointXYZ & centerpoint,
    float & curvature_sum)
  {
    return get_contact_points(
      input_cloud,
      sliced_cloud_normal,
      centerpoint,
      curvature_sum
    );
  }

  void update_max_min_values_public(
    const int & num_contact_points,
    const float & average_curvature,
    const float & center_dist)
  {
    return update_max_min_values(
      num_contact_points,
      average_curvature,
      center_dist
    );
  }

  void get_all_grasp_ranks_public(
    emd_msgs::msg::GraspMethod & grasp_method,
    const GraspObject & object)
  {
    return get_all_grasp_ranks(
      grasp_method,
      object
    );
  }

  std::vector<double> get_planar_rpy_public(
    const Eigen::Vector3f & col_vector,
    const Eigen::Vector3f & row_vector)
  {
    return get_planar_rpy(
      col_vector,
      row_vector
    );
  }

  geometry_msgs::msg::PoseStamped get_grasp_pose_public(
    const std::shared_ptr<SuctionCupArray> & grasp,
    const GraspObject & object)
  {
    return get_grasp_pose(
      grasp,
      object
    );
  }

  SuctionCupArray generate_grasp_sample_public(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & projected_cloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & sliced_cloud_normal,
    const pcl::PointXYZ & sample_gripper_center,
    const pcl::PointXYZ & object_center,
    const Eigen::Vector3f & grasp_direction,
    const Eigen::Vector3f & object_direction,
    const float & object_max_dim,
    std::string camera_frame)
  {
    return generate_grasp_sample(
      projected_cloud,
      sliced_cloud_normal,
      sample_gripper_center,
      object_center,
      grasp_direction,
      object_direction,
      object_max_dim,
      camera_frame
    );
  }

  int generate_weighted_contact_points_public(
    const int & contact_points,
    const float & cup_to_center_dist,
    const float & object_max_dim)
  {
    return generate_weighted_contact_points(
      contact_points,
      cup_to_center_dist,
      object_max_dim
    );
  }

  float get_gap_public(
    const int & itr,
    const bool & is_even,
    const float & initial_gap,
    const float & dist_between_cups)
  {
    return get_gap(
      itr,
      is_even,
      initial_gap,
      dist_between_cups
    );
  }

  void update_gripper_attributes()
  {

    num_sample_along_axis_public = num_sample_along_axis;

    search_resolution_public = search_resolution;

    search_angle_resolution_public = search_angle_resolution;

    cloud_normal_radius_public = cloud_normal_radius;

    curvature_weight_public = curvature_weight;

    grasp_center_distance_weight_public = grasp_center_distance_weight;

    num_contact_points_weight_public = num_contact_points_weight;

    length_direction_public = length_direction;

    breadth_direction_public = breadth_direction;

    grasp_approach_direction_public = grasp_approach_direction;

    breadth_dim_public = breadth_dim;

    length_dim_public = length_dim;

    col_is_even_public = col_is_even;

    col_itr_public = col_itr;

    col_dist_between_cups_public = col_dist_between_cups;

    col_initial_gap_public = col_initial_gap;

    col_direction_public = col_direction;

    row_is_even_public = row_is_even;

    row_itr_public = row_itr;

    row_dist_between_cups_public = row_dist_between_cups;

    row_initial_gap_public = row_initial_gap;

    row_direction_public = row_direction;

    max_curvature_public = max_curvature;

    min_curvature_public = min_curvature;

    max_contact_points_public = max_contact_points;

    min_contact_points_public = min_contact_points;

    max_center_dist_public = max_center_dist;

    min_center_dist_public = min_center_dist;

    cup_array_samples_public = cup_array_samples;
  }
};


class SuctionGripperTest : public ::testing::Test
{
public:
  using CollisionObject = grasp_planner::collision::CollisionObject;
  // GraspObject object;

  std::string id;
  int num_cups_length;
  int num_cups_breadth;
  float dist_between_cups_length;
  float dist_between_cups_breadth;
  float cup_radius;
  float cup_height;
  int num_sample_along_axis;
  float search_resolution;
  int search_angle_resolution;
  float cloud_normal_radius;
  float curvature_weight;
  float grasp_center_distance_weight;
  float num_contact_points_weight;
  std::string length_direction;
  std::string breadth_direction;
  std::string grasp_approach_direction;
  std::string camera_frame;

  std::shared_ptr<SuctionGripperTestFixture> gripper;
  std::shared_ptr<grasp_planner::collision::CollisionObject> collision_object_ptr;

  SuctionGripperTest();
  void reset_variables();
  void LoadGripperWithWeights();
  void LoadGripperNoWeights();
  GraspObject GenerateObjectHorizontal();
  GraspObject GenerateObjectVertical();
  GraspObject CreateSphereCloud(
    Eigen::Vector3f & centerpoint,
    const float & radius, const int & resolution,
    const float & x_scale, const float & y_scale, const float & z_scale);
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

#endif  // SUCTION_GRIPPER_TEST_HPP_
