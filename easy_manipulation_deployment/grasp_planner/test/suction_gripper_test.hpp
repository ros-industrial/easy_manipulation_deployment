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

#include "grasp_planner/end_effectors/suction_gripper.hpp"
#include "grasp_planner/common/pcl_functions.hpp"
#include "grasp_planner/common/fcl_types.hpp"
#include "grasp_planner/grasp_object.hpp"

class SuctionGripperTest : public ::testing::Test
{
public:
  using CollisionObject = grasp_planner::collision::CollisionObject;
  std::shared_ptr<GraspObject> object;

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

  std::shared_ptr<SuctionGripper> gripper;
  std::shared_ptr<grasp_planner::collision::CollisionObject> collision_object_ptr;

  SuctionGripperTest();
  void ResetVariables();
  void LoadGripperWithWeights();
  void LoadGripperNoWeights();
  void GenerateObjectHorizontal();
  void GenerateObjectVertical();
  void CreateSphereCloud(
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
