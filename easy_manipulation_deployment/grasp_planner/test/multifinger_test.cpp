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

#include <gtest/gtest.h>
#include "multifinger_test.hpp"

TEST_F(MultiFingerTest, FingerSidesZero)
{
  std::string id = "test_gripper";
  int num_fingers_side_1 = 0 ;
  int num_fingers_side_2 = 0 ;
  float distance_between_fingers_1 = 0.02 ;
  float distance_between_fingers_2 = 0.02 ;
  float finger_thickness = 0.02 ;
  float gripper_stroke = 0.085 ;
  float voxel_size = 0.01 ;
  float grasp_quality_weight1 = 1.5 ;
  float grasp_quality_weight2 = 1.0 ;
  float grasp_plane_dist_limit = 0.007 ;
  float cloud_normal_radius = 0.03 ;
  float worldXAngleThreshold = 0.5 ;
  float worldYAngleThreshold = 0.5 ;
  float worldZAngleThreshold = 0.25 ;

  
    ASSERT_THROW(FingerGripper gripper(
      id,
      num_fingers_side_1,
      num_fingers_side_2,
      distance_between_fingers_1,
      distance_between_fingers_2,
      finger_thickness,
      gripper_stroke,
      voxel_size,
      grasp_quality_weight1,
      grasp_quality_weight2,
      grasp_plane_dist_limit,
      cloud_normal_radius,
      worldXAngleThreshold,
      worldYAngleThreshold,
      worldZAngleThreshold), std::invalid_argument);

    num_fingers_side_1 = 1 ;
    num_fingers_side_2 = 0;
    ASSERT_THROW(FingerGripper gripper(
      id,
      num_fingers_side_1,
      num_fingers_side_2,
      distance_between_fingers_1,
      distance_between_fingers_2,
      finger_thickness,
      gripper_stroke,
      voxel_size,
      grasp_quality_weight1,
      grasp_quality_weight2,
      grasp_plane_dist_limit,
      cloud_normal_radius,
      worldXAngleThreshold,
      worldYAngleThreshold,
      worldZAngleThreshold), std::invalid_argument);

    num_fingers_side_1 = 0;
    num_fingers_side_2 = 1;

    ASSERT_THROW(FingerGripper gripper(
      id,
      num_fingers_side_1,
      num_fingers_side_2,
      distance_between_fingers_1,
      distance_between_fingers_2,
      finger_thickness,
      gripper_stroke,
      voxel_size,
      grasp_quality_weight1,
      grasp_quality_weight2,
      grasp_plane_dist_limit,
      cloud_normal_radius,
      worldXAngleThreshold,
      worldYAngleThreshold,
      worldZAngleThreshold), std::invalid_argument);

    num_fingers_side_1 = 1;
    num_fingers_side_2 = 1;
    ASSERT_THROW(FingerGripper gripper(
      id,
      num_fingers_side_1,
      num_fingers_side_2,
      distance_between_fingers_1,
      distance_between_fingers_2,
      finger_thickness,
      gripper_stroke,
      voxel_size,
      grasp_quality_weight1,
      grasp_quality_weight2,
      grasp_plane_dist_limit,
      cloud_normal_radius,
      worldXAngleThreshold,
      worldYAngleThreshold,
      worldZAngleThreshold), std::invalid_argument);

}

TEST_F(MultiFingerTest, ThicknessMoreThanSpacing)
{
}

TEST_F(MultiFingerTest, FingerSidesNeg)
{
}

TEST_F(MultiFingerTest, FingerThicknessZero)
{
}
TEST_F(MultiFingerTest, FingerThicknessNeg)
{
}

TEST_F(MultiFingerTest, FingerStrokeZero)
{
}
TEST_F(MultiFingerTest, FingerStrokeNeg)
{
}

TEST_F(MultiFingerTest, AddPlaneTestInside1)
{
}

TEST_F(MultiFingerTest, AddPlaneTestInside2)
{
}

TEST_F(MultiFingerTest, AddPlaneTestInsideBoth)
{
}


TEST_F(MultiFingerTest, CheckPlaneExistsTest)
{
}


TEST_F(MultiFingerTest, GetCenterCuttingPlaneCheck)
{
}


TEST_F(MultiFingerTest, GetCuttingPlaneBothOdd)
{
}

TEST_F(MultiFingerTest, GetCuttingPlaneBothEven)
{
}

TEST_F(MultiFingerTest, GetCuttingPlaneOddEven)
{
}