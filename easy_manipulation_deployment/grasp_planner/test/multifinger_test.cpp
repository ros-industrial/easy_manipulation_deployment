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
#include "grasp_object.h"
class MultiFingerTest : public testing::Test
{
public:
  std::shared_ptr<GraspObject> object;
  MultiFingerTest()
  {}
  void SetUp(void)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rectangle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    float length = 0.05;
    float breadth = 0.01;
    float height = 0.02;

    for (float length_ = 0.0; length_ < length; length_ += 0.001) {
      for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.001) {
        for (float height_ = 0.0; height_ < height; height_ += 0.001) {
          pcl::PointXYZRGB temp_point(length_, breadth_, height_);
          rectangle_cloud->points.push_back(temp_point);
        }
      }
    }
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*rectangle_cloud, centroid);
    GraspObject object_("camera_frame", rectangle_cloud, centroid);
    object = std::make_shared<GraspObject>(object_);
  }
  void TearDown(void)
  {
  }
}

TEST_F(MultiFingerTest, FingerSidesZero)
{
  std::string id = "test_gripper";
  int num_fingers_side_1 =  ;
  int num_fingers_side_2 =  ;
  float distance_between_fingers_1 =  ;
  float distance_between_fingers_2 =  ;
  float finger_thickness =  ;
  float gripper_stroke =  ;
  float voxel_size =  ;
  float grasp_quality_weight1 =  ;
  float grasp_quality_weight2 =  ;
  float grasp_plane_dist_limit =  ;
  float cloud_normal_radius =  ;
  float worldXAngleThreshold =  ;
  float worldYAngleThreshold =  ;
  float worldZAngleThreshold =  ;

  FingerGripper gripper()
  FingerGripper::FingerGripper(
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
  const float worldZAngleThreshold_)
}

TEST_F(MultiFingerTest, ThicknessMoreThanSpacing)
{
}

TEST_F(MultiFingerTest, FingerSidesNeg)
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