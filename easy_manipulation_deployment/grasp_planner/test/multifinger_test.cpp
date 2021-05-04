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

MultiFingerTest::MultiFingerTest()
//: viewer(new pcl::visualization::PCLVisualizer("Cloud viewer"))
{
  ResetVariables();
}
void MultiFingerTest::ResetVariables()
{
  id = "test_gripper";
  num_fingers_side_1 = 2;
  num_fingers_side_2 = 2;
  distance_between_fingers_1 = 0.02;
  distance_between_fingers_2 = 0.02;
  finger_thickness = 0.01;
  gripper_stroke = 0.085;
  voxel_size = 0.01;
  grasp_quality_weight1 = 1.5;
  grasp_quality_weight2 = 1.0;
  grasp_plane_dist_limit = 0.007;
  cloud_normal_radius = 0.03;
  worldXAngleThreshold = 0.5;
  worldYAngleThreshold = 0.5;
  worldZAngleThreshold = 0.25;
}

void MultiFingerTest::LoadGripper()
{
  FingerGripper gripper_(
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
    worldZAngleThreshold);
  gripper_.generateGripperAttributes();
    gripper = std::make_shared<FingerGripper>(gripper_);
}

void MultiFingerTest::GenerateObjectHorizontal()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rectangle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  float length = 0.05;
  float breadth = 0.01;
  float height = 0.02;

  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.0; height_ < height; height_ += 0.0025) {
        pcl::PointXYZRGB temp_point(length_, breadth_, height_);
        rectangle_cloud->points.push_back(temp_point);
      }
    }
  }
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*rectangle_cloud, centroid);
  GraspObject object_("camera_frame", rectangle_cloud, centroid);
  object = std::make_shared<GraspObject>(object_);
  PCLFunctions::computeCloudNormal(object->cloud, object->cloud_normal, 0.03);
  object->get_object_bb();
  object->get_object_world_angles();
}

void MultiFingerTest::GenerateObjectVertical()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rectangle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  float length = 0.01;
  float breadth = 0.05;
  float height = 0.02;

  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.0; height_ < height; height_ += 0.0025) {
        pcl::PointXYZRGB temp_point(length_, breadth_, height_);
        rectangle_cloud->points.push_back(temp_point);
      }
    }
  }
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*rectangle_cloud, centroid);
  GraspObject object_("camera_frame", rectangle_cloud, centroid);
  object = std::make_shared<GraspObject>(object_);
  PCLFunctions::computeCloudNormal(object->cloud, object->cloud_normal, 0.03);
  object->get_object_bb();
  object->get_object_world_angles();
}

void MultiFingerTest::GenerateObjectCollision(float length, float breadth, float height)
{
  grasp_planner::collision::Box * collision_object_shape =
    new grasp_planner::collision::Box(length, breadth, height);

  grasp_planner::collision::Transform collision_object_transform;
  collision_object_transform.setIdentity();

  #if FCL_VERSION_0_6_OR_HIGHER == 1
    collision_object_transform.translation() << 0, 0, 0;
  #else
    collision_object_transform.setTranslation(
      grasp_planner::collision::Vector(0, 0, 0));
  #endif

  grasp_planner::collision::CollisionObject collision_object(
    std::shared_ptr<grasp_planner::collision::CollisionGeometry>(collision_object_shape), fcl::Transform3<float>::Identity());

  collision_object_ptr =
    std::make_shared<grasp_planner::collision::CollisionObject>(collision_object);
}

TEST_F(MultiFingerTest, GenerateGripperAttributesTestEvenOdd)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 2;
  num_fingers_side_2 = 1;
  distance_between_fingers_1 = 0.01;
  distance_between_fingers_2 = 0;
  ASSERT_NO_THROW(LoadGripper());
  EXPECT_TRUE(gripper->is_even_1);
  EXPECT_FALSE(gripper->is_even_2);
  EXPECT_NEAR(0.005, gripper->initial_gap_1, 0.0001);
  EXPECT_NEAR(0, gripper->initial_gap_2, 0.0001);
  EXPECT_EQ(1, gripper->num_itr_1);
  EXPECT_EQ(0, gripper->num_itr_2);
}

TEST_F(MultiFingerTest, GenerateGripperAttributesTestOddEven)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 3;
  num_fingers_side_2 = 6;
  distance_between_fingers_1 = 0.03;
  distance_between_fingers_2 = 0.02;
  ASSERT_NO_THROW(LoadGripper());
  EXPECT_FALSE(gripper->is_even_1);
  EXPECT_TRUE(gripper->is_even_2);
  EXPECT_NEAR(0.03, gripper->initial_gap_1, 0.0001);
  EXPECT_NEAR(0.01, gripper->initial_gap_2, 0.0001);
  EXPECT_EQ(1, gripper->num_itr_1);
  EXPECT_EQ(3, gripper->num_itr_2);
}

TEST_F(MultiFingerTest, GenerateGripperAttributesTestOddOdd)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 3;
  num_fingers_side_2 = 5;
  distance_between_fingers_1 = 0.02;
  distance_between_fingers_2 = 0.06;
  ASSERT_NO_THROW(LoadGripper());
  EXPECT_FALSE(gripper->is_even_1);
  EXPECT_FALSE(gripper->is_even_2);
  EXPECT_NEAR(0.02, gripper->initial_gap_1, 0.0001);
  EXPECT_NEAR(0.06, gripper->initial_gap_2, 0.0001);
  EXPECT_EQ(1, gripper->num_itr_1);
  EXPECT_EQ(2, gripper->num_itr_2);
}

TEST_F(MultiFingerTest, GenerateGripperAttributesTestEvenEven)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 2;
  num_fingers_side_2 = 4;
  distance_between_fingers_1 = 0.06;
  distance_between_fingers_2 = 0.01;
  ASSERT_NO_THROW(LoadGripper());
  EXPECT_TRUE(gripper->is_even_1);
  EXPECT_TRUE(gripper->is_even_2);
  EXPECT_NEAR(0.03, gripper->initial_gap_1, 0.0001);
  EXPECT_NEAR(0.005, gripper->initial_gap_2, 0.0001);
  EXPECT_EQ(1, gripper->num_itr_1);
  EXPECT_EQ(2, gripper->num_itr_2);
}

TEST_F(MultiFingerTest, FingerSidesZero)
{
  ResetVariables();
  num_fingers_side_1 = 0 ;
  num_fingers_side_2 = 0 ;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);

  num_fingers_side_1 = 1 ;
  num_fingers_side_2 = 0;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);


  num_fingers_side_1 = 0;
  num_fingers_side_2 = 1;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);

  num_fingers_side_1 = 1;
  num_fingers_side_2 = 1;
  distance_between_fingers_1 = 0;
  distance_between_fingers_2 = 0;
  EXPECT_NO_THROW(LoadGripper());

}

TEST_F(MultiFingerTest, FingerSidesNeg)
{
  ResetVariables();
  num_fingers_side_1 = -1 ;
  num_fingers_side_2 = -1 ;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);

  num_fingers_side_1 = -1 ;
  num_fingers_side_2 = 1;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);

  num_fingers_side_1 = 1;
  num_fingers_side_2 = -1;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);
}

TEST_F(MultiFingerTest, Side1MultipleZeroSpacing)
{
  ResetVariables();
  num_fingers_side_1 = 2 ;
  num_fingers_side_2 = 1 ;
  distance_between_fingers_1 = 0.02 ;
  distance_between_fingers_2 = 0;
  finger_thickness = 0.023 ;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);
}

TEST_F(MultiFingerTest, Side2MultipleZeroSpacing)
{
  ResetVariables();
  num_fingers_side_1 = 1;
  num_fingers_side_2 = 4 ;
  distance_between_fingers_1 = 0;
  distance_between_fingers_2 = 0.03;
  finger_thickness = 0.04;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);
}

TEST_F(MultiFingerTest, BothSidesMultipleZeroSpacing)
{
  ResetVariables();
  finger_thickness = 0.04;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);
}

TEST_F(MultiFingerTest, ThicknessMoreThanSpacing)
{
  ResetVariables();
  distance_between_fingers_1 = 0.02 ;
  finger_thickness = 0.023 ;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);

  ResetVariables();
  distance_between_fingers_2 = 0.02 ;
  finger_thickness = 0.023 ;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);
}
TEST_F(MultiFingerTest, FingerThicknessZero)
{
  ResetVariables();
  finger_thickness = 0;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);
}
TEST_F(MultiFingerTest, FingerThicknessNeg)
{
  ResetVariables();
  finger_thickness = -0.1;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);
}

TEST_F(MultiFingerTest, GripperStrokeZero)
{
  ResetVariables();
  gripper_stroke = 0;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);
}
TEST_F(MultiFingerTest, GripperStrokeNeg)
{
  ResetVariables();
  gripper_stroke = -0.1;
  EXPECT_THROW(LoadGripper(), std::invalid_argument);
}

TEST_F(MultiFingerTest, AddPlaneTestInside1)
{
  ResetVariables();
  ASSERT_NO_THROW(LoadGripper());
  float dist = 0.01;
  Eigen::Vector4f centerpoint(0, 0, 0, 0);
  Eigen::Vector4f plane_vector(2, -8, 5, 18);
  gripper->addPlane(dist, centerpoint, plane_vector, true, false);
  EXPECT_EQ(1, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(1, static_cast<int>(gripper->grasp_samples.size()));
  EXPECT_EQ(1, static_cast<int>(gripper->plane_1_index.size()));
  EXPECT_EQ(0, static_cast<int>(gripper->plane_2_index.size()));

  Eigen::Vector4f plane_vector2(-12, 3, -18, 129);
  gripper->addPlane(dist, centerpoint, plane_vector2, true, true);
  EXPECT_EQ(2, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(2, static_cast<int>(gripper->grasp_samples.size()));
  EXPECT_EQ(2, static_cast<int>(gripper->plane_1_index.size()));
  EXPECT_EQ(1, static_cast<int>(gripper->plane_2_index.size()));

}

TEST_F(MultiFingerTest, AddPlaneTestInside2)
{
  ResetVariables();
  ASSERT_NO_THROW(LoadGripper());
  float dist = 0.01;
  Eigen::Vector4f centerpoint(0, 0, 0, 0);
  Eigen::Vector4f plane_vector(2, -8, 5, 18);
  gripper->addPlane(dist, centerpoint, plane_vector, false, true);
  EXPECT_EQ(1, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(1, static_cast<int>(gripper->grasp_samples.size()));
  EXPECT_EQ(0, static_cast<int>(gripper->plane_1_index.size()));
  EXPECT_EQ(1, static_cast<int>(gripper->plane_2_index.size()));

  Eigen::Vector4f plane_vector2(-12, 3, -18, 129);
  gripper->addPlane(dist, centerpoint, plane_vector2, true, true);
  EXPECT_EQ(2, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(2, static_cast<int>(gripper->grasp_samples.size()));
  EXPECT_EQ(1, static_cast<int>(gripper->plane_1_index.size()));
  EXPECT_EQ(2, static_cast<int>(gripper->plane_2_index.size()));
}

TEST_F(MultiFingerTest, AddPlaneTestInsideBoth)
{
  ResetVariables();
  ASSERT_NO_THROW(LoadGripper());
  float dist = 0.01;
  Eigen::Vector4f centerpoint(0, 0, 0, 0);
  Eigen::Vector4f plane_vector(2, -8, 5, 18);
  gripper->addPlane(dist, centerpoint, plane_vector, true, true);
  EXPECT_EQ(1, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(1, static_cast<int>(gripper->grasp_samples.size()));
  EXPECT_EQ(1, static_cast<int>(gripper->plane_1_index.size()));
  EXPECT_EQ(1, static_cast<int>(gripper->plane_2_index.size()));

  Eigen::Vector4f plane_vector2(-12, 3, -18, 129);
  gripper->addPlane(dist, centerpoint, plane_vector2, false, false);
  EXPECT_EQ(2, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(2, static_cast<int>(gripper->grasp_samples.size()));
  EXPECT_EQ(1, static_cast<int>(gripper->plane_1_index.size()));
  EXPECT_EQ(1, static_cast<int>(gripper->plane_2_index.size()));
}


TEST_F(MultiFingerTest, CheckPlaneExistsTest)
{
  ResetVariables();
  ASSERT_NO_THROW(LoadGripper());
  float dist = 0.01;
  Eigen::Vector4f centerpoint(0, 0, 0, 0);
  Eigen::Vector4f plane_vector(2, -8, 5, 18);
  gripper->addPlane(dist, centerpoint, plane_vector, true, true);

  float dist2 = 0.03;
  Eigen::Vector4f centerpoint2(0.01, 0.02, -0.03, 0);
  Eigen::Vector4f plane_vector2(2, -8, 5, 18);
  gripper->addPlane(dist2, centerpoint2, plane_vector2, false, true);

  float dist3 = 0.05;
  Eigen::Vector4f centerpoint3(0, 0.02, -0.03, 0);
  Eigen::Vector4f plane_vector3(2, -8, 5, 1);
  gripper->addPlane(dist3, centerpoint3, plane_vector3, false, false);


  EXPECT_EQ(0, gripper->checkPlaneExists(0.01));
  EXPECT_EQ(1, gripper->checkPlaneExists(0.03));
  EXPECT_EQ(2, gripper->checkPlaneExists(0.05));
  EXPECT_EQ(-1, gripper->checkPlaneExists(0.07));
}


TEST_F(MultiFingerTest, GetCenterCuttingPlaneCheck)
{
  GenerateObjectHorizontal();
  ResetVariables();
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  Eigen::Vector3d centerpoint(0.025, 0.005, 0.01);
  Eigen::Vector3d point_on_plane(0.025, 0.01, 0.01);
  Eigen::Vector3d vector_on_plane =  point_on_plane -  centerpoint;
  Eigen::Vector3d center_cutting_plane_normal(gripper->center_cutting_plane_normal(0),
   gripper->center_cutting_plane_normal(1),
   gripper->center_cutting_plane_normal(2));
  float dot_pdt = vector_on_plane.dot(center_cutting_plane_normal);
  ASSERT_NEAR(0, dot_pdt, 0.0001);

  // pcl::PointXYZ vector1(vector_on_plane(0), vector_on_plane(1), vector_on_plane(2));
  // pcl::PointXYZ vector2(gripper->center_cutting_plane_normal(0),
  //  gripper->center_cutting_plane_normal(1),
  //  gripper->center_cutting_plane_normal(2));
  // pcl::PointXYZ origin(0,0,0);
  // pcl::PointXYZ centerpointp(0.025, 0.005, 0.01);

  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb (
  //   object->cloud, 0, 255, 0);
  // viewer->addPointCloud<pcl::PointXYZRGB> (
  //   object->cloud, rgb, "rectangle_cloud" + object->object_name);

  // viewer->addLine(origin,vector1, 1, 0, 0, "Arrow_2");
  // viewer->addLine(origin,vector2, 1, 0, 0, "Arrow_1");
  // viewer->spin();
  // viewer->close();

}

TEST_F(MultiFingerTest, generateGraspSamplesTest)
{
//   Eigen::Vector3f point3f(
//   object->centerpoint(0),
//   object->centerpoint(1),
//   object->centerpoint(2));

// this->grasp_samples.push_back(
//   generateGraspSamples(
//     this->center_cutting_plane,
//     point3f,
//     0));
}

TEST_F(MultiFingerTest, addCuttingPlanesEqualAlignedOdd)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 3 ;
  num_fingers_side_2 = 1 ;
  distance_between_fingers_2 = 0;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->addCuttingPlanesEqualAligned(object->centerpoint, gripper->center_cutting_plane, false);
  ASSERT_EQ(3, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(1, static_cast<int>(gripper->plane_2_index.size()));
  EXPECT_EQ(0, gripper->plane_1_index[0]);
  EXPECT_EQ(1, gripper->plane_1_index[1]);
  EXPECT_EQ(2, gripper->plane_1_index[2]);
  EXPECT_EQ(0, gripper->plane_2_index[0]);
  ASSERT_EQ(3, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(0, gripper->cutting_plane_distances[0]);
  EXPECT_NEAR(-0.02, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(0.02, gripper->cutting_plane_distances[2], 0.00001);
}

TEST_F(MultiFingerTest, addCuttingPlanesEqualAlignedEven)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 2 ;
  num_fingers_side_2 = 4 ;
  distance_between_fingers_1 = 0.03;
  distance_between_fingers_2 = 0.03;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->addCuttingPlanesEqualAligned(object->centerpoint, gripper->center_cutting_plane, true);
  ASSERT_EQ(2, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(4, static_cast<int>(gripper->plane_2_index.size()));
  EXPECT_EQ(0, gripper->plane_1_index[0]);
  EXPECT_EQ(1, gripper->plane_1_index[1]);

  EXPECT_EQ(0, gripper->plane_2_index[0]);
  EXPECT_EQ(1, gripper->plane_2_index[1]);
  EXPECT_EQ(2, gripper->plane_2_index[2]);
  EXPECT_EQ(3, gripper->plane_2_index[3]);

  ASSERT_EQ(4, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_NEAR(-0.015, gripper->cutting_plane_distances[0], 0.00001);
  EXPECT_NEAR(0.015, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(-0.045, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(0.045, gripper->cutting_plane_distances[3], 0.00001);
}


TEST_F(MultiFingerTest, addCuttingPlanesSameDistDiffFingersOddEven)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 1 ;
  num_fingers_side_2 = 2 ;
  distance_between_fingers_1 = 0;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->addCuttingPlanes(object->centerpoint, gripper->center_cutting_plane, 0, 1, 0, 0.01);
  ASSERT_EQ(1, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(2, static_cast<int>(gripper->plane_2_index.size()));
  EXPECT_EQ(0, gripper->plane_1_index[0]);
  EXPECT_EQ(1, gripper->plane_2_index[0]);
  EXPECT_EQ(2, gripper->plane_2_index[1]);
  ASSERT_EQ(3, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(0, gripper->cutting_plane_distances[0]);
  EXPECT_NEAR(-0.01, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(0.01, gripper->cutting_plane_distances[2], 0.00001);
}

TEST_F(MultiFingerTest, addCuttingPlanesSameDistDiffFingersEvenOdd)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 2 ;
  num_fingers_side_2 = 5 ;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->addCuttingPlanes(object->centerpoint, gripper->center_cutting_plane, 1, 2, 0.01, 0.02);
  ASSERT_EQ(2, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(5, static_cast<int>(gripper->plane_2_index.size()));
  EXPECT_EQ(1, gripper->plane_1_index[0]);
  EXPECT_EQ(2, gripper->plane_1_index[1]);

  EXPECT_EQ(0, gripper->plane_2_index[0]);
  EXPECT_EQ(3, gripper->plane_2_index[1]);
  EXPECT_EQ(4, gripper->plane_2_index[2]);
  EXPECT_EQ(5, gripper->plane_2_index[3]);
  EXPECT_EQ(6, gripper->plane_2_index[4]);

  ASSERT_EQ(7, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(0, gripper->cutting_plane_distances[0]);
  EXPECT_NEAR(-0.01, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(0.01, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(-0.02, gripper->cutting_plane_distances[3], 0.00001);
  EXPECT_NEAR(0.02, gripper->cutting_plane_distances[4], 0.00001);
  EXPECT_NEAR(-0.04, gripper->cutting_plane_distances[5], 0.00001);
  EXPECT_NEAR(0.04, gripper->cutting_plane_distances[6], 0.00001);
}

TEST_F(MultiFingerTest, addCuttingPlanesDiffDistDiffFingersOddEven)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 3 ;
  num_fingers_side_2 = 4 ;
  distance_between_fingers_1 = 0.01;
  distance_between_fingers_2 = 0.02;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->addCuttingPlanes(object->centerpoint, gripper->center_cutting_plane, 1, 2, 0.01, 0.01);
  ASSERT_EQ(3, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(4, static_cast<int>(gripper->plane_2_index.size()));
  EXPECT_EQ(0, gripper->plane_1_index[0]);
  EXPECT_EQ(1, gripper->plane_1_index[1]);
  EXPECT_EQ(2, gripper->plane_1_index[2]);
  EXPECT_EQ(1, gripper->plane_2_index[0]);
  EXPECT_EQ(2, gripper->plane_2_index[1]);
  EXPECT_EQ(3, gripper->plane_2_index[2]);
  EXPECT_EQ(4, gripper->plane_2_index[3]);

  ASSERT_EQ(5, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(0, gripper->cutting_plane_distances[0]);
  EXPECT_NEAR(-0.01, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(0.01, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(-0.03, gripper->cutting_plane_distances[3], 0.00001);
  EXPECT_NEAR(0.03, gripper->cutting_plane_distances[4], 0.00001);
}

TEST_F(MultiFingerTest, addCuttingPlanesDiffDistDiffFingersEvenOdd)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 4 ;
  num_fingers_side_2 = 5 ;
  distance_between_fingers_1 = 0.02;
  distance_between_fingers_2 = 0.03;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->addCuttingPlanes(object->centerpoint, gripper->center_cutting_plane, 2, 2, 0.01, 0.03);
  ASSERT_EQ(4, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(5, static_cast<int>(gripper->plane_2_index.size()));
  EXPECT_EQ(1, gripper->plane_1_index[0]);
  EXPECT_EQ(2, gripper->plane_1_index[1]);
  EXPECT_EQ(3, gripper->plane_1_index[2]);
  EXPECT_EQ(4, gripper->plane_1_index[3]);

  EXPECT_EQ(0, gripper->plane_2_index[0]);
  EXPECT_EQ(3, gripper->plane_2_index[1]);
  EXPECT_EQ(4, gripper->plane_2_index[2]);
  EXPECT_EQ(5, gripper->plane_2_index[3]);
  EXPECT_EQ(6, gripper->plane_2_index[4]);

  ASSERT_EQ(7, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(0, gripper->cutting_plane_distances[0]);
  EXPECT_NEAR(-0.01, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(0.01, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(-0.03, gripper->cutting_plane_distances[3], 0.00001);
  EXPECT_NEAR(0.03, gripper->cutting_plane_distances[4], 0.00001);
  EXPECT_NEAR(-0.06, gripper->cutting_plane_distances[5], 0.00001);
  EXPECT_NEAR(0.06, gripper->cutting_plane_distances[6], 0.00001);
}

TEST_F(MultiFingerTest, addCuttingPlanesDiffDistBothEven)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 4 ;
  num_fingers_side_2 = 2 ;
  distance_between_fingers_1 = 0.01;
  distance_between_fingers_2 = 0.02;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->addCuttingPlanes(object->centerpoint, gripper->center_cutting_plane, 2, 1, 0.005, 0.01);
  ASSERT_EQ(4, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(2, static_cast<int>(gripper->plane_2_index.size()));
  EXPECT_EQ(0, gripper->plane_1_index[0]);
  EXPECT_EQ(1, gripper->plane_1_index[1]);
  EXPECT_EQ(2, gripper->plane_1_index[2]);
  EXPECT_EQ(3, gripper->plane_1_index[3]);
  EXPECT_EQ(4, gripper->plane_2_index[0]);
  EXPECT_EQ(5, gripper->plane_2_index[1]);
  
  ASSERT_EQ(6, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_NEAR(-0.005, gripper->cutting_plane_distances[0], 0.00001);
  EXPECT_NEAR(0.005, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(-0.015, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(0.015, gripper->cutting_plane_distances[3], 0.00001);
  EXPECT_NEAR(-0.01, gripper->cutting_plane_distances[4], 0.00001);
  EXPECT_NEAR(0.01, gripper->cutting_plane_distances[5], 0.00001);
}

TEST_F(MultiFingerTest, addCuttingPlanesDiffDistBothOdd)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 3 ;
  num_fingers_side_2 = 5 ;
  distance_between_fingers_1 = 0.03;
  distance_between_fingers_2 = 0.01;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->addCuttingPlanes(object->centerpoint, gripper->center_cutting_plane, 1, 2, 0.03, 0.01);
  ASSERT_EQ(3, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(5, static_cast<int>(gripper->plane_2_index.size()));

  EXPECT_EQ(0, gripper->plane_1_index[0]);
  EXPECT_EQ(1, gripper->plane_1_index[1]);
  EXPECT_EQ(2, gripper->plane_1_index[2]);

  EXPECT_EQ(0, gripper->plane_2_index[0]);
  EXPECT_EQ(3, gripper->plane_2_index[1]);
  EXPECT_EQ(4, gripper->plane_2_index[2]);
  EXPECT_EQ(5, gripper->plane_2_index[3]);
  EXPECT_EQ(6, gripper->plane_2_index[4]);
  
  ASSERT_EQ(7, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(0, gripper->cutting_plane_distances[0]);
  EXPECT_NEAR(-0.03, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(0.03, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(-0.01, gripper->cutting_plane_distances[3], 0.00001);
  EXPECT_NEAR(0.01, gripper->cutting_plane_distances[4], 0.00001);
  EXPECT_NEAR(-0.02, gripper->cutting_plane_distances[5], 0.00001);
  EXPECT_NEAR(0.02, gripper->cutting_plane_distances[6], 0.00001);

}

TEST_F(MultiFingerTest, GetCuttingPlaneBothOdd)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 5;
  num_fingers_side_2 = 3;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  ASSERT_EQ(5, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(3, static_cast<int>(gripper->plane_2_index.size()));

  EXPECT_EQ(0, gripper->plane_1_index[0]);
  EXPECT_EQ(1, gripper->plane_1_index[1]);
  EXPECT_EQ(2, gripper->plane_1_index[2]);
  EXPECT_EQ(3, gripper->plane_1_index[3]);
  EXPECT_EQ(4, gripper->plane_1_index[4]);

  EXPECT_EQ(0, gripper->plane_2_index[0]);
  EXPECT_EQ(1, gripper->plane_2_index[1]);
  EXPECT_EQ(2, gripper->plane_2_index[2]);

  ASSERT_EQ(5, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(0, gripper->cutting_plane_distances[0]);
  EXPECT_NEAR(-0.02, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(0.02, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(-0.04, gripper->cutting_plane_distances[3], 0.00001);
  EXPECT_NEAR(0.04, gripper->cutting_plane_distances[4], 0.00001);
}

TEST_F(MultiFingerTest, GetCuttingPlaneBothOddDiffDist)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 3;
  num_fingers_side_2 = 5;
  distance_between_fingers_1 = 0.02;
  distance_between_fingers_2 = 0.03;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);

  ASSERT_EQ(3, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(5, static_cast<int>(gripper->plane_2_index.size()));

  EXPECT_EQ(0, gripper->plane_1_index[0]);
  EXPECT_EQ(1, gripper->plane_1_index[1]);
  EXPECT_EQ(2, gripper->plane_1_index[2]);


  EXPECT_EQ(0, gripper->plane_2_index[0]);
  EXPECT_EQ(3, gripper->plane_2_index[1]);
  EXPECT_EQ(4, gripper->plane_2_index[2]);
  EXPECT_EQ(5, gripper->plane_2_index[3]);
  EXPECT_EQ(6, gripper->plane_2_index[4]);

  ASSERT_EQ(7, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(0, gripper->cutting_plane_distances[0]);
  EXPECT_NEAR(-0.02, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(0.02, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(-0.03, gripper->cutting_plane_distances[3], 0.00001);
  EXPECT_NEAR(0.03, gripper->cutting_plane_distances[4], 0.00001);
  EXPECT_NEAR(-0.06, gripper->cutting_plane_distances[5], 0.00001);
  EXPECT_NEAR(0.06, gripper->cutting_plane_distances[6], 0.00001);
}

TEST_F(MultiFingerTest, GetCuttingPlaneBothEven)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 2;
  num_fingers_side_2 = 4;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  
  ASSERT_EQ(2, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(4, static_cast<int>(gripper->plane_2_index.size()));

  EXPECT_EQ(0, gripper->plane_1_index[0]);
  EXPECT_EQ(1, gripper->plane_1_index[1]);


  EXPECT_EQ(0, gripper->plane_2_index[0]);
  EXPECT_EQ(1, gripper->plane_2_index[1]);
  EXPECT_EQ(2, gripper->plane_2_index[2]);
  EXPECT_EQ(3, gripper->plane_2_index[3]);

  ASSERT_EQ(4, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_NEAR(-0.01, gripper->cutting_plane_distances[0], 0.00001);
  EXPECT_NEAR(0.01, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(-0.03, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(0.03, gripper->cutting_plane_distances[3], 0.00001);
}

TEST_F(MultiFingerTest, GetCuttingPlaneBothEvenDiffDist)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 4;
  num_fingers_side_2 = 6;
  distance_between_fingers_1 = 0.01;
  distance_between_fingers_2 = 0.04;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  
  ASSERT_EQ(4, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(6, static_cast<int>(gripper->plane_2_index.size()));

  EXPECT_EQ(0, gripper->plane_1_index[0]);
  EXPECT_EQ(1, gripper->plane_1_index[1]);
  EXPECT_EQ(2, gripper->plane_1_index[2]);
  EXPECT_EQ(3, gripper->plane_1_index[3]);

  EXPECT_EQ(4, gripper->plane_2_index[0]);
  EXPECT_EQ(5, gripper->plane_2_index[1]);
  EXPECT_EQ(6, gripper->plane_2_index[2]);
  EXPECT_EQ(7, gripper->plane_2_index[3]);
  EXPECT_EQ(8, gripper->plane_2_index[4]);
  EXPECT_EQ(9, gripper->plane_2_index[5]);

  ASSERT_EQ(10, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_NEAR(-0.005, gripper->cutting_plane_distances[0], 0.00001);
  EXPECT_NEAR(0.005, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(-0.015, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(0.015, gripper->cutting_plane_distances[3], 0.00001);
  EXPECT_NEAR(-0.02, gripper->cutting_plane_distances[4], 0.00001);
  EXPECT_NEAR(0.02, gripper->cutting_plane_distances[5], 0.00001);
  EXPECT_NEAR(-0.06, gripper->cutting_plane_distances[6], 0.00001);
  EXPECT_NEAR(0.06, gripper->cutting_plane_distances[7], 0.00001);
  EXPECT_NEAR(-0.10, gripper->cutting_plane_distances[8], 0.00001);
  EXPECT_NEAR(0.10, gripper->cutting_plane_distances[9], 0.00001);
}

TEST_F(MultiFingerTest, GetCuttingPlaneOddEven)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 3;
  num_fingers_side_2 = 2;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  ASSERT_EQ(3, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(2, static_cast<int>(gripper->plane_2_index.size()));

  EXPECT_EQ(0, gripper->plane_1_index[0]);
  EXPECT_EQ(1, gripper->plane_1_index[1]);
  EXPECT_EQ(2, gripper->plane_1_index[2]);

  EXPECT_EQ(3, gripper->plane_2_index[0]);
  EXPECT_EQ(4, gripper->plane_2_index[1]);

  ASSERT_EQ(5, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(0, gripper->cutting_plane_distances[0]);
  EXPECT_NEAR(-0.02, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(0.02, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(-0.01, gripper->cutting_plane_distances[3], 0.00001);
  EXPECT_NEAR(0.01, gripper->cutting_plane_distances[4], 0.00001);  
}
TEST_F(MultiFingerTest, GetCuttingPlaneOddEvenDiffDist)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 5;
  num_fingers_side_2 = 4;
  distance_between_fingers_1 = 0.03;
  distance_between_fingers_2 = 0.02;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  ASSERT_EQ(5, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(4, static_cast<int>(gripper->plane_2_index.size()));

  EXPECT_EQ(0, gripper->plane_1_index[0]);
  EXPECT_EQ(1, gripper->plane_1_index[1]);
  EXPECT_EQ(2, gripper->plane_1_index[2]);
  EXPECT_EQ(3, gripper->plane_1_index[3]);
  EXPECT_EQ(4, gripper->plane_1_index[4]);

  EXPECT_EQ(5, gripper->plane_2_index[0]);
  EXPECT_EQ(6, gripper->plane_2_index[1]);
  EXPECT_EQ(1, gripper->plane_2_index[2]);
  EXPECT_EQ(2, gripper->plane_2_index[3]);

  ASSERT_EQ(7, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(0, gripper->cutting_plane_distances[0]);
  EXPECT_NEAR(-0.03, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(0.03, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(-0.06, gripper->cutting_plane_distances[3], 0.00001);
  EXPECT_NEAR(0.06, gripper->cutting_plane_distances[4], 0.00001);
  EXPECT_NEAR(-0.01, gripper->cutting_plane_distances[5], 0.00001);
  EXPECT_NEAR(0.01, gripper->cutting_plane_distances[6], 0.00001);
}

TEST_F(MultiFingerTest, GetCuttingPlaneEvenOdd)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 2;
  num_fingers_side_2 = 5;

  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  ASSERT_EQ(2, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(5, static_cast<int>(gripper->plane_2_index.size()));

  EXPECT_EQ(1, gripper->plane_1_index[0]);
  EXPECT_EQ(2, gripper->plane_1_index[1]);

  EXPECT_EQ(0, gripper->plane_2_index[0]);
  EXPECT_EQ(3, gripper->plane_2_index[1]);
  EXPECT_EQ(4, gripper->plane_2_index[2]);
  EXPECT_EQ(5, gripper->plane_2_index[3]);
  EXPECT_EQ(6, gripper->plane_2_index[4]);

  ASSERT_EQ(7, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(0, gripper->cutting_plane_distances[0]);
  EXPECT_NEAR(-0.01, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(0.01, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(-0.02, gripper->cutting_plane_distances[3], 0.00001);
  EXPECT_NEAR(0.02, gripper->cutting_plane_distances[4], 0.00001);
  EXPECT_NEAR(-0.04, gripper->cutting_plane_distances[5], 0.00001);
  EXPECT_NEAR(0.04, gripper->cutting_plane_distances[6], 0.00001);
}

TEST_F(MultiFingerTest, GetCuttingPlaneEvenOddDiffDist)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 4;
  num_fingers_side_2 = 3;
  distance_between_fingers_1 = 0.05;
  distance_between_fingers_2 = 0.02;

  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  ASSERT_EQ(4, static_cast<int>(gripper->plane_1_index.size()));
  ASSERT_EQ(3, static_cast<int>(gripper->plane_2_index.size()));

  EXPECT_EQ(1, gripper->plane_1_index[0]);
  EXPECT_EQ(2, gripper->plane_1_index[1]);
  EXPECT_EQ(3, gripper->plane_1_index[2]);
  EXPECT_EQ(4, gripper->plane_1_index[3]);

  EXPECT_EQ(0, gripper->plane_2_index[0]);
  EXPECT_EQ(5, gripper->plane_2_index[1]);
  EXPECT_EQ(6, gripper->plane_2_index[2]);


  ASSERT_EQ(7, static_cast<int>(gripper->cutting_plane_distances.size()));
  EXPECT_EQ(0, gripper->cutting_plane_distances[0]);
  EXPECT_NEAR(-0.025, gripper->cutting_plane_distances[1], 0.00001);
  EXPECT_NEAR(0.025, gripper->cutting_plane_distances[2], 0.00001);
  EXPECT_NEAR(-0.075, gripper->cutting_plane_distances[3], 0.00001);
  EXPECT_NEAR(0.075, gripper->cutting_plane_distances[4], 0.00001);
  EXPECT_NEAR(-0.02, gripper->cutting_plane_distances[5], 0.00001);
  EXPECT_NEAR(0.02, gripper->cutting_plane_distances[6], 0.00001);
}

TEST_F(MultiFingerTest, GetGraspCloudTest)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 3;
  num_fingers_side_2 = 2;
  distance_between_fingers_1 = 0.06;
  distance_between_fingers_2 = 0.02;

  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);

  ASSERT_EQ(5, static_cast<int>(gripper->grasp_samples.size()));
  EXPECT_TRUE(gripper->grasp_samples[0]->plane_intersects_object);
  EXPECT_FALSE(static_cast<int>(gripper->grasp_samples[1]->plane_intersects_object));
  EXPECT_FALSE(static_cast<int>(gripper->grasp_samples[2]->plane_intersects_object));
  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[3]->plane_intersects_object));
  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[4]->plane_intersects_object));
}

TEST_F(MultiFingerTest, InitialSamplePointsTestHorizontal)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 3;
  num_fingers_side_2 = 2;
  distance_between_fingers_1 = 0.06;
  distance_between_fingers_2 = 0.02;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);

  ASSERT_TRUE(gripper->getInitialSamplePoints(object));
  EXPECT_TRUE(gripper->grasp_samples[0]->sample_side_1->start_index >= 0);
  EXPECT_TRUE(gripper->grasp_samples[0]->sample_side_2->start_index >= 0);

  EXPECT_FALSE(gripper->grasp_samples[1]->sample_side_1->start_index >= 0);
  EXPECT_FALSE(gripper->grasp_samples[1]->sample_side_2->start_index >= 0);

  EXPECT_FALSE(gripper->grasp_samples[2]->sample_side_1->start_index >= 0);
  EXPECT_FALSE(gripper->grasp_samples[2]->sample_side_2->start_index >= 0);

  EXPECT_TRUE(gripper->grasp_samples[3]->sample_side_1->start_index >= 0);
  EXPECT_TRUE(gripper->grasp_samples[3]->sample_side_2->start_index >= 0);

  EXPECT_TRUE(gripper->grasp_samples[4]->sample_side_1->start_index >= 0);
  EXPECT_TRUE(gripper->grasp_samples[4]->sample_side_2->start_index >= 0);
}

TEST_F(MultiFingerTest, InitialSamplePointsTestVertical)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 2;
  num_fingers_side_2 = 4;
  distance_between_fingers_1 = 0.01;
  distance_between_fingers_2 = 0.04;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);
  gripper->getInitialSamplePoints(object);

  ASSERT_EQ(6, static_cast<int>(gripper->grasp_samples.size()));

  EXPECT_TRUE(gripper->grasp_samples[0]->sample_side_1->start_index >= 0);
  EXPECT_TRUE(gripper->grasp_samples[0]->sample_side_2->start_index >= 0);

  EXPECT_TRUE(gripper->grasp_samples[1]->sample_side_1->start_index >= 0);
  EXPECT_TRUE(gripper->grasp_samples[1]->sample_side_2->start_index >= 0);

  EXPECT_TRUE(gripper->grasp_samples[2]->sample_side_1->start_index >= 0);
  EXPECT_TRUE(gripper->grasp_samples[2]->sample_side_2->start_index >= 0);

  EXPECT_TRUE(gripper->grasp_samples[3]->sample_side_1->start_index >= 0);
  EXPECT_TRUE(gripper->grasp_samples[3]->sample_side_2->start_index >= 0);

  EXPECT_FALSE(gripper->grasp_samples[4]->sample_side_1->start_index >= 0);
  EXPECT_FALSE(gripper->grasp_samples[4]->sample_side_2->start_index >= 0);

  EXPECT_FALSE(gripper->grasp_samples[5]->sample_side_1->start_index >= 0);
  EXPECT_FALSE(gripper->grasp_samples[5]->sample_side_2->start_index >= 0);
}

TEST_F(MultiFingerTest, GetInitialSampleCloudTest)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 2;
  num_fingers_side_2 = 1;
  distance_between_fingers_1 = 0.01;
  distance_between_fingers_2 = 0;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);
  gripper->getInitialSamplePoints(object);
  gripper->getInitialSampleCloud(object);
  
  ASSERT_EQ(3, static_cast<int>(gripper->grasp_samples.size()));
  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[0]->sample_side_1->
    finger_cloud->points.size()) > 0);
  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[0]->sample_side_2->
    finger_cloud->points.size()) > 0);
  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[0]->sample_side_1->
    finger_ncloud->points.size()) > 0);
  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[0]->sample_side_2->
    finger_ncloud->points.size()) > 0);

  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[1]->sample_side_1->
    finger_cloud->points.size()) > 0);
  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[1]->sample_side_2->
    finger_cloud->points.size()) > 0);
  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[1]->sample_side_1->
    finger_ncloud->points.size()) > 0);
  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[1]->sample_side_2->
    finger_ncloud->points.size()) > 0);

  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[2]->sample_side_1->
    finger_cloud->points.size()) > 0);
  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[2]->sample_side_2->
    finger_cloud->points.size()) > 0);
  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[2]->sample_side_1->
    finger_ncloud->points.size()) > 0);
  EXPECT_TRUE(static_cast<int>(gripper->grasp_samples[2]->sample_side_2->
    finger_ncloud->points.size()) > 0);
}

TEST_F(MultiFingerTest, VoxelizeSampleCloudTest)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 2;
  num_fingers_side_2 = 1;
  distance_between_fingers_1 = 0.01;
  distance_between_fingers_2 = 0;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);
  gripper->getInitialSamplePoints(object);
  gripper->getInitialSampleCloud(object);
  gripper->voxelizeSampleCloud();
  
  ASSERT_EQ(3, static_cast<int>(gripper->grasp_samples.size()));
  int ncloud_side1_0 = static_cast<int>(gripper->grasp_samples[0]->sample_side_1->
    finger_cloud->points.size());
  int ncloud_side1_1 = static_cast<int>(gripper->grasp_samples[1]->sample_side_1->
    finger_cloud->points.size());
  int ncloud_side1_2 = static_cast<int>(gripper->grasp_samples[2]->sample_side_1->
    finger_cloud->points.size());
    
  int ncloud_side2_0 = static_cast<int>(gripper->grasp_samples[0]->sample_side_2->
    finger_cloud->points.size());
  int ncloud_side2_1 = static_cast<int>(gripper->grasp_samples[1]->sample_side_2->
    finger_cloud->points.size());
  int ncloud_side2_2 = static_cast<int>(gripper->grasp_samples[2]->sample_side_2->
    finger_cloud->points.size());

  int nvoxel_side1_0 = static_cast<int>(gripper->grasp_samples[0]->sample_side_1->
    finger_nvoxel->points.size());
  int nvoxel_side1_1 = static_cast<int>(gripper->grasp_samples[1]->sample_side_1->
    finger_nvoxel->points.size());
  int nvoxel_side1_2 = static_cast<int>(gripper->grasp_samples[2]->sample_side_1->
    finger_nvoxel->points.size());

  int nvoxel_side2_0 = static_cast<int>(gripper->grasp_samples[0]->sample_side_2->
    finger_nvoxel->points.size());
  int nvoxel_side2_1 = static_cast<int>(gripper->grasp_samples[1]->sample_side_2->
    finger_nvoxel->points.size());
  int nvoxel_side2_2 = static_cast<int>(gripper->grasp_samples[2]->sample_side_2->
    finger_nvoxel->points.size());
  

  EXPECT_TRUE(ncloud_side1_0 > nvoxel_side1_0);
  EXPECT_TRUE(ncloud_side2_0 > nvoxel_side2_0);
  EXPECT_TRUE(ncloud_side1_1 > nvoxel_side1_1);
  EXPECT_TRUE(ncloud_side2_1 > nvoxel_side2_1);
  EXPECT_TRUE(ncloud_side1_2 > nvoxel_side1_2);
  EXPECT_TRUE(ncloud_side2_2 > nvoxel_side2_2);
}

TEST_F(MultiFingerTest, UpdateMaxAttributesTest)
{
  GenerateObjectVertical();
  ResetVariables();
  ASSERT_NO_THROW(LoadGripper());
  std::shared_ptr<fingerCloudSample> sample(new fingerCloudSample);
  gripper->updateMaxMinAttributes(sample, 0.1, 0.2, 0.3);
  EXPECT_NEAR(0.1, sample->centroid_dist_min, 0.00001);
  EXPECT_NEAR(0.1, sample->centroid_dist_max, 0.00001);
  EXPECT_NEAR(0.2, sample->grasp_plane_dist_min, 0.00001);
  EXPECT_NEAR(0.2, sample->grasp_plane_dist_max, 0.00001);
  EXPECT_NEAR(0.3, sample->curvature_min, 0.00001);
  EXPECT_NEAR(0.3, sample->curvature_max, 0.00001);

  gripper->updateMaxMinAttributes(sample, 0.4, 0.1, 0.3);
  EXPECT_NEAR(0.1, sample->centroid_dist_min, 0.00001);
  EXPECT_NEAR(0.4, sample->centroid_dist_max, 0.00001);
  EXPECT_NEAR(0.1, sample->grasp_plane_dist_min, 0.00001);
  EXPECT_NEAR(0.2, sample->grasp_plane_dist_max, 0.00001);
  EXPECT_NEAR(0.3, sample->curvature_min, 0.00001);
  EXPECT_NEAR(0.3, sample->curvature_max, 0.00001);

  gripper->updateMaxMinAttributes(sample, -0.2, 0.7, 1.4);
  EXPECT_NEAR(-0.2, sample->centroid_dist_min, 0.00001);
  EXPECT_NEAR(0.4, sample->centroid_dist_max, 0.00001);
  EXPECT_NEAR(0.1, sample->grasp_plane_dist_min, 0.00001);
  EXPECT_NEAR(0.7, sample->grasp_plane_dist_max, 0.00001);
  EXPECT_NEAR(0.3, sample->curvature_min, 0.00001);
  EXPECT_NEAR(1.4, sample->curvature_max, 0.00001);
}

TEST_F(MultiFingerTest, GetMaxMinValuesTest)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 2;
  num_fingers_side_2 = 1;
  distance_between_fingers_1 = 0.01;
  distance_between_fingers_2 = 0;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);
  gripper->getInitialSamplePoints(object);
  gripper->getInitialSampleCloud(object);
  gripper->voxelizeSampleCloud();
  gripper->getMaxMinValues(object);

  pcl::PointNormal centroid_point{
    object->centerpoint(0),
    object->centerpoint(1),
    object->centerpoint(2)};
  std::vector<float> curvature_vec_1;
  std::vector<float> grasp_plane_distance_vec_1;
  std::vector<float> centroid_distance_vec_1;

  std::vector<float> curvature_vec_2;
  std::vector<float> grasp_plane_distance_vec_2;
  std::vector<float> centroid_distance_vec_2;
  for (auto & sample : gripper->grasp_samples) {
    if (sample->plane_intersects_object) {
      for (auto & point : sample->sample_side_1->finger_nvoxel->points) {
        curvature_vec_1.push_back(point.curvature);
        centroid_distance_vec_1.push_back(
          pcl::geometry::distance(point, centroid_point));
        grasp_plane_distance_vec_1.push_back(
          PCLFunctions::pointToPlane(sample->plane_eigen, point));
      }

      for (auto & point : sample->sample_side_2->finger_nvoxel->points) {
        curvature_vec_2.push_back(point.curvature);
        centroid_distance_vec_2.push_back(
          pcl::geometry::distance(point, centroid_point));
        grasp_plane_distance_vec_2.push_back(
          PCLFunctions::pointToPlane(sample->plane_eigen, point));
      }

      EXPECT_NEAR(
        grasp_plane_distance_vec_1[
          std::distance(
            grasp_plane_distance_vec_1.begin(),
            std::min_element(grasp_plane_distance_vec_1.begin(),
              grasp_plane_distance_vec_1.end()))],
        sample->sample_side_1->grasp_plane_dist_min,
        0.00001);
      
      EXPECT_NEAR(
        curvature_vec_1[
          std::distance(
            curvature_vec_1.begin(),
            std::min_element(curvature_vec_1.begin(), curvature_vec_1.end()))],
        sample->sample_side_1->curvature_min,
        0.00001);
      
      EXPECT_NEAR(
        centroid_distance_vec_1[
          std::distance(
            centroid_distance_vec_1.begin(),
            std::min_element(centroid_distance_vec_1.begin(),
              centroid_distance_vec_1.end()))],
        sample->sample_side_1->centroid_dist_min,
        0.00001);

      EXPECT_NEAR(
        grasp_plane_distance_vec_2[
          std::distance(
            grasp_plane_distance_vec_2.begin(),
            std::min_element(grasp_plane_distance_vec_2.begin(),
              grasp_plane_distance_vec_2.end()))],
        sample->sample_side_2->grasp_plane_dist_min,
        0.00001);
      
      EXPECT_NEAR(
        curvature_vec_2[
          std::distance(
            curvature_vec_2.begin(),
            std::min_element(curvature_vec_2.begin(), curvature_vec_2.end()))],
        sample->sample_side_2->curvature_min,
        0.00001);
      
      EXPECT_NEAR(
        centroid_distance_vec_2[
          std::distance(
            centroid_distance_vec_2.begin(),
            std::min_element(centroid_distance_vec_2.begin(),
              centroid_distance_vec_2.end()))],
        sample->sample_side_2->centroid_dist_min,
        0.00001);

      EXPECT_NEAR(
        grasp_plane_distance_vec_1[
          std::distance(
            grasp_plane_distance_vec_1.begin(),
            std::max_element(grasp_plane_distance_vec_1.begin(),
              grasp_plane_distance_vec_1.end()))],
        sample->sample_side_1->grasp_plane_dist_max,
        0.00001);
      
      EXPECT_NEAR(
        curvature_vec_1[
          std::distance(
            curvature_vec_1.begin(),
            std::max_element(curvature_vec_1.begin(), curvature_vec_1.end()))],
        sample->sample_side_1->curvature_max,
        0.00001);
      
      EXPECT_NEAR(
        centroid_distance_vec_1[
          std::distance(
            centroid_distance_vec_1.begin(),
            std::max_element(centroid_distance_vec_1.begin(),
              centroid_distance_vec_1.end()))],
        sample->sample_side_1->centroid_dist_max,
        0.00001);

      EXPECT_NEAR(
        grasp_plane_distance_vec_2[
          std::distance(
            grasp_plane_distance_vec_2.begin(),
            std::max_element(grasp_plane_distance_vec_2.begin(),
              grasp_plane_distance_vec_2.end()))],
        sample->sample_side_2->grasp_plane_dist_max,
        0.00001);
      
      EXPECT_NEAR(
        curvature_vec_2[
          std::distance(
            curvature_vec_2.begin(),
            std::max_element(curvature_vec_2.begin(), curvature_vec_2.end()))],
        sample->sample_side_2->curvature_max,
        0.00001);
      
      EXPECT_NEAR(
        centroid_distance_vec_2[
          std::distance(
            centroid_distance_vec_2.begin(),
            std::max_element(centroid_distance_vec_2.begin(),
              centroid_distance_vec_2.end()))],
        sample->sample_side_2->centroid_dist_max,
        0.00001);

      curvature_vec_1.clear();
      grasp_plane_distance_vec_1.clear();
      centroid_distance_vec_1.clear();
      curvature_vec_2.clear();
      grasp_plane_distance_vec_2.clear();
      centroid_distance_vec_2.clear();
    }
  }

}

TEST_F(MultiFingerTest, GetFingerSamplesTest)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 2;
  num_fingers_side_2 = 1;
  distance_between_fingers_1 = 0.01;
  distance_between_fingers_2 = 0;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);
  gripper->getInitialSamplePoints(object);
  gripper->getInitialSampleCloud(object);
  gripper->voxelizeSampleCloud();
  gripper->getMaxMinValues(object);


  for (auto & sample : gripper->grasp_samples) {
    EXPECT_TRUE(static_cast<int>(sample->sample_side_1->finger_samples.size()) == 0);
    EXPECT_TRUE(static_cast<int>(sample->sample_side_2->finger_samples.size()) == 0);
  }
  gripper->getFingerSamples(object);

  for (auto & sample : gripper->grasp_samples) {
    EXPECT_TRUE(static_cast<int>(sample->sample_side_1->finger_samples.size()) > 0);
    EXPECT_TRUE(static_cast<int>(sample->sample_side_2->finger_samples.size()) > 0);
  }

}

TEST_F(MultiFingerTest, GetGripperClustersTest1)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 4;
  num_fingers_side_2 = 3;
  distance_between_fingers_1 = 0.02;
  distance_between_fingers_2 = 0.03;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);
  gripper->getInitialSamplePoints(object);
  gripper->getInitialSampleCloud(object);
  gripper->voxelizeSampleCloud();
  gripper->getMaxMinValues(object);
  gripper->getFingerSamples(object);
  EXPECT_TRUE(static_cast<int>(gripper->gripper_clusters.size()) == 0);
  gripper->getGripperClusters();
  EXPECT_TRUE(static_cast<int>(gripper->gripper_clusters.size()) == 7);
}

TEST_F(MultiFingerTest, GetGripperClustersTest2)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 1;
  num_fingers_side_2 = 1;
  distance_between_fingers_1 = 0;
  distance_between_fingers_2 = 0;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);
  gripper->getInitialSamplePoints(object);
  gripper->getInitialSampleCloud(object);
  gripper->voxelizeSampleCloud();
  gripper->getMaxMinValues(object);
  gripper->getFingerSamples(object);
  EXPECT_TRUE(static_cast<int>(gripper->gripper_clusters.size()) == 0);
  gripper->getGripperClusters();
  EXPECT_TRUE(static_cast<int>(gripper->gripper_clusters.size()) == 2);
}

TEST_F(MultiFingerTest, GetGripperClustersTest3)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 4;
  num_fingers_side_2 = 2;
  distance_between_fingers_1 = 0.02;
  distance_between_fingers_2 = 0.01;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);
  gripper->getInitialSamplePoints(object);
  gripper->getInitialSampleCloud(object);
  gripper->voxelizeSampleCloud();
  gripper->getMaxMinValues(object);
  gripper->getFingerSamples(object);
  EXPECT_TRUE(static_cast<int>(gripper->gripper_clusters.size()) == 0);
  gripper->getGripperClusters();
  EXPECT_TRUE(static_cast<int>(gripper->gripper_clusters.size()) == 6);
}

TEST_F(MultiFingerTest, getGripperPlaneTest)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 3;
  num_fingers_side_2 = 2;
  distance_between_fingers_1 = 0.02;
  distance_between_fingers_2 = 0.03;
  ASSERT_NO_THROW(LoadGripper());
  pcl::PointNormal finger_point_1;
  finger_point_1.x = -0.005;
  finger_point_1.y = 0.025;
  finger_point_1.z = 0.01;
  pcl::PointNormal finger_point_2;
  finger_point_2.x = 0.015;
  finger_point_2.y = 0.025;
  finger_point_2.z = 0.01;
  Eigen::Vector3f finger_point_1_eigen(-0.005, 0.025, 0.01);
  Eigen::Vector3f finger_point_2_eigen(0.015, 0.025, 0.01);

  Eigen::Vector3f grasp_direction = Eigen::ParametrizedLine<float, 3>::Through(
          finger_point_1_eigen, finger_point_2_eigen).direction();
  auto finger_1 = std::make_shared<singleFinger>(
    finger_point_1,
    0,0,0,0
  );
  auto finger_2 = std::make_shared<singleFinger>(
    finger_point_2,
    0,0,0,0
  );
  Eigen::Vector3f output_vec = gripper->getGripperPlane(
    finger_1,
    finger_2,
    grasp_direction,
    object
  );

  float dot_pdt = output_vec.dot(grasp_direction);
  ASSERT_NEAR(0, dot_pdt, 0.00001);
  // Must be perpendicular to the grasp direction
}

TEST_F(MultiFingerTest, getOpenFingerCoordinatesTest)
{
  GenerateObjectVertical();
  ResetVariables();
  gripper_stroke = 0.04;
  ASSERT_NO_THROW(LoadGripper());
  
  Eigen::Vector3f closed_finger_point_1(0.06, 0.03, 0.03);
  Eigen::Vector3f closed_finger_point_2(0.06, 0.03, 0.01);
  Eigen::Vector3f grasp_direction = Eigen::ParametrizedLine<float, 3>::Through(
          closed_finger_point_1, closed_finger_point_2).direction();
  std::vector<Eigen::Vector3f> open_fingers = gripper->getOpenFingerCoordinates(
    grasp_direction, closed_finger_point_1, closed_finger_point_2);
  
  EXPECT_EQ(2, static_cast<int>(open_fingers.size()));
  EXPECT_NEAR(0.06, open_fingers[0](0), 0.00001);
  EXPECT_NEAR(0.03, open_fingers[0](1), 0.00001);
  EXPECT_NEAR(0.04, open_fingers[0](2), 0.00001);

  EXPECT_NEAR(0.06, open_fingers[1](0), 0.00001);
  EXPECT_NEAR(0.03, open_fingers[1](1), 0.00001);
  EXPECT_NEAR(0, open_fingers[1](2), 0.00001);
  
}

// Test works with compiler hackish solution
TEST_F(MultiFingerTest, checkFingerCollisionTest)
{
  GenerateObjectVertical();
  ResetVariables();
  gripper_stroke = 0.04;
  ASSERT_NO_THROW(LoadGripper());

  GenerateObjectCollision(0.01, 0.05, 0.02);

  Eigen::Vector3f finger_point_sample_center(0.005, 0.025, 0.01);
  Eigen::Vector3f finger_point_far_away(0.1, 0.25, 0.1);
  EXPECT_TRUE(gripper->checkFingerCollision(finger_point_sample_center, collision_object_ptr));
  EXPECT_FALSE(gripper->checkFingerCollision(finger_point_far_away, collision_object_ptr));
}

TEST_F(MultiFingerTest, getNearestPlaneIndexTest)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 3;
  num_fingers_side_2 = 2;
  distance_between_fingers_1 = 0.02;
  distance_between_fingers_2 = 0.01;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);
  gripper->getInitialSamplePoints(object);
  gripper->getInitialSampleCloud(object);
  gripper->voxelizeSampleCloud();
  gripper->getMaxMinValues(object);
  gripper->getFingerSamples(object);
  gripper->getGripperClusters();
  EXPECT_EQ(0, gripper->getNearestPlaneIndex(0));
  EXPECT_EQ(1, gripper->getNearestPlaneIndex(-0.03));
  EXPECT_EQ(2, gripper->getNearestPlaneIndex(0.015));
  EXPECT_EQ(3, gripper->getNearestPlaneIndex(-0.01));
  EXPECT_EQ(3, gripper->getNearestPlaneIndex(-0.004));
  EXPECT_EQ(4, gripper->getNearestPlaneIndex(0.003));
}

TEST_F(MultiFingerTest, getNearestPointIndexTest)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 4;
  num_fingers_side_2 = 2;
  distance_between_fingers_1 = 0.02;
  distance_between_fingers_2 = 0.01;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);
  gripper->getInitialSamplePoints(object);
  gripper->getInitialSampleCloud(object);
  gripper->voxelizeSampleCloud();
  gripper->getMaxMinValues(object);
  gripper->getFingerSamples(object);
  EXPECT_TRUE(static_cast<int>(gripper->gripper_clusters.size()) == 0);
  gripper->getGripperClusters();
  pcl::PointNormal midpoint;
  midpoint.x = 0.005;
  midpoint.y = 0.025;
  midpoint.z = 0.01;
  EXPECT_TRUE(gripper->getNearestPointIndex(midpoint, object->cloud_normal) > 0);

  pcl::PointNormal origin;
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;
  EXPECT_TRUE(gripper->getNearestPointIndex(origin, object->cloud_normal) == 0);

  pcl::PointNormal bottom_corner;
  midpoint.x = 0.01;
  midpoint.y = 0.05;
  midpoint.z = 0.02;
  EXPECT_TRUE(gripper->getNearestPointIndex(midpoint, object->cloud_normal) ==
    static_cast<int>(object->cloud_normal->points.size()) - 1);
}

TEST_F(MultiFingerTest, generateGripperOpenConfigTest)
{
  GenerateObjectHorizontal();
  ResetVariables();
  num_fingers_side_1 = 4;
  num_fingers_side_2 = 2;
  distance_between_fingers_1 = 0.02;
  distance_between_fingers_2 = 0.01;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);
  gripper->getInitialSamplePoints(object);
  gripper->getInitialSampleCloud(object);
  gripper->voxelizeSampleCloud();
  gripper->getMaxMinValues(object);
  gripper->getFingerSamples(object);
  gripper->getGripperClusters();

  Eigen::Vector3f closed_finger_point_1(0.005, 0.025, 0.025);
  Eigen::Vector3f closed_finger_point_2(0.005, 0.025, -0.005);
  pcl::PointNormal finger_point_1;
  finger_point_1.x = closed_finger_point_1(0);
  finger_point_1.y = closed_finger_point_1(1);
  finger_point_1.z = closed_finger_point_1(2);

  pcl::PointNormal finger_point_2;
  finger_point_2.x = closed_finger_point_2(0);
  finger_point_2.y = closed_finger_point_2(1);
  finger_point_2.z = closed_finger_point_2(2);

  Eigen::Vector3f grasp_direction = Eigen::ParametrizedLine<float, 3>::Through(
          closed_finger_point_1, closed_finger_point_2).direction();

  auto finger_1 = std::make_shared<singleFinger>(
    finger_point_1,
    0,0,0,0
  );
  auto finger_2 = std::make_shared<singleFinger>(
    finger_point_2,
    0,0,0,0
  );
  Eigen::Vector3f perpendicular_grasp_direction = gripper->getGripperPlane(
    finger_1,
    finger_2,
    grasp_direction,
    object
  );

  std::vector<Eigen::Vector3f> open_coords = gripper->getOpenFingerCoordinates(
    grasp_direction,
    closed_finger_point_1,
    closed_finger_point_2);

  GenerateObjectCollision(0.01, 0.05, 0.02);
  std::shared_ptr<multiFingerGripper> gripper_sample = gripper->generateGripperOpenConfig(
    collision_object_ptr, finger_1, finger_2,
    open_coords[0], open_coords[1], perpendicular_grasp_direction,
    grasp_direction);

  EXPECT_FALSE(gripper_sample->collides_with_world);
  EXPECT_EQ(4, static_cast<int>(gripper_sample->closed_fingers_1.size()));
  EXPECT_EQ(4, static_cast<int>(gripper_sample->open_fingers_1.size()));
  EXPECT_EQ(2, static_cast<int>(gripper_sample->closed_fingers_2.size()));
  EXPECT_EQ(2, static_cast<int>(gripper_sample->open_fingers_2.size()));

  EXPECT_EQ(0, gripper_sample->closed_fingers_1[0]->plane_index);
  EXPECT_EQ(1, gripper_sample->closed_fingers_1[1]->plane_index);
  EXPECT_EQ(2, gripper_sample->closed_fingers_1[2]->plane_index);
  EXPECT_EQ(3, gripper_sample->closed_fingers_1[3]->plane_index);

  EXPECT_EQ(4, gripper_sample->closed_fingers_2[0]->plane_index);
  EXPECT_EQ(5, gripper_sample->closed_fingers_2[1]->plane_index);
}

TEST_F(MultiFingerTest, generateGripperOpenConfigTestCollision)
{
  GenerateObjectVertical();
  ResetVariables();
  num_fingers_side_1 = 4;
  num_fingers_side_2 = 2;
  distance_between_fingers_1 = 0.02;
  distance_between_fingers_2 = 0.01;
  gripper_stroke = 0.03;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
  gripper->getCuttingPlanes(object);
  gripper->getGraspCloud(object);
  gripper->getInitialSamplePoints(object);
  gripper->getInitialSampleCloud(object);
  gripper->voxelizeSampleCloud();
  gripper->getMaxMinValues(object);
  gripper->getFingerSamples(object);
  gripper->getGripperClusters();

  Eigen::Vector3f closed_finger_point_1(0.005, 0.025, 0.025);
  Eigen::Vector3f closed_finger_point_2(0.005, 0.025, 0.0);
  pcl::PointNormal finger_point_1;
  finger_point_1.x = closed_finger_point_1(0);
  finger_point_1.y = closed_finger_point_1(1);
  finger_point_1.z = closed_finger_point_1(2);

  pcl::PointNormal finger_point_2;
  finger_point_2.x = closed_finger_point_2(0);
  finger_point_2.y = closed_finger_point_2(1);
  finger_point_2.z = closed_finger_point_2(2);

  Eigen::Vector3f grasp_direction = Eigen::ParametrizedLine<float, 3>::Through(
          closed_finger_point_1, closed_finger_point_2).direction();

  auto finger_1 = std::make_shared<singleFinger>(
    finger_point_1,
    0,0,0,0
  );
  auto finger_2 = std::make_shared<singleFinger>(
    finger_point_2,
    0,0,0,0
  );
  Eigen::Vector3f perpendicular_grasp_direction = gripper->getGripperPlane(
    finger_1,
    finger_2,
    grasp_direction,
    object
  );

  std::vector<Eigen::Vector3f> open_coords = gripper->getOpenFingerCoordinates(
    grasp_direction,
    closed_finger_point_1,
    closed_finger_point_2);

  GenerateObjectCollision(0.01, 0.05, 0.02);
  std::shared_ptr<multiFingerGripper> gripper_sample = gripper->generateGripperOpenConfig(
    collision_object_ptr, finger_1, finger_2,
    open_coords[0], open_coords[1], perpendicular_grasp_direction,
    grasp_direction);

  EXPECT_TRUE(gripper_sample->collides_with_world);
  EXPECT_EQ(4, static_cast<int>(gripper_sample->closed_fingers_1.size()));
  EXPECT_EQ(4, static_cast<int>(gripper_sample->open_fingers_1.size()));
  EXPECT_EQ(2, static_cast<int>(gripper_sample->closed_fingers_2.size()));
  EXPECT_EQ(2, static_cast<int>(gripper_sample->open_fingers_2.size()));

  EXPECT_EQ(0, gripper_sample->closed_fingers_1[0]->plane_index);
  EXPECT_EQ(1, gripper_sample->closed_fingers_1[1]->plane_index);
  EXPECT_EQ(2, gripper_sample->closed_fingers_1[2]->plane_index);
  EXPECT_EQ(3, gripper_sample->closed_fingers_1[3]->plane_index);

  EXPECT_EQ(4, gripper_sample->closed_fingers_2[0]->plane_index);
  EXPECT_EQ(5, gripper_sample->closed_fingers_2[1]->plane_index);
}

TEST_F(MultiFingerTest, getAllGripperConfigsTest)
{
  GenerateObjectVertical();
  ResetVariables();
  gripper_stroke = 0.04;
  ASSERT_NO_THROW(LoadGripper());

}

TEST_F(MultiFingerTest, getGraspPoseTest)
{

}

TEST_F(MultiFingerTest, getGripperRankTest)
{

}

TEST_F(MultiFingerTest, getAllRanksTest)
{

}