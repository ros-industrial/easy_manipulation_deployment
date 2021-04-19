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
  num_fingers_side_1 = 1;
  num_fingers_side_2 = 1;
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
    gripper = std::make_shared<FingerGripper>(gripper_);
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
  ResetVariables();
  num_fingers_side_1 = 3 ;
  num_fingers_side_2 = 1 ;
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
  std::cout << "addCuttingPlanesSameDistDiffFingersOddEven" << std::endl;
  ResetVariables();
  num_fingers_side_1 = 1 ;
  num_fingers_side_2 = 2 ;
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
  std::cout << "addCuttingPlanesSameDistDiffFingersEvenOdd" << std::endl;
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
  std::cout << "addCuttingPlanesDiffDistDiffFingersOddEven" << std::endl;
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
  std::cout << "addCuttingPlanesDiffDistDiffFingersOddEven" << std::endl;
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
  ResetVariables();
  num_fingers_side_1 = 1;
  num_fingers_side_2 = 3;
  distance_between_fingers_1 = 0.02;
  distance_between_fingers_2 = 0.02;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
}

TEST_F(MultiFingerTest, GetCuttingPlaneBothEven)
{
  ResetVariables();
  num_fingers_side_1 = 2;
  num_fingers_side_2 = 2;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
}

TEST_F(MultiFingerTest, GetCuttingPlaneBothEvenDiffDist)
{
  
}

TEST_F(MultiFingerTest, GetCuttingPlaneOddEven)
{
  ResetVariables();
  num_fingers_side_1 = 1;
  num_fingers_side_2 = 2;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
}
TEST_F(MultiFingerTest, GetCuttingPlaneOddEvenDiffDist)
{
  ResetVariables();
  num_fingers_side_1 = 4;
  num_fingers_side_2 = 3;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
}

TEST_F(MultiFingerTest, GetCuttingPlaneEvenOdd)
{
  ResetVariables();
  num_fingers_side_1 = 4;
  num_fingers_side_2 = 3;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
}

TEST_F(MultiFingerTest, GetCuttingPlaneEvenOddDiffDist)
{
  ResetVariables();
  num_fingers_side_1 = 4;
  num_fingers_side_2 = 3;
  ASSERT_NO_THROW(LoadGripper());
  gripper->getCenterCuttingPlane(object);
}
