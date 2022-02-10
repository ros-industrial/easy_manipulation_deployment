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
#include "suction_gripper_test.hpp"

SuctionGripperTest::SuctionGripperTest()
//: viewer(new pcl::visualization::PCLVisualizer("Cloud viewer"))
{
  reset_variables();
}
void SuctionGripperTest::reset_variables()
{
  id = "test_gripper";
  num_cups_length = 1;
  num_cups_breadth = 1;
  dist_between_cups_length = 0.06;
  dist_between_cups_breadth = 0.06;
  cup_radius = 0.005;
  cup_height = 0.01;
  num_sample_along_axis = 3;
  search_resolution = 0.01;
  search_angle_resolution = 4;
  cloud_normal_radius = 0.03;
  curvature_weight = 1.0;
  grasp_center_distance_weight = 1.0;
  num_contact_points_weight = 1.0;
  length_direction = "x";
  breadth_direction = "y";
  grasp_approach_direction = "z";
}

void SuctionGripperTest::LoadGripperWithWeights()
{
  SuctionGripperTestFixture gripper_(
    id,
    num_cups_length,
    num_cups_breadth,
    dist_between_cups_length,
    dist_between_cups_breadth,
    cup_radius,
    cup_height,
    num_sample_along_axis,
    search_resolution,
    search_angle_resolution,
    cloud_normal_radius,
    curvature_weight,
    grasp_center_distance_weight,
    num_contact_points_weight,
    length_direction,
    breadth_direction,
    grasp_approach_direction);
  gripper = std::make_shared<SuctionGripperTestFixture>(gripper_);
}

GraspObject SuctionGripperTest::GenerateObjectHorizontal()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rectangle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  float length = 0.05;
  float breadth = 0.03;
  float height = 0.01;

  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.0; height_ < height; height_ += 0.0025) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = length_;
        temp_point.y = breadth_;
        temp_point.z = height_;
        rectangle_cloud->points.push_back(temp_point);
      }
    }
  }
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*rectangle_cloud, centroid);
  GraspObject object("camera_frame", rectangle_cloud, centroid);
  // object = std::make_shared<GraspObject>(object_);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();
  object.get_object_world_angles();
  return object;
}

GraspObject SuctionGripperTest::GenerateObjectVertical()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rectangle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // float length = 0.05;
  // float breadth = 0.03;
  // float height = 0.02;


  // float length = 0.01;
  // float breadth = 0.03;
  // float height = 0.06;

  float length = 0.06;
  float breadth = 0.01;
  float height = 0.03;

  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.0; height_ < height; height_ += 0.0025) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = length_;
        temp_point.y = breadth_;
        temp_point.z = height_;
        rectangle_cloud->points.push_back(temp_point);
      }
    }
  }
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*rectangle_cloud, centroid);
  GraspObject object("camera_frame", rectangle_cloud, centroid);
  // object = std::make_shared<GraspObject>(object_);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();
  object.get_object_world_angles();
  return object;
}

GraspObject SuctionGripperTest::CreateSphereCloud(
  Eigen::Vector3f & centerpoint,
  const float & radius, const int & resolution,
  const float & x_scale, const float & y_scale, const float & z_scale)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_sphere_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  float px, py, pz;
  for (float phi = 0; phi < M_PI; phi += M_PI / resolution) {
    pz = z_scale * radius * cos(phi);
    for (float theta = 0; theta < 2 * M_PI; theta += 2 * M_PI / resolution) {
      px = x_scale * radius * sin(phi) * cos(theta) + centerpoint(0);
      py = y_scale * radius * sin(phi) * sin(theta) + centerpoint(1);
      pcl::PointXYZRGB point;
      point.x = px;
      point.y = py;
      point.z = pz + centerpoint(2);
      point.r = 255;
      output_sphere_cloud->points.push_back(point);
    }
  }
  // output_sphere_cloud->is_dense = true;
  // output_sphere_cloud->height = 1;
  // output_sphere_cloud->width = output_sphere_cloud->points.size();

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*output_sphere_cloud, centroid);
  GraspObject object("camera_frame", output_sphere_cloud, centroid);
  // object = std::make_shared<GraspObject>(object_);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();
  object.get_object_world_angles();
  return object;
}

void SuctionGripperTest::GenerateObjectCollision(float length, float breadth, float height)
{
  grasp_planner::collision::Box * collision_object_shape =
    new grasp_planner::collision::Box(length, breadth, height);

  grasp_planner::collision::Transform collision_object_transform;
  collision_object_transform.setIdentity();

  #if FCL_VERSION_0_6_OR_HIGHER == 1
  collision_object_transform.translation() << 0, 0, 0;
  grasp_planner::collision::CollisionObject collision_object(
    std::shared_ptr<grasp_planner::collision::CollisionGeometry>(
      collision_object_shape), fcl::Transform3<float>::Identity());
  collision_object_ptr =
    std::make_shared<grasp_planner::collision::CollisionObject>(collision_object);
  #else
  collision_object_transform.setTranslation(
    grasp_planner::collision::Vector(0, 0, 0));
  grasp_planner::collision::CollisionObject collision_object(
    std::shared_ptr<grasp_planner::collision::CollisionGeometry>(
      collision_object_shape), collision_object_transform);
  collision_object_ptr =
    std::make_shared<grasp_planner::collision::CollisionObject>(collision_object);
  #endif
}

TEST_F(SuctionGripperTest, NegAttributes)
{
  reset_variables();
  num_cups_length = -1;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);

  reset_variables();
  num_cups_breadth = -1;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);

  reset_variables();
  cup_radius = -1;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);

  reset_variables();
  cup_height = -1;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);

  reset_variables();
  num_sample_along_axis = -1;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);

  reset_variables();
  dist_between_cups_length = -1;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);

  reset_variables();
  dist_between_cups_breadth = -1;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);

  reset_variables();
  search_resolution = -1;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);

  reset_variables();
  search_angle_resolution = -1;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);
}

TEST_F(SuctionGripperTest, InvalidWeights)
{
  reset_variables();
  curvature_weight = -0.8;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);
  reset_variables();
  grasp_center_distance_weight = -0.9;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);
  num_contact_points_weight = -1.5;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);

  reset_variables();
  curvature_weight = 1.1;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);
  reset_variables();
  grasp_center_distance_weight = 2.4;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);
  num_contact_points_weight = 3.3;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);
}

TEST_F(SuctionGripperTest, SizeCollision)
{
  reset_variables();
  cup_radius = 0.03;
  num_cups_length = 5;
  dist_between_cups_length = 0.02;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);

  reset_variables();
  cup_radius = 0.05;
  num_cups_breadth = 3;
  dist_between_cups_breadth = 0.03;
  EXPECT_THROW(LoadGripperWithWeights(), std::invalid_argument);
}

TEST_F(SuctionGripperTest, NoThrowTest)
{
  reset_variables();
  num_cups_length = 2;
  num_cups_breadth = 3;
  dist_between_cups_length = 0.06;
  dist_between_cups_breadth = 0.04;
  cup_radius = 0.005;
  EXPECT_NO_THROW(LoadGripperWithWeights());
  gripper->update_gripper_attributes();
  EXPECT_NEAR(0.06, gripper->length_dim_public, 0.0001);
  EXPECT_NEAR(0.08, gripper->breadth_dim_public, 0.0001);
}

TEST_F(SuctionGripperTest, generateGripperAttributesTestOddOdd)
{
  reset_variables();
  num_cups_length = 5;
  num_cups_breadth = 3;
  dist_between_cups_length = 0.02;
  dist_between_cups_breadth = 0.01;
  cup_radius = 0.005;
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  gripper->update_gripper_attributes();
  EXPECT_EQ(3, gripper->col_itr_public);
  EXPECT_FALSE(gripper->col_is_even_public);
  EXPECT_NEAR(0.02, gripper->col_dist_between_cups_public, 0.00001);
  EXPECT_NEAR(0.02, gripper->col_initial_gap_public, 0.00001);

  EXPECT_EQ(2, gripper->row_itr_public);
  EXPECT_FALSE(gripper->row_is_even_public);
  EXPECT_NEAR(0.01, gripper->row_dist_between_cups_public, 0.00001);
  EXPECT_NEAR(0.01, gripper->row_initial_gap_public, 0.00001);
}

TEST_F(SuctionGripperTest, generateGripperAttributesTestOddEven)
{
  reset_variables();
  num_cups_length = 3;
  num_cups_breadth = 2;
  dist_between_cups_length = 0.01;
  dist_between_cups_breadth = 0.05;
  cup_radius = 0.005;
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  gripper->update_gripper_attributes();
  EXPECT_EQ(2, gripper->col_itr_public);
  EXPECT_TRUE(gripper->col_is_even_public);
  EXPECT_NEAR(0.05, gripper->col_dist_between_cups_public, 0.00001);
  EXPECT_NEAR(0.025, gripper->col_initial_gap_public, 0.00001);

  EXPECT_EQ(2, gripper->row_itr_public);
  EXPECT_FALSE(gripper->row_is_even_public);
  EXPECT_NEAR(0.01, gripper->row_dist_between_cups_public, 0.00001);
  EXPECT_NEAR(0.01, gripper->row_initial_gap_public, 0.00001);
}

TEST_F(SuctionGripperTest, generateGripperAttributesTestEvenOdd)
{
  reset_variables();
  num_cups_length = 4;
  num_cups_breadth = 1;
  dist_between_cups_length = 0.01;
  dist_between_cups_breadth = 0.03;
  cup_radius = 0.005;
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  gripper->update_gripper_attributes();
  EXPECT_EQ(3, gripper->col_itr_public);
  EXPECT_TRUE(gripper->col_is_even_public);
  EXPECT_NEAR(0.01, gripper->col_dist_between_cups_public, 0.00001);
  EXPECT_NEAR(0.005, gripper->col_initial_gap_public, 0.00001);

  EXPECT_EQ(1, gripper->row_itr_public);
  EXPECT_FALSE(gripper->row_is_even_public);
  EXPECT_NEAR(0.03, gripper->row_dist_between_cups_public, 0.00001);
  EXPECT_NEAR(0.03, gripper->row_initial_gap_public, 0.00001);
}

TEST_F(SuctionGripperTest, generateGripperAttributesTestEvenEven)
{
  reset_variables();
  num_cups_length = 2;
  num_cups_breadth = 6;
  dist_between_cups_length = 0.05;
  dist_between_cups_breadth = 0.02;
  cup_radius = 0.005;
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  gripper->update_gripper_attributes();
  EXPECT_EQ(4, gripper->col_itr_public);
  EXPECT_TRUE(gripper->col_is_even_public);
  EXPECT_NEAR(0.02, gripper->col_dist_between_cups_public, 0.00001);
  EXPECT_NEAR(0.01, gripper->col_initial_gap_public, 0.00001);

  EXPECT_EQ(2, gripper->row_itr_public);
  EXPECT_TRUE(gripper->row_is_even_public);
  EXPECT_NEAR(0.05, gripper->row_dist_between_cups_public, 0.00001);
  EXPECT_NEAR(0.025, gripper->row_initial_gap_public, 0.00001);
}

TEST_F(SuctionGripperTest, findHighestPointTest)
{
  pcl::PointXYZRGB high_point;
  high_point.x = 0.025;
  high_point.y = 0.005;
  high_point.z = 0.03;
  reset_variables();
  GraspObject object = GenerateObjectHorizontal();
  object.cloud->points.push_back(high_point);
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  gripper->update_gripper_attributes();
  pcl::PointXYZRGB object_top_point = gripper->find_highest_point_public(object.cloud, 'z', true);
  EXPECT_NEAR(0.025, object_top_point.x, 0.00001);
  EXPECT_NEAR(0.005, object_top_point.y, 0.00001);
  EXPECT_NEAR(0.03, object_top_point.z, 0.00001);
}

TEST_F(SuctionGripperTest, getStartingPlaneTest)
{
  reset_variables();
  GraspObject object = GenerateObjectHorizontal();
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  pcl::PointXYZRGB object_top_point = gripper->find_highest_point_public(object.cloud, 'z', true);
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  gripper->get_starting_plane_public(
    plane, object.minor_axis,
    object.centerpoint, object_top_point, 'z');
  Eigen::Vector3f normal_vector{
    plane->values[0],
    plane->values[1],
    plane->values[2]};
  float result = object.centerpoint(0) * plane->values[0] +
    object.centerpoint(1) * plane->values[1] +
    object_top_point.z * plane->values[2] + plane->values[3];
  EXPECT_NEAR(0, result, 0.00001);
  ASSERT_NEAR(0, normal_vector.dot(object.axis), 0.0001);
  ASSERT_NEAR(0, normal_vector.dot(object.grasp_axis), 0.0001);
  Eigen::Vector3f cross_pdt = normal_vector.cross(object.minor_axis);
  ASSERT_NEAR(0, cross_pdt(0), 0.0001);
  ASSERT_NEAR(0, cross_pdt(1), 0.0001);
  ASSERT_NEAR(0, cross_pdt(2), 0.0001);
}
TEST_F(SuctionGripperTest, getSlicedCloudTest)
{
  reset_variables();
  GraspObject object = GenerateObjectHorizontal();
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  pcl::PointXYZRGB object_top_point = gripper->find_highest_point_public(object.cloud, 'z', true);
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  gripper->get_starting_plane_public(
    plane, object.minor_axis,
    object.centerpoint, object_top_point, 'z');
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal(
    new pcl::PointCloud<pcl::PointNormal>);
  gripper->get_sliced_cloud_public(
    object.cloud, object.cloud_normal, 0, 0, sliced_cloud,
    sliced_cloud_normal, 'z');
  EXPECT_EQ(
    ((0.05 / 0.0025)) * ((0.03 / 0.0025) + 1),
    static_cast<int>(sliced_cloud->points.size()));
  EXPECT_EQ(
    (0.05 / 0.0025) * (0.03 / 0.0025 + 1),
    static_cast<int>(sliced_cloud_normal->points.size()));
}
TEST_F(SuctionGripperTest, projectCloudToPlaneTest)
{
  float radius = 0.05;
  Eigen::Vector3f centerpoint{0.025, 0.0125, 0.05};
  reset_variables();
  GraspObject object = CreateSphereCloud(centerpoint, radius, 50, 0.5, 0.25, 1.0);
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  pcl::PointXYZRGB object_top_point = gripper->find_highest_point_public(object.cloud, 'z', true);
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  gripper->get_starting_plane_public(
    plane, object.axis,
    object.centerpoint, object_top_point, 'z');
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal(
    new pcl::PointCloud<pcl::PointNormal>);
  gripper->get_sliced_cloud_public(
    object.cloud, object.cloud_normal, object_top_point.z / 2, 0, sliced_cloud,
    sliced_cloud_normal, 'z');

  // EXPECT_EQ(
  //   static_cast<int>(object.cloud->points.size() / 2),
  //   static_cast<int>(sliced_cloud->points.size()));
  // EXPECT_EQ(
  //   static_cast<int>(object.cloud->points.size() / 2),
  //   static_cast<int>(sliced_cloud_normal->points.size()));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  gripper->project_cloud_to_plane_public(sliced_cloud, plane, projected_cloud);
  EXPECT_EQ(
    static_cast<int>(projected_cloud->points.size()),
    static_cast<int>(sliced_cloud->points.size()));

  for (auto point : projected_cloud->points) {
    EXPECT_NEAR(point.z, object_top_point.z, 0.0001);
  }
}

TEST_F(SuctionGripperTest, getCentroidIndexTest)
{
  Eigen::Vector3f centerpoint{0.025, 0.0125, 0.05};
  reset_variables();
  GraspObject object = GenerateObjectHorizontal();
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  pcl::PointXYZRGB object_top_point = gripper->find_highest_point_public(object.cloud, 'z', true);
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  gripper->get_starting_plane_public(
    plane, object.minor_axis,
    object.centerpoint, object_top_point, 'z');
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal(
    new pcl::PointCloud<pcl::PointNormal>);
  gripper->get_sliced_cloud_public(
    object.cloud, object.cloud_normal, 0, 0, sliced_cloud,
    sliced_cloud_normal, 'z');

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  gripper->project_cloud_to_plane_public(sliced_cloud, plane, projected_cloud);
  int centroid_index = gripper->get_centroid_index_public(projected_cloud);
  EXPECT_EQ(
    (0.05 / 0.0025) * (0.03 / 0.0025 + 1),
    static_cast<int>(sliced_cloud_normal->points.size()));
  EXPECT_NEAR(0.05 / 2, projected_cloud->points[centroid_index].x, 0.0001);
  EXPECT_NEAR(0.03 / 2, projected_cloud->points[centroid_index].y, 0.0001);
  EXPECT_NEAR(object_top_point.z, projected_cloud->points[centroid_index].z, 0.0001);
}

TEST_F(SuctionGripperTest, getGripperCenterTest)
{
  Eigen::Vector3f centerpoint{0.025, 0.0125, 0.05};
  reset_variables();
  GraspObject object = GenerateObjectHorizontal();
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  pcl::PointXYZRGB object_top_point = gripper->find_highest_point_public(object.cloud, 'z', true);
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  gripper->get_starting_plane_public(
    plane, object.minor_axis,
    object.centerpoint, object_top_point, 'z');
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal(
    new pcl::PointCloud<pcl::PointNormal>);
  gripper->get_sliced_cloud_public(
    object.cloud, object.cloud_normal, 0, 0, sliced_cloud,
    sliced_cloud_normal, 'z');
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  gripper->project_cloud_to_plane_public(sliced_cloud, plane, projected_cloud);
  int centroid_index = gripper->get_centroid_index_public(projected_cloud);

  //Need to update test to reflect new implementation
  // pcl::PointXYZ sample_gripper_center_x = gripper->get_gripper_center_public(
  //   object.axis,
  //   0.002,
  //   projected_cloud->points[centroid_index]);

  // pcl::PointXYZ sample_gripper_center_y = gripper->get_gripper_center_public(
  //   object.grasp_axis,
  //   0.001,
  //   projected_cloud->points[centroid_index]);

  // pcl::PointXYZ sample_gripper_center_z = gripper->get_gripper_center_public(
  //   object.minor_axis,
  //   0.003,
  //   projected_cloud->points[centroid_index]);

  // EXPECT_NEAR(projected_cloud->points[centroid_index].x + 0.002, sample_gripper_center_x.x, 0.0001);
  // EXPECT_NEAR(projected_cloud->points[centroid_index].y, sample_gripper_center_x.y, 0.0001);
  // EXPECT_NEAR(projected_cloud->points[centroid_index].z, sample_gripper_center_x.z, 0.0001);

  // EXPECT_NEAR(projected_cloud->points[centroid_index].x, sample_gripper_center_y.x, 0.0001);
  // EXPECT_NEAR(projected_cloud->points[centroid_index].y - 0.001, sample_gripper_center_y.y, 0.0001);
  // EXPECT_NEAR(projected_cloud->points[centroid_index].z, sample_gripper_center_y.z, 0.0001);

  // EXPECT_NEAR(projected_cloud->points[centroid_index].x, sample_gripper_center_z.x, 0.0001);
  // EXPECT_NEAR(projected_cloud->points[centroid_index].y, sample_gripper_center_z.y, 0.0001);
  // EXPECT_NEAR(projected_cloud->points[centroid_index].z + 0.003, sample_gripper_center_z.z, 0.0001);
}

// TEST_F(SuctionGripperTest, getGraspDirectionTest)
// {
//     Eigen::Vector3f centerpoint{0.025, 0.0125, 0.05};
//   reset_variables();
//   GraspObject object = GenerateObjectHorizontal();
//   ASSERT_NO_THROW(LoadGripperWithWeights());

//   Eigen::Vector3f grasp_direction = gripper->getGraspDirection(object.grasp_axis, 2);
//   EXPECT_NEAR(0, std::abs(grasp_direction.dot(object.grasp_axis)), 0.0001);

//   Eigen::Vector3f grasp_direction1 = gripper->getGraspDirection(object.grasp_axis, 0.66666);
//   EXPECT_NEAR(0, std::abs(grasp_direction1.dot(object.grasp_axis)), 0.0001);

//   Eigen::Vector3f grasp_direction2 = gripper->getGraspDirection(object.grasp_axis, 1);
//   EXPECT_TRUE(std::abs(grasp_direction2.dot(object.grasp_axis)) > 0);

//   Eigen::Vector3f grasp_direction3 = gripper->getGraspDirection(object.grasp_axis, 0.5);
//   EXPECT_TRUE(std::abs(grasp_direction3.dot(object.grasp_axis)) > 0);
// }

// TEST_F(SuctionGripperTest, getObjectDirectionTest)
// {
//     Eigen::Vector3f centerpoint{0.025, 0.0125, 0.05};
//   reset_variables();
//   GraspObject object = GenerateObjectHorizontal();
//   ASSERT_NO_THROW(LoadGripperWithWeights());

//   Eigen::Vector3f grasp_direction = gripper->getObjectDirection(object.axis, 2);
//   EXPECT_NEAR(0, std::abs(grasp_direction.dot(object.axis)), 0.0001);

//   Eigen::Vector3f grasp_direction1 = gripper->getObjectDirection(object.axis, 0.66666);
//   EXPECT_NEAR(0, std::abs(grasp_direction1.dot(object.axis)), 0.0001);

//   Eigen::Vector3f grasp_direction2 = gripper->getObjectDirection(object.axis, 1);
//   EXPECT_TRUE(std::abs(grasp_direction2.dot(object.axis)) > 0);

//   Eigen::Vector3f grasp_direction3 = gripper->getObjectDirection(object.axis, 0.5);
//   EXPECT_TRUE(std::abs(grasp_direction3.dot(object.axis)) > 0);
// }

TEST_F(SuctionGripperTest, getGapTestEvenOdd) {
  reset_variables();
  num_cups_length = 5;
  num_cups_breadth = 4;
  dist_between_cups_length = 0.02;
  dist_between_cups_breadth = 0.01;
  cup_radius = 0.005;
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  gripper->update_gripper_attributes();
  float col_gap = gripper->get_gap_public(
    2, gripper->col_is_even_public,
    gripper->col_initial_gap_public, gripper->col_dist_between_cups_public);
  EXPECT_NEAR(0.04, col_gap, 0.0001);
  gripper->update_gripper_attributes();
  float row_gap = gripper->get_gap_public(
    1, gripper->row_is_even_public,
    gripper->row_initial_gap_public, gripper->row_dist_between_cups_public);
  EXPECT_NEAR(0.005, row_gap, 0.0001);
}

TEST_F(SuctionGripperTest, getGapTestOddEven) {
  reset_variables();
  num_cups_length = 4;
  num_cups_breadth = 5;
  dist_between_cups_length = 0.02;
  dist_between_cups_breadth = 0.01;
  cup_radius = 0.005;
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  gripper->update_gripper_attributes();
  float col_gap = gripper->get_gap_public(
    1, gripper->col_is_even_public,
    gripper->col_initial_gap_public, gripper->col_dist_between_cups_public);
  EXPECT_NEAR(0.01, col_gap, 0.0001);
  gripper->update_gripper_attributes();
  float row_gap = gripper->get_gap_public(
    2, gripper->row_is_even_public,
    gripper->row_initial_gap_public, gripper->row_dist_between_cups_public);
  EXPECT_NEAR(0.02, row_gap, 0.0001);
}


TEST_F(SuctionGripperTest, getGapTestOddOdd) {
  reset_variables();
  num_cups_length = 1;
  num_cups_breadth = 3;
  dist_between_cups_length = 0.02;
  dist_between_cups_breadth = 0.01;
  cup_radius = 0.005;
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  gripper->update_gripper_attributes();
  float col_gap = gripper->get_gap_public(
    1, gripper->col_is_even_public,
    gripper->col_initial_gap_public, gripper->col_dist_between_cups_public);
  EXPECT_NEAR(0.01, col_gap, 0.0001);
  gripper->update_gripper_attributes();
  float row_gap = gripper->get_gap_public(
    0, gripper->row_is_even_public,
    gripper->row_initial_gap_public, gripper->row_dist_between_cups_public);
  EXPECT_NEAR(0, row_gap, 0.0001);
}

TEST_F(SuctionGripperTest, getGapTestEvenEven) {
  reset_variables();
  num_cups_length = 4;
  num_cups_breadth = 2;
  dist_between_cups_length = 0.02;
  dist_between_cups_breadth = 0.01;
  cup_radius = 0.005;
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  gripper->update_gripper_attributes();
  float col_gap = gripper->get_gap_public(
    2, gripper->col_is_even_public,
    gripper->col_initial_gap_public, gripper->col_dist_between_cups_public);
  EXPECT_NEAR(0.03, col_gap, 0.0001);
  gripper->update_gripper_attributes();
  float row_gap = gripper->get_gap_public(
    1, gripper->row_is_even_public,
    gripper->row_initial_gap_public, gripper->row_dist_between_cups_public);
  EXPECT_NEAR(0.005, row_gap, 0.0001);
}

TEST_F(SuctionGripperTest, getContactPointsTest) {

  Eigen::Vector3f centerpoint{0.025, 0.0125, 0.05};
  reset_variables();
  GraspObject object = GenerateObjectHorizontal();
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  pcl::PointXYZRGB object_top_point = gripper->find_highest_point_public(object.cloud, 'z', true);
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  gripper->get_starting_plane_public(
    plane, object.minor_axis,
    object.centerpoint, object_top_point, 'z');
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal(
    new pcl::PointCloud<pcl::PointNormal>);
  gripper->get_sliced_cloud_public(
    object.cloud, object.cloud_normal, 0, 0, sliced_cloud,
    sliced_cloud_normal, 'z');
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  gripper->project_cloud_to_plane_public(sliced_cloud, plane, projected_cloud);

  float curvature_sum;
  // float * curvature_sum_ptr;
  // curvature_sum_ptr = &curvature_sum;

  pcl::PointXYZ full_cup_point;
  full_cup_point.x = 0.025;
  full_cup_point.y = 0.015;
  full_cup_point.z = 0.01;

  pcl::PointXYZ half_cup_point;
  half_cup_point.x = 0.025;
  half_cup_point.y = 0.03;
  half_cup_point.z = 0.01;

  pcl::PointXYZ no_cup_point;
  no_cup_point.x = 0.06;
  no_cup_point.y = 0.05;
  no_cup_point.z = 0.04;

  int full_contact_points = gripper->get_contact_points_public(
    projected_cloud, sliced_cloud_normal, full_cup_point, curvature_sum);
  int half_contact_points = gripper->get_contact_points_public(
    projected_cloud, sliced_cloud_normal, half_cup_point, curvature_sum);
  int no_contact_points = gripper->get_contact_points_public(
    projected_cloud, sliced_cloud_normal, no_cup_point, curvature_sum);

  EXPECT_TRUE(full_contact_points > half_contact_points);
  EXPECT_TRUE(full_contact_points > 0);
  EXPECT_TRUE(half_contact_points > 0);
  EXPECT_EQ(0, no_contact_points);

  int result_weighted = gripper->generate_weighted_contact_points_public(
    10,
    0.001,
    0.3);
  EXPECT_NEAR(10, result_weighted, 0.0001);

}

TEST_F(SuctionGripperTest, generateSuctionCupTest) {
  reset_variables();
  float radius = 0.08;
  Eigen::Vector3f centerpoint{0.04, 0.0125, 0.08};
  num_cups_length = 2;
  num_cups_breadth = 6;
  dist_between_cups_length = 0.05;
  dist_between_cups_breadth = 0.02;
  cup_radius = 0.005;
  GraspObject object = CreateSphereCloud(centerpoint, radius, 50, 0.5, 0.25, 1.0);
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  pcl::PointXYZRGB object_top_point = gripper->find_highest_point_public(object.cloud, 'z', true);
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  gripper->get_starting_plane_public(
    plane, object.axis,
    object.centerpoint, object_top_point, 'z');
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal(
    new pcl::PointCloud<pcl::PointNormal>);
  gripper->get_sliced_cloud_public(
    object.cloud, object.cloud_normal, object_top_point.z / 2, 0, sliced_cloud,
    sliced_cloud_normal, 'z');

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  gripper->project_cloud_to_plane_public(sliced_cloud, plane, projected_cloud);

  pcl::PointXYZ object_center;
  object_center.x = object.centerpoint(0);
  object_center.y = object.centerpoint(1);
  object_center.z = object.centerpoint(2);
  float object_max_dim =
    *std::max_element(std::begin(object.dimensions), std::end(object.dimensions));

  Eigen::Vector3f full_cup_point{object_top_point.x, object_top_point.y, object_top_point.z};
  Eigen::Vector3f half_cup_point{object_top_point.x + static_cast<float>(0.031),
    object_top_point.y + static_cast<float>(0.011), object_top_point.z};
  Eigen::Vector3f no_cup_point{0.3, 0.3, 0.08};

  SingleSuctionCup cup_full = gripper->generate_suction_cup_public(
    projected_cloud, sliced_cloud_normal,
    full_cup_point, object_center, object_max_dim);

  SingleSuctionCup cup_half = gripper->generate_suction_cup_public(
    projected_cloud, sliced_cloud_normal,
    half_cup_point, object_center, object_max_dim);

  SingleSuctionCup cup_none = gripper->generate_suction_cup_public(
    projected_cloud, sliced_cloud_normal,
    no_cup_point, object_center, object_max_dim);

  EXPECT_NEAR(object_top_point.x, cup_full.cup_center.x, 0.0001);
  EXPECT_NEAR(object_top_point.y, cup_full.cup_center.y, 0.0001);
  EXPECT_NEAR(object_top_point.z, cup_full.cup_center.z, 0.0001);

  EXPECT_NEAR(object_top_point.x + 0.031, cup_half.cup_center.x, 0.0001);
  EXPECT_NEAR(object_top_point.y + 0.011, cup_half.cup_center.y, 0.0001);
  EXPECT_NEAR(object_top_point.z, cup_half.cup_center.z, 0.0001);

  EXPECT_GT(cup_full.contact_points, cup_half.contact_points);
  EXPECT_GT(cup_full.contact_points, 0);
  EXPECT_GT(cup_half.contact_points, 0);
  EXPECT_EQ(cup_none.contact_points, 0);
}

// Temporary comment for testing of suction gripper
// TEST_F(SuctionGripperTest, generate_grasp_sample_publicTest) {
//   reset_variables();
//   float radius = 0.08;
//   // Eigen::Vector3f centerpoint{0.04, 0.0125, 0.08};
//   // qEigen::Vector3f centerpoint{0.08, 0.04, 0.0125};
//   Eigen::Vector3f centerpoint{0.0125, 0.08, 0.04};
//   num_cups_length = 2;
//   num_cups_breadth = 6;
//   dist_between_cups_length = 0.05;
//   dist_between_cups_breadth = 0.02;
//   cup_radius = 0.005;
//   GraspObject object = CreateSphereCloud(centerpoint, radius, 50, 0.25, 1, 0.5);
//   ASSERT_NO_THROW(LoadGripperWithWeights());
//   gripper->generate_gripper_attributes();
//   pcl::PointXYZRGB object_top_point = gripper->find_highest_point_public(
//     object.cloud,
//     object.alignments[2], true);
//   pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
//   gripper->get_starting_plane_public(
//     plane, object.minor_axis,
//     object.centerpoint, object_top_point, object.alignments[2]);
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//   pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal(
//     new pcl::PointCloud<pcl::PointNormal>);
//   gripper->get_sliced_cloud_public(
//     object.cloud, object.cloud_normal, object_top_point.z / 2, 0, sliced_cloud,
//     sliced_cloud_normal, object.alignments[2]);

//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//   gripper->project_cloud_to_plane_public(sliced_cloud, plane, projected_cloud);
//   int centroid_index = gripper->get_centroid_index_public(projected_cloud);
//   pcl::PointXYZ sample_gripper_center = gripper->get_gripper_center_public(
//     object.axis,
//     0,
//     projected_cloud->points[centroid_index]);

//   Eigen::Vector3f grasp_direction = object.grasp_axis;
//   Eigen::Vector3f object_direction = object.axis;

//   pcl::PointXYZ object_center;
//   object_center.x = object.centerpoint(0);
//   object_center.y = object.centerpoint(1);
//   object_center.z = object.centerpoint(2);
//   float object_max_dim =
//     *std::max_element(std::begin(object.dimensions), std::end(object.dimensions));

//   SuctionCupArray grasp_sample = gripper->generate_grasp_sample_public(
//     projected_cloud,
//     sliced_cloud_normal,
//     sample_gripper_center,
//     object_center,
//     grasp_direction,
//     object_direction,
//     object_max_dim,
//     camera_frame);

//   ASSERT_EQ(2, static_cast<int>(grasp_sample.cup_array.size()));
//   ASSERT_EQ(6, static_cast<int>(grasp_sample.cup_array[0].size()));
//   ASSERT_EQ(6, static_cast<int>(grasp_sample.cup_array[1].size()));
//   float z_vals[2];
//   for (size_t i = 0; i < grasp_sample.cup_array.size(); i++) {
//     EXPECT_EQ(6, static_cast<int>(grasp_sample.cup_array[i].size()));
//     std::vector<float> sorted_z_vals;
//     std::vector<float> sorted_y_vals;
//     for (size_t j = 0; j < grasp_sample.cup_array[i].size(); j++) {
//       sorted_z_vals.push_back(grasp_sample.cup_array[i][j]->cup_center.z);
//       z_vals[i] = grasp_sample.cup_array[i][j]->cup_center.z;
//       sorted_y_vals.push_back(grasp_sample.cup_array[i][j]->cup_center.y);
//     }
//     std::sort(sorted_z_vals.begin(), sorted_z_vals.end());
//     std::sort(sorted_y_vals.begin(), sorted_y_vals.end());

//     for (size_t k = 0; k < sorted_z_vals.size(); k++) {
//       if (k != 0) {
//         EXPECT_NEAR(sorted_z_vals[k], sorted_z_vals[k - 1], 0.0001);
//       }
//     }

//     for (size_t l = 0; l < sorted_y_vals.size(); l++) {
//       if (l != 0) {
//         EXPECT_NEAR(0.02, std::abs(sorted_y_vals[l] - sorted_y_vals[l - 1]), 0.0001);
//       }
//     }
//   }

//   EXPECT_NEAR(0.05, std::abs(z_vals[0] - z_vals[1]), 0.0001);
// }

TEST_F(SuctionGripperTest, updateMaxMinValuesTest) {

  reset_variables();
  Eigen::Vector3f centerpoint{0.0125, 0.08, 0.04};
  GraspObject object = CreateSphereCloud(centerpoint, 0.08, 50, 0.25, 1, 0.5);
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();

  gripper->update_max_min_values_public(45, 0.004, 0.002);
  gripper->update_gripper_attributes();
  EXPECT_NEAR(0.002, gripper->max_center_dist_public, 0.00001);
  EXPECT_NEAR(0.002, gripper->min_center_dist_public, 0.00001);
  EXPECT_NEAR(0.004, gripper->max_curvature_public, 0.00001);
  EXPECT_NEAR(0.004, gripper->min_curvature_public, 0.00001);
  EXPECT_NEAR(45, gripper->max_contact_points_public, 0.00001);
  EXPECT_NEAR(45, gripper->min_contact_points_public, 0.00001);

  gripper->update_max_min_values_public(15, 0.001, 0.0005);
  gripper->update_gripper_attributes();
  EXPECT_NEAR(0.002, gripper->max_center_dist_public, 0.00001);
  EXPECT_NEAR(0.0005, gripper->min_center_dist_public, 0.00001);
  EXPECT_NEAR(0.004, gripper->max_curvature_public, 0.00001);
  EXPECT_NEAR(0.001, gripper->min_curvature_public, 0.00001);
  EXPECT_NEAR(45, gripper->max_contact_points_public, 0.00001);
  EXPECT_NEAR(15, gripper->min_contact_points_public, 0.00001);

  gripper->update_max_min_values_public(55, 0.007, 0.006);
  gripper->update_gripper_attributes();
  EXPECT_NEAR(0.006, gripper->max_center_dist_public, 0.00001);
  EXPECT_NEAR(0.0005, gripper->min_center_dist_public, 0.00001);
  EXPECT_NEAR(0.007, gripper->max_curvature_public, 0.00001);
  EXPECT_NEAR(0.001, gripper->min_curvature_public, 0.00001);
  EXPECT_NEAR(55, gripper->max_contact_points_public, 0.00001);
  EXPECT_NEAR(15, gripper->min_contact_points_public, 0.00001);
}

TEST_F(SuctionGripperTest, getAllPossibleGraspsTest) {

  reset_variables();
  Eigen::Vector3f centerpoint{0.0125, 0.08, 0.04};
  num_cups_length = 2;
  num_cups_breadth = 6;
  dist_between_cups_length = 0.05;
  dist_between_cups_breadth = 0.02;
  cup_radius = 0.005;
  GraspObject object = CreateSphereCloud(centerpoint, 0.08, 50, 0.25, 1, 0.5);
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  pcl::PointXYZ object_center;
  object_center.x = object.centerpoint(0);
  object_center.y = object.centerpoint(1);
  object_center.z = object.centerpoint(2);
  pcl::PointXYZRGB object_top_point = gripper->find_highest_point_public(
    object.cloud,
    object.alignments[2], true);
  EXPECT_EQ(0, static_cast<int>(gripper->cup_array_samples_public.size()));
  gripper->get_all_possible_grasps_public(object, object_center, object_top_point, camera_frame);
  gripper->update_gripper_attributes();
  EXPECT_GT(static_cast<int>(gripper->cup_array_samples_public.size()), 0);
  for (auto grasp_sample : gripper->cup_array_samples_public) {
    EXPECT_NEAR(0, grasp_sample->rank, 0.0001);
  }
}

TEST_F(SuctionGripperTest, getAllRanksTest) {
  reset_variables();
  Eigen::Vector3f centerpoint{0.0125, 0.08, 0.04};
  num_cups_length = 2;
  num_cups_breadth = 6;
  dist_between_cups_length = 0.05;
  dist_between_cups_breadth = 0.02;
  cup_radius = 0.005;
  GraspObject object = CreateSphereCloud(centerpoint, 0.08, 50, 0.25, 1, 0.5);
  ASSERT_NO_THROW(LoadGripperWithWeights());

  pcl::PointXYZ object_center; gripper->generate_gripper_attributes();
  object_center.x = object.centerpoint(0);
  object_center.y = object.centerpoint(1);
  object_center.z = object.centerpoint(2);
  pcl::PointXYZRGB object_top_point = gripper->find_highest_point_public(
    object.cloud,
    object.alignments[2], true);
  gripper->get_all_possible_grasps_public(object, object_center, object_top_point, camera_frame);
  gripper->update_gripper_attributes();
  EXPECT_GT(static_cast<int>(gripper->cup_array_samples_public.size()), 0);
  for (auto grasp_sample : gripper->cup_array_samples_public) {
    EXPECT_NEAR(0, grasp_sample->rank, 0.0001);
  }
  emd_msgs::msg::GraspMethod grasp_method;
  gripper->get_all_grasp_ranks_public(grasp_method, object);
  gripper->update_gripper_attributes();
  for (auto grasp_sample : gripper->cup_array_samples_public) {
    EXPECT_GT(grasp_sample->rank, 0);
  }

  for (size_t i = 0; i < grasp_method.grasp_ranks.size(); i++) {
    if (i != 0) {
      EXPECT_GE(grasp_method.grasp_ranks[i - 1], grasp_method.grasp_ranks[i]);
    }
  }
}

TEST_F(SuctionGripperTest, getGraspPoseTest) {
  reset_variables();
  Eigen::Vector3f centerpoint{0.0125, 0.08, 0.04};
  num_cups_length = 2;
  num_cups_breadth = 3;
  dist_between_cups_length = 0.01;
  dist_between_cups_breadth = 0.03;
  cup_radius = 0.005;
  GraspObject object = CreateSphereCloud(centerpoint, 0.08, 50, 0.25, 1, 0.5);
  ASSERT_NO_THROW(LoadGripperWithWeights());

  pcl::PointXYZ object_center; gripper->generate_gripper_attributes();
  object_center.x = object.centerpoint(0);
  object_center.y = object.centerpoint(1);
  object_center.z = object.centerpoint(2);
  pcl::PointXYZRGB object_top_point = gripper->find_highest_point_public(
    object.cloud,
    object.alignments[2], true);
  gripper->get_all_possible_grasps_public(object, object_center, object_top_point, camera_frame);
  gripper->update_gripper_attributes();
  for (auto sample : gripper->cup_array_samples_public) {
    geometry_msgs::msg::PoseStamped grasp_pose = gripper->get_grasp_pose_public(sample, object);
    EXPECT_EQ(0, grasp_pose.pose.orientation.x);
    EXPECT_EQ(0, grasp_pose.pose.orientation.y);
    EXPECT_GE(std::abs(grasp_pose.pose.orientation.z), 0);
    EXPECT_GE(std::abs(grasp_pose.pose.orientation.w), 0);

    EXPECT_EQ(sample->gripper_center.x, grasp_pose.pose.position.x);
    EXPECT_EQ(sample->gripper_center.y, grasp_pose.pose.position.y);
    EXPECT_EQ(sample->gripper_center.z, grasp_pose.pose.position.z);
  }
}

// TEST_F(SuctionGripperTest, get_planar_rpy_publicTestXYZ)
// {
//   reset_variables();
//   num_cups_length = 6;
//   num_cups_breadth = 2;
//   dist_between_cups_length = 0.02;
//   dist_between_cups_breadth = 0.03;
//   cup_radius = 0.005;
//   ASSERT_NO_THROW(LoadGripperWithWeights());
//   gripper->generate_gripper_attributes();

//   std::vector<double> output = gripper->get_planar_rpy_public({1, 0, 0}, {0, 1, 0});
//   EXPECT_NEAR(0.0, static_cast<float>(output[0]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output[1]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output[2]), 0.0001);

//   std::vector<double> output2 = gripper->get_planar_rpy_public({0, 1, 0}, {-1, 0, 0});
//   EXPECT_NEAR(0.0, static_cast<float>(output2[0]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output2[1]), 0.0001);
//   EXPECT_GT(std::abs(static_cast<float>(output2[2])), 0);
//   EXPECT_NEAR(1.5708, std::abs(static_cast<float>(output2[2])), 0.00001);

//   std::vector<double> output3 = gripper->get_planar_rpy_public({0, 0, -1}, {0, 1, 0});
//   EXPECT_NEAR(0.0, static_cast<float>(output3[0]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output3[2]), 0.0001);
//   EXPECT_GT(std::abs(static_cast<float>(output3[1])), 0);
//   EXPECT_NEAR(1.5708, std::abs(static_cast<float>(output3[1])), 0.00001);

//   std::vector<double> output4 = gripper->get_planar_rpy_public({1, 0, 0}, {0, 0, 1});
//   EXPECT_NEAR(0.0, static_cast<float>(output4[1]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output4[2]), 0.0001);
//   EXPECT_GT(std::abs(static_cast<float>(output4[0])), 0);
//   EXPECT_NEAR(1.5708, std::abs(static_cast<float>(output4[0])), 0.00001);
// }

// TEST_F(SuctionGripperTest, get_planar_rpy_publicTestYZX)
// {
//   reset_variables();
//   num_cups_length = 6;
//   num_cups_breadth = 2;
//   dist_between_cups_length = 0.02;
//   dist_between_cups_breadth = 0.03;
//   cup_radius = 0.005;
//   length_direction = "y";
//   breadth_direction = "z";
//   grasp_approach_direction = "x";
//   ASSERT_NO_THROW(LoadGripperWithWeights());
//   gripper->generate_gripper_attributes();

//   std::vector<double> output = gripper->get_planar_rpy_public({0, 1, 0}, {0, 0, 1});
//   EXPECT_NEAR(0.0, static_cast<float>(output[0]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output[1]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output[2]), 0.0001);

//   std::vector<double> output1 = gripper->get_planar_rpy_public({-1, 0, 0}, {0, 0, 1});
//   EXPECT_NEAR(0.0, static_cast<float>(output1[0]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output1[1]), 0.0001);
//   EXPECT_GT(std::abs(static_cast<float>(output1[2])), 0);
//   EXPECT_NEAR(1.5708, std::abs(static_cast<float>(output1[2])), 0.00001);

//   std::vector<double> output2 = gripper->get_planar_rpy_public({0, 1, 0}, {1, 0, 0});
//   EXPECT_NEAR(0.0, static_cast<float>(output2[0]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output2[2]), 0.0001);
//   EXPECT_GT(std::abs(static_cast<float>(output2[1])), 0);
//   EXPECT_NEAR(1.5708, std::abs(static_cast<float>(output2[1])), 0.00001);

//   std::vector<double> output3 = gripper->get_planar_rpy_public({0, 0, 1}, {0, -1, 0});
//   EXPECT_NEAR(0.0, static_cast<float>(output3[1]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output3[2]), 0.0001);
//   EXPECT_GT(std::abs(static_cast<float>(output3[0])), 0);
//   EXPECT_NEAR(1.5708, std::abs(static_cast<float>(output3[0])), 0.00001);
// }

// TEST_F(SuctionGripperTest, get_planar_rpy_publicTestZXY)
// {
//   reset_variables();
//   num_cups_length = 6;
//   num_cups_breadth = 2;
//   dist_between_cups_length = 0.02;
//   dist_between_cups_breadth = 0.03;
//   cup_radius = 0.005;
//   length_direction = "z";
//   breadth_direction = "x";
//   grasp_approach_direction = "y";
//   ASSERT_NO_THROW(LoadGripperWithWeights());
//   gripper->generate_gripper_attributes();

//   std::vector<double> output = gripper->get_planar_rpy_public({0, 0, 1}, {1, 0, 0});
//   EXPECT_NEAR(0.0, static_cast<float>(output[0]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output[1]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output[2]), 0.0001);

//   std::vector<double> output1 = gripper->get_planar_rpy_public({0, 0, 1}, {0, 1, 0});
//   EXPECT_NEAR(0.0, static_cast<float>(output1[0]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output1[1]), 0.0001);
//   EXPECT_GT(std::abs(static_cast<float>(output1[2])), 0);
//   EXPECT_NEAR(1.5708, std::abs(static_cast<float>(output1[2])), 0.00001);

//   std::vector<double> output2 = gripper->get_planar_rpy_public({1, 0, 0}, {0, 0, -1});
//   EXPECT_NEAR(0.0, static_cast<float>(output2[0]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output2[2]), 0.0001);
//   EXPECT_GT(std::abs(static_cast<float>(output2[1])), 0);
//   EXPECT_NEAR(1.5708, std::abs(static_cast<float>(output2[1])), 0.00001);

//   std::vector<double> output3 = gripper->get_planar_rpy_public({0, -1, 0}, {1, 0, 0});
//   EXPECT_NEAR(0.0, static_cast<float>(output3[1]), 0.0001);
//   EXPECT_NEAR(0.0, static_cast<float>(output3[2]), 0.0001);
//   EXPECT_GT(std::abs(static_cast<float>(output3[0])), 0);
//   EXPECT_NEAR(1.5708, std::abs(static_cast<float>(output3[0])), 0.00001);
// }

TEST_F(SuctionGripperTest, planGraspsTest)
{
  // GraspObject object = GenerateObjectVertical();
  reset_variables();
  num_cups_length = 6;
  num_cups_breadth = 2;
  dist_between_cups_length = 0.02;
  dist_between_cups_breadth = 0.03;
  cup_radius = 0.005;
  Eigen::Vector3f centerpoint{0.0125, 0.08, 0.04};
  GraspObject object = CreateSphereCloud(centerpoint, 0.08, 50, 0.25, 1, 0.5);
  ASSERT_NO_THROW(LoadGripperWithWeights());
  gripper->generate_gripper_attributes();
  GenerateObjectCollision(0.06, 0.01, 0.03);

  emd_msgs::msg::GraspMethod grasp_method;
  grasp_method.ee_id = gripper->get_id();
  grasp_method.grasp_ranks.insert(
    grasp_method.grasp_ranks.begin(), std::numeric_limits<float>::min());
  EXPECT_EQ(0, static_cast<int>(grasp_method.grasp_poses.size()));
  EXPECT_EQ(1, static_cast<int>(grasp_method.grasp_ranks.size()));
  gripper->plan_grasps(
    object, grasp_method, collision_object_ptr,
    "camera_frame");
  grasp_method.grasp_ranks.pop_back();
  object.grasp_target.grasp_methods.push_back(grasp_method);

  EXPECT_GT(static_cast<int>(grasp_method.grasp_poses.size()), 0);
  EXPECT_GT(static_cast<int>(grasp_method.grasp_ranks.size()), 0);
}

// pcl::PointXYZRGB centerpoint_;
// centerpoint_.x = object.centerpoint(0);
// centerpoint_.y = object.centerpoint(1);
// centerpoint_.z = object.centerpoint(2);

// pcl::PointXYZRGB axis1;
// axis1.x = object.axis(0);
// axis1.y = object.axis(1);
// axis1.z = object.axis(2);

// pcl::PointXYZRGB axis2;
// axis2.x = object.grasp_axis(0);
// axis2.y = object.grasp_axis(1);
// axis2.z = object.grasp_axis(2);

// pcl::PointXYZRGB axis3;
// axis3.x = object.minor_axis(0);
// axis3.y = object.minor_axis(1);
// axis3.z = object.minor_axis(2);

// Eigen::Vector3f x_axis{1, 0, 0};
// Eigen::Vector3f y_axis{0, 1, 0};
// Eigen::Vector3f z_axis{0, 0, 1};

// pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb(projected_cloud, 255, 0, 0);
// viewer->addPointCloud<pcl::PointXYZRGB>(projected_cloud, rgb, "cloud");
// pcl::PointXYZRGB sphere = projected_cloud->points[centroid_index];
// sphere.x = object.centerpoint(0);
// sphere.y = object.centerpoint(1);
// sphere.z = object_top_point.z;
// viewer->addSphere(sphere, 0.001, "sphere12");
// viewer->addLine(axis1, centerpoint_, 255, 0, 0, "Axis1", 0);
// viewer->addLine(axis2, centerpoint_, 0, 255, 0, "Axis2", 0);
// viewer->addLine(axis3, centerpoint_, 0, 0, 255, "Axis3", 0);
// viewer->addCoordinateSystem(0.1);
// viewer->add_plane(*plane, "plane", 0);
// viewer->spin();
// viewer->close();
// viewer->removeAllPointClouds();
