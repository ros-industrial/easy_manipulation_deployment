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
#include "grasp_object_test.hpp"

GraspObjectTest::GraspObjectTest()
: object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
}

void GraspObjectTest::GenerateObjectCloud(float length, float breadth, float height)
{
  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.0; height_ < height; height_ += 0.0025) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = length_;
        temp_point.y = breadth_;
        temp_point.z = height_;
        object_cloud->points.push_back(temp_point);
      }
    }
  }
}

TEST_F(GraspObjectTest, GraspObjectConstructor)
{
  GenerateObjectCloud(0.05, 0.01, 0.02);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);
  EXPECT_TRUE(object.grasp_target.target_type == "unknown_object");
  EXPECT_NEAR(object.centerpoint(0), centroid(0), 0.0001);
  EXPECT_NEAR(object.centerpoint(1), centroid(1), 0.0001);
  EXPECT_NEAR(object.centerpoint(2), centroid(2), 0.0001);

  GraspObject object_named("object_name", "camera_frame", object_cloud, centroid);
  EXPECT_TRUE(object_named.grasp_target.target_type == "object_name");
  EXPECT_NEAR(object_named.centerpoint(0), centroid(0), 0.0001);
  EXPECT_NEAR(object_named.centerpoint(1), centroid(1), 0.0001);
  EXPECT_NEAR(object_named.centerpoint(2), centroid(2), 0.0001);
}

TEST_F(GraspObjectTest, getObjectBBTestAlignedX)
{
  GenerateObjectCloud(0.05, 0.01, 0.02);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();

  EXPECT_NEAR(0, object.axis.dot(world_y), 0.0001);
  EXPECT_NEAR(0, object.axis.dot(world_z), 0.0001);

  EXPECT_NEAR(0, object.grasp_axis.dot(world_x), 0.0001);
  EXPECT_NEAR(0, object.grasp_axis.dot(world_y), 0.0001);

  EXPECT_NEAR(0, object.minor_axis.dot(world_x), 0.0001);
  EXPECT_NEAR(0, object.minor_axis.dot(world_z), 0.0001);
}

TEST_F(GraspObjectTest, getObjectBBTestAlignedY)
{
  GenerateObjectCloud(0.03, 0.06, 0.01);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();

  EXPECT_NEAR(0, object.axis.dot(world_x), 0.0001);
  EXPECT_NEAR(0, object.axis.dot(world_z), 0.0001);

  EXPECT_NEAR(0, object.grasp_axis.dot(world_y), 0.0001);
  EXPECT_NEAR(0, object.grasp_axis.dot(world_z), 0.0001);

  EXPECT_NEAR(0, object.minor_axis.dot(world_x), 0.0001);
  EXPECT_NEAR(0, object.minor_axis.dot(world_y), 0.0001);
}

TEST_F(GraspObjectTest, getObjectBBTestAlignedZ)
{
  GenerateObjectCloud(0.05, 0.01, 0.07);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();

  EXPECT_NEAR(0, object.axis.dot(world_x), 0.0001);
  EXPECT_NEAR(0, object.axis.dot(world_y), 0.0001);

  EXPECT_NEAR(0, object.grasp_axis.dot(world_y), 0.0001);
  EXPECT_NEAR(0, object.grasp_axis.dot(world_z), 0.0001);

  EXPECT_NEAR(0, object.minor_axis.dot(world_x), 0.0001);
  EXPECT_NEAR(0, object.minor_axis.dot(world_z), 0.0001);
}

TEST_F(GraspObjectTest, getObjectDimensionsTest)
{
  GenerateObjectCloud(0.05, 0.01, 0.07);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();
  object.get_object_dimensions();
  // TODO Glenn: Fix this failing test
  // EXPECT_NEAR(0.0475, object.dimensions[0], 0.0001);
  // EXPECT_NEAR(0.0075, object.dimensions[1], 0.0001);
  // EXPECT_NEAR(0.0675, object.dimensions[2], 0.0001);
}

TEST_F(GraspObjectTest, getObjectWorldAnglesTestX)
{
  GenerateObjectCloud(0.05, 0.01, 0.02);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();
  object.get_object_world_angles();

  EXPECT_NEAR(1, object.objectWorldCosX, 0.0001);
  EXPECT_NEAR(0, object.objectWorldCosY, 0.0001);
  EXPECT_NEAR(0, object.objectWorldCosZ, 0.0001);
}

TEST_F(GraspObjectTest, getObjectWorldAnglesTestY)
{
  GenerateObjectCloud(0.03, 0.06, 0.05);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();
  object.get_object_world_angles();

  EXPECT_NEAR(0, object.objectWorldCosX, 0.0001);
  EXPECT_NEAR(1, object.objectWorldCosY, 0.0001);
  EXPECT_NEAR(0, object.objectWorldCosZ, 0.0001);

}

TEST_F(GraspObjectTest, getObjectWorldAnglesTestZ)
{
  GenerateObjectCloud(0.05, 0.01, 0.07);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();
  object.get_object_world_angles();

  EXPECT_NEAR(0, object.objectWorldCosX, 0.0001);
  EXPECT_NEAR(0, object.objectWorldCosY, 0.0001);
  EXPECT_NEAR(1, object.objectWorldCosZ, 0.0001);
}

TEST_F(GraspObjectTest, getObjectPoseTest)
{
  GenerateObjectCloud(0.01, 0.03, 0.05);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();
  object.get_object_world_angles();
  geometry_msgs::msg::PoseStamped result_pose =
    object.get_object_pose("camera_frame");
  EXPECT_NEAR(centroid(0), result_pose.pose.position.x, 0.0001);
  EXPECT_NEAR(centroid(1), result_pose.pose.position.y, 0.0001);
  EXPECT_NEAR(centroid(2), result_pose.pose.position.z, 0.0001);

  EXPECT_NEAR(0, result_pose.pose.orientation.x, 0.0001);
  EXPECT_NEAR(0, result_pose.pose.orientation.y, 0.0001);
  EXPECT_NEAR(0, result_pose.pose.orientation.z, 0.0001);
  EXPECT_NEAR(1, result_pose.pose.orientation.w, 0.0001);
}

TEST_F(GraspObjectTest, getObjectPoseTest2)
{
  GenerateObjectCloud(0.01, 0.06, 0.03);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();
  object.get_object_world_angles();
  geometry_msgs::msg::PoseStamped result_pose =
    object.get_object_pose("camera_frame");
  EXPECT_NEAR(centroid(0), result_pose.pose.position.x, 0.0001);
  EXPECT_NEAR(centroid(1), result_pose.pose.position.y, 0.0001);
  EXPECT_NEAR(centroid(2), result_pose.pose.position.z, 0.0001);

  EXPECT_NEAR(0.7071068, result_pose.pose.orientation.x, 0.0001);
  EXPECT_NEAR(0, result_pose.pose.orientation.y, 0.0001);
  EXPECT_NEAR(0, result_pose.pose.orientation.z, 0.0001);
  EXPECT_NEAR(0.7071068, result_pose.pose.orientation.w, 0.0001);
}

TEST_F(GraspObjectTest, getObjectShapeTest)
{
  GenerateObjectCloud(0.05, 0.01, 0.07);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);
  PCLFunctions::compute_cloud_normal(object.cloud, object.cloud_normal, 0.03);
  object.get_object_bb();
  shape_msgs::msg::SolidPrimitive shape = object.get_object_shape();
}

TEST_F(GraspObjectTest, getAxisAlignmentsTest)
{
  GenerateObjectCloud(0.01, 0.06, 0.03);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);
  object.axis = {0, 0, 1};
  object.grasp_axis = {1, 0, 0};
  object.minor_axis = {0, 1, 0};
  object.get_axis_alignments();
  EXPECT_TRUE(object.alignments[0] == 'z');
  EXPECT_TRUE(object.alignments[1] == 'x');
  EXPECT_TRUE(object.alignments[2] == 'y');

}

TEST_F(GraspObjectTest, getAxisTest)
{
  GenerateObjectCloud(0.01, 0.06, 0.03);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  GraspObject object("camera_frame", object_cloud, centroid);

  EXPECT_TRUE(object.get_axis({1, 0, 0}) == 'x');
  EXPECT_TRUE(object.get_axis({0, 1, 0}) == 'y');
  EXPECT_TRUE(object.get_axis({0, 0, 1}) == 'z');
}
