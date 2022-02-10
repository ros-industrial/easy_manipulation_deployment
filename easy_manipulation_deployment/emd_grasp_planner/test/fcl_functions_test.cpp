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
#include "emd/common/fcl_functions.hpp"


TEST(FCLFunctionTest, CollisionTestCollide)
{
  float length = 0.05;
  float breadth = 0.01;
  float height = 0.02;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZ>());

  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.0; height_ < height; height_ += 0.0025) {
        pcl::PointXYZ temp_point;
        temp_point.x = length_;
        temp_point.y = breadth_;
        temp_point.z = height_;
        cloud->points.push_back(temp_point);
      }
    }
  }

  octomap::point3d sensor_origin(-0.5915, 0.19, 0.65);

  std::shared_ptr<grasp_planner::collision::CollisionObject> collision1_object_ptr =
    FCLFunctions::create_collision_object_from_pointcloud(
    cloud,
    sensor_origin,
    0.005);

  grasp_planner::collision::Sphere * collision2_shape =
    new grasp_planner::collision::Sphere(0.01 / 2);

  grasp_planner::collision::Transform collision2_transform;
  collision2_transform.setIdentity();

#if FCL_VERSION_0_6_OR_HIGHER == 1
  collision2_transform.translation() << 0.025, 0.005, 0.01;
#else
  collision2_transform.setTranslation(
    grasp_planner::collision::Vector(0.04, 0.005, 0.015));
#endif
  grasp_planner::collision::CollisionObject collision2_object(
    std::shared_ptr<grasp_planner::collision::CollisionGeometry>(collision2_shape),
    collision2_transform);
  std::shared_ptr<grasp_planner::collision::CollisionObject> collision2_object_ptr =
    std::make_shared<grasp_planner::collision::CollisionObject>(collision2_object);

  grasp_planner::collision::CollisionRequest request;
  request.enable_contact = true;

  grasp_planner::collision::CollisionResult result;
  fcl::collide(collision1_object_ptr.get(), collision2_object_ptr.get(), request, result);
  EXPECT_TRUE(result.isCollision());
}

TEST(FCLFunctionTest, CollisionTestNoCollide)
{
  float length = 0.05;
  float breadth = 0.01;
  float height = 0.02;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZ>());

  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.0; height_ < height; height_ += 0.0025) {
        pcl::PointXYZ temp_point;
        temp_point.x = length_;
        temp_point.y = breadth_;
        temp_point.z = height_;
        cloud->points.push_back(temp_point);
      }
    }
  }

  octomap::point3d sensor_origin(-0.5915, 0.19, 0.65);

  std::shared_ptr<grasp_planner::collision::CollisionObject> collision1_object_ptr =
    FCLFunctions::create_collision_object_from_pointcloud(
    cloud,
    sensor_origin,
    0.005);

  grasp_planner::collision::Sphere * collision2_shape =
    new grasp_planner::collision::Sphere(0.01 / 2);

  grasp_planner::collision::Transform collision2_transform;
  collision2_transform.setIdentity();

#if FCL_VERSION_0_6_OR_HIGHER == 1
  collision2_transform.translation() << 0.025, 0.005, 0.01;
#else
  collision2_transform.setTranslation(
    grasp_planner::collision::Vector(0.07, 0.07, 0.07));
#endif
  grasp_planner::collision::CollisionObject collision2_object(
    std::shared_ptr<grasp_planner::collision::CollisionGeometry>(collision2_shape),
    collision2_transform);
  std::shared_ptr<grasp_planner::collision::CollisionObject> collision2_object_ptr =
    std::make_shared<grasp_planner::collision::CollisionObject>(collision2_object);

  grasp_planner::collision::CollisionRequest request;
  request.enable_contact = true;

  grasp_planner::collision::CollisionResult result;
  fcl::collide(collision1_object_ptr.get(), collision2_object_ptr.get(), request, result);
  EXPECT_FALSE(result.isCollision());
}

TEST(FCLFunctionTest, RGBCollisionTestCollide)
{
  float length = 0.05;
  float breadth = 0.01;
  float height = 0.02;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZRGB>());

  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.0; height_ < height; height_ += 0.0025) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = length_;
        temp_point.y = breadth_;
        temp_point.z = height_;
        cloud->points.push_back(temp_point);
      }
    }
  }

  octomap::point3d sensor_origin(-0.5915, 0.19, 0.65);

  std::shared_ptr<grasp_planner::collision::CollisionObject> collision1_object_ptr =
    FCLFunctions::create_collision_object_from_pointcloud_rgb(
    cloud,
    sensor_origin,
    0.005);

  grasp_planner::collision::Sphere * collision2_shape =
    new grasp_planner::collision::Sphere(0.01 / 2);

  grasp_planner::collision::Transform collision2_transform;
  collision2_transform.setIdentity();

#if FCL_VERSION_0_6_OR_HIGHER == 1
  collision2_transform.translation() << 0.025, 0.005, 0.01;
#else
  collision2_transform.setTranslation(
    grasp_planner::collision::Vector(0.025, 0.005, 0.01));
#endif
  grasp_planner::collision::CollisionObject collision2_object(
    std::shared_ptr<grasp_planner::collision::CollisionGeometry>(collision2_shape),
    collision2_transform);
  std::shared_ptr<grasp_planner::collision::CollisionObject> collision2_object_ptr =
    std::make_shared<grasp_planner::collision::CollisionObject>(collision2_object);

  grasp_planner::collision::CollisionRequest request;
  request.enable_contact = true;

  grasp_planner::collision::CollisionResult result;
  fcl::collide(collision1_object_ptr.get(), collision2_object_ptr.get(), request, result);
  EXPECT_TRUE(result.isCollision());
}

TEST(FCLFunctionTest, RGBCollisionTestNoCollide)
{
  float length = 0.05;
  float breadth = 0.01;
  float height = 0.02;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZRGB>());

  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.0; height_ < height; height_ += 0.0025) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = length_;
        temp_point.y = breadth_;
        temp_point.z = height_;
        cloud->points.push_back(temp_point);
      }
    }
  }

  octomap::point3d sensor_origin(-0.5915, 0.19, 0.65);

  std::shared_ptr<grasp_planner::collision::CollisionObject> collision1_object_ptr =
    FCLFunctions::create_collision_object_from_pointcloud_rgb(
    cloud,
    sensor_origin,
    0.005);

  grasp_planner::collision::Sphere * collision2_shape =
    new grasp_planner::collision::Sphere(0.01 / 2);

  grasp_planner::collision::Transform collision2_transform;
  collision2_transform.setIdentity();

#if FCL_VERSION_0_6_OR_HIGHER == 1
  collision2_transform.translation() << 0.025, 0.005, 0.01;
#else
  collision2_transform.setTranslation(
    grasp_planner::collision::Vector(0.09, 0.09, 0.09));
#endif
  grasp_planner::collision::CollisionObject collision2_object(
    std::shared_ptr<grasp_planner::collision::CollisionGeometry>(collision2_shape),
    collision2_transform);
  std::shared_ptr<grasp_planner::collision::CollisionObject> collision2_object_ptr =
    std::make_shared<grasp_planner::collision::CollisionObject>(collision2_object);

  grasp_planner::collision::CollisionRequest request;
  request.enable_contact = true;

  grasp_planner::collision::CollisionResult result;
  fcl::collide(collision1_object_ptr.get(), collision2_object_ptr.get(), request, result);
  EXPECT_FALSE(result.isCollision());
  // std::shared_ptr<CollisionObject> FCLFunctions::create_collision_object_from_pointcloud(
  //   const pcl::PointCloud<pcl::PointXYZ>::Ptr & pointcloud_ptr,
  //   const octomap::point3d & sensor_origin_wrt_world,
  //   float resolution)
}
