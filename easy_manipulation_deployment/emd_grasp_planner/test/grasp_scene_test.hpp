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


#ifndef GRASP_SCENE_TEST_HPP_
#define GRASP_SCENE_TEST_HPP_

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

#include "emd/grasp_planner/grasp_scene.hpp"

class GraspSceneTest : public ::testing::Test
{
public:
  // std::shared_ptr<grasp_planner::GraspScene> grasp_scene;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud;
  Eigen::Vector3f world_x{1, 0, 0};
  Eigen::Vector3f world_y{0, 1, 0};
  Eigen::Vector3f world_z{0, 0, 1};
  rclcpp::Node::SharedPtr node;

  GraspSceneTest();
  void GenerateSceneCloud(float length, float breadth, float height);

  void SetUp(void)
  {
    std::cout << "Setup" << std::endl;
  }
  void TearDown(void)
  {
    std::cout << "Teardown" << std::endl;
  }
};

#endif  // GRASP_OBJECT_TEST_HPP_
