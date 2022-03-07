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


#ifndef PCL_FUNCTIONS_TEST_HPP_
#define PCL_FUNCTIONS_TEST_HPP_

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

#include "emd/grasp_planner/end_effectors/finger_gripper.hpp"
#include "emd/common/pcl_functions.hpp"
#include "emd/common/fcl_types.hpp"
#include "emd/grasp_planner/grasp_object.hpp"

class PCLFunctionsTest : public ::testing::Test
{
public:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rectangle_cloud;
  PCLFunctionsTest();
  void GenerateCloud(float length, float breadth, float height);
  void SetUp(void)
  {
    std::cout << "Setup" << std::endl;
  }
  void TearDown(void)
  {
    std::cout << "Teardown" << std::endl;
  }
};

#endif  // PCL_FUNCTIONS_TEST_HPP_
