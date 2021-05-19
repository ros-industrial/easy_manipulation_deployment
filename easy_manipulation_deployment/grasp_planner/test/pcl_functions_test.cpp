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
#include "pcl_functions_test.hpp"

PCLFunctionsTest() : rectangle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
}

void PCLFunctionsTest::GenerateCloud()
{
  float length = 0.05;
  float breadth = 0.01;
  float height = 0.02;

  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.0; height_ < height; height_ += 0.0025) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = length_;
        temp_point.y = breadth_;
        temp_point.x = height_;
        rectangle_cloud->points.push_back(temp_point);
      }
    }
  }
}

void PCLFunctionsTest::AddNoise()
{

}

TEST_F(PCLFunctionsTest, NormalizeTest)
{
  float result = MathFunctions::normalize(10.0, 0.0, 100.0);
  EXPECT_NEAR(result, 0.1, 0.00001);
}
