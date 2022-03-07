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
#include "emd/common/math_functions.hpp"

TEST(MathFunctionTest, NormalizeTest)
{
  float result = MathFunctions::normalize(10.0, 0.0, 100.0);
  EXPECT_NEAR(result, 0.1, 0.00001);
}

TEST(MathFunctionTest, NormalizeTestMaxMinTargetEqual)
{
  float result = MathFunctions::normalize(5.5, 5.5, 5.5);
  EXPECT_NEAR(result, 1, 0.00001);
}

TEST(MathFunctionTest, NormalizeTestMinMoreThanMax)
{
  float result = MathFunctions::normalize(13.0, 5.6, 1.2);
  EXPECT_NEAR(result, -1, 0.00001);
}

TEST(MathFunctionTest, NormalizeTestMinEqualMax)
{
  float result = MathFunctions::normalize(2.4, 4.4, 4.4);
  EXPECT_NEAR(result, -1, 0.00001);
}

TEST(MathFunctionTest, NormalizeIntTest)
{
  float result = MathFunctions::normalize_int(10, 1, 200);
  EXPECT_NEAR(result, 0.045226, 0.00001);
}

TEST(MathFunctionTest, NormalizeIntTestMaxMinTargetEqual)
{
  float result = MathFunctions::normalize_int(6, 6, 6);
  EXPECT_NEAR(result, 1, 0.00001);
}

TEST(MathFunctionTest, NormalizeIntTestMinMoreThanMax)
{
  float result = MathFunctions::normalize_int(22, 13, 7);
  EXPECT_NEAR(result, -1, 0.00001);
}

TEST(MathFunctionTest, NormalizeIntTestMinEqualMax)
{
  float result = MathFunctions::normalize_int(20, 60, 60);
  EXPECT_NEAR(result, -1, 0.00001);
}


TEST(MathFunctionTest, getAngleBetweenVectorsTest)
{
  Eigen::Vector3f vector_1{3, 4, 0};
  Eigen::Vector3f vector_2{4, 4, 2};
  float result = MathFunctions::get_angle_between_vectors(vector_1, vector_2);
  EXPECT_NEAR(0.933333, result, 0.0001);
}

TEST(MathFunctionTest, getPointInDirectionTestPosDir)
{
  Eigen::Vector3f direction{0, 1.0, 0};
  Eigen::Vector3f point_1{0, 3.1, 0};
  Eigen::Vector3f point_2 = MathFunctions::get_point_in_direction(point_1, direction, 2.0);
  EXPECT_NEAR(0, point_2(0), 0.0001);
  EXPECT_NEAR(5.1, point_2(1), 0.0001);
  EXPECT_NEAR(0, point_2(2), 0.0001);
}

TEST(MathFunctionTest, getPointInDirectionTestNegDir)
{
  Eigen::Vector3f direction{1, 0, 0};
  Eigen::Vector3f point_1{2.4, 0, 0};
  Eigen::Vector3f point_2 = MathFunctions::get_point_in_direction(point_1, direction, -3.1);
  EXPECT_NEAR(-0.7, point_2(0), 0.0001);
  EXPECT_NEAR(0, point_2(1), 0.0001);
  EXPECT_NEAR(0, point_2(2), 0.0001);
}

TEST(MathFunctionTest, getRotatedVectorTestXRot)
{
  Eigen::Vector3f direction{0, 0, 1};
  Eigen::Vector3f direction2{1, 0, 0};
  Eigen::Vector3f result = MathFunctions::get_rotated_vector(
    direction, 1.57079632679, 'x');
  EXPECT_NEAR(0, direction.dot(result), 0.0001);
  EXPECT_NEAR(0, direction2.dot(result), 0.0001);
}

TEST(MathFunctionTest, getRotatedVectorTestYRot)
{
  Eigen::Vector3f direction{1, 0, 0};
  Eigen::Vector3f direction2{0, 1, 0};
  Eigen::Vector3f result = MathFunctions::get_rotated_vector(
    direction, 1.57079632679, 'y');
  EXPECT_NEAR(0, direction.dot(result), 0.0001);
  EXPECT_NEAR(0, direction2.dot(result), 0.0001);
}

TEST(MathFunctionTest, getRotatedVectorTestZRot)
{
  Eigen::Vector3f direction{0, 1, 0};
  Eigen::Vector3f direction2{0, 0, 1};
  Eigen::Vector3f result = MathFunctions::get_rotated_vector(
    direction, 1.57079632679, 'z');
  EXPECT_NEAR(0, direction.dot(result), 0.0001);
  EXPECT_NEAR(0, direction2.dot(result), 0.0001);
}

TEST(MathFunctionTest, getRotatedVectorTestWrongRot)
{
  Eigen::Vector3f direction{0, 1, 0};
  Eigen::Vector3f direction2{0, 0, 1};
  EXPECT_THROW(
    MathFunctions::get_rotated_vector(
      direction, 1.57079632679, 'o'), std::invalid_argument);
}

TEST(MathFunctionTest, generateTaskIdTest)
{
  for (int i = 0; i < 10; i++) {
    EXPECT_FALSE(MathFunctions::generate_task_id() == MathFunctions::generate_task_id());
  }

}
