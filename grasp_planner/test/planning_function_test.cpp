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
#include <vector>
#include "planning_functions.hpp"
#include "msg.hpp"

TEST(LengthTest, PositiveNos)
{
  EXPECT_NEAR(3.61, length(1, 3, 4, 5), 0.01);
  EXPECT_NEAR(25.50, length(35, 45, 40, 20), 0.01);
  EXPECT_NEAR(0.0, length(0, 0, 0, 0), 0.01);
}

TEST(LengthTest, NegativeNos)
{
  EXPECT_NEAR(6.32, length(-2, -9, -8, -7), 0.01);
  EXPECT_NEAR(52.33, length(-47, -36, -84, -73), 0.01);
  EXPECT_NEAR(52.33, length(47, -36, 84, -73), 0.01);
  EXPECT_NEAR(52.33, length(-47, 36, -84, 73), 0.01);
  EXPECT_NEAR(170.42, length(-47, -36, 84, 73), 0.01);
}

TEST(InCircleTest, InsideCircle)
{
  ASSERT_TRUE(in_circle(4, {3, 5}, {5, 5}));
  ASSERT_TRUE(in_circle(12, {-15, 1}, {-5, -2}));
}

TEST(InCircleTest, OutsideCircle)
{
  ASSERT_FALSE(in_circle(4, {3, 5}, {8, 5}));
  ASSERT_FALSE(in_circle(12, {-15, 1}, {-20, 16}));
}


TEST(InCircleTest, OnCircle)
{
  ASSERT_FALSE(in_circle(4, {3, 5}, {3, 1}));
}

TEST(QuadraticEquationTest, PositiveDiscriminant)
{
  std::vector<float> result = quadratic_equation(6, 11, -35);

  EXPECT_EQ(2, static_cast<int>(result.size()));

  EXPECT_NEAR(1.667, result[0], 0.01);
  EXPECT_NEAR(-3.5, result[1], 0.01);

  result = quadratic_equation(-1, 6, 18);
  EXPECT_EQ(2, static_cast<int>(result.size()));
  EXPECT_NEAR(-2.2, result[0], 0.01);
  EXPECT_NEAR(8.2, result[1], 0.01);
}
TEST(QuadraticEquationTest, NegativeDiscriminant)
{
  std::vector<float> result = quadratic_equation(6, 1, 5);

  EXPECT_EQ(0, static_cast<int>(result.size()));

  result = quadratic_equation(10, 5, 12);
  EXPECT_EQ(0, static_cast<int>(result.size()));
}
TEST(QuadraticEquationTest, ZeroDiscriminant)
{
  std::vector<float> result = quadratic_equation(3, 6, 3);

  EXPECT_EQ(1, static_cast<int>(result.size()));
  EXPECT_NEAR(-1, result[0], 0.01);

  result = quadratic_equation(2, 10, 12.5);
  EXPECT_EQ(1, static_cast<int>(result.size()));
  EXPECT_NEAR(-2.5, result[0], 0.01);
}
TEST(CircleLineIntersectTest, TwoIntersects)
{
  std::vector<std::vector<int>> result = circle_line_intersect(5, 6, 13.5, {3, 19});

  EXPECT_EQ(2, static_cast<int>(result.size()));
  ASSERT_TRUE(
    static_cast<int>(result[0].size()) == 2 &&
    static_cast<int>(result[1].size()) == 2);

  ASSERT_TRUE(result[0][0] == 5 || result[0][0] == 0);
  ASSERT_TRUE(result[1][0] == 5 || result[1][0] == 0);
  ASSERT_TRUE(result[0][1] == 6 || result[0][1] == 32);
  ASSERT_TRUE(result[1][1] == 6 || result[1][1] == 32);

  result = circle_line_intersect(1, 8, sqrt(17), {-3, 1});
  EXPECT_EQ(2, static_cast<int>(result.size()));
  ASSERT_TRUE(
    static_cast<int>(result[0].size()) == 2 &&
    static_cast<int>(result[1].size()) == 2);
  ASSERT_TRUE(result[0][0] == -7 || result[0][0] == -3);
  ASSERT_TRUE(result[1][0] == -7 || result[1][0] == -3);
  ASSERT_TRUE(result[0][1] == 1 || result[0][1] == 5);
  ASSERT_TRUE(result[1][1] == 1 || result[1][1] == 5);
}
TEST(CircleLineIntersectTest, OneIntersect)
{
  std::vector<std::vector<int>> result = circle_line_intersect(-0.25, 4.5, sqrt(17), {-3, 1});

  EXPECT_EQ(0, static_cast<int>(result.size()));
}
TEST(CircleLineIntersectTest, NoIntersect)
{
  std::vector<std::vector<int>> result = circle_line_intersect(1.5, -15, 13.5, {3, 19});

  EXPECT_EQ(0, static_cast<int>(result.size()));

  result = circle_line_intersect(-5, 8, sqrt(17), {-3, 1});
  EXPECT_EQ(0, static_cast<int>(result.size()));
}
TEST(CircleLineIntersectTest, CenterVecSize)
{
  std::vector<std::vector<int>> result = circle_line_intersect(1.5, -15, 13.5, {3});

  EXPECT_EQ(0, static_cast<int>(result.size()));
  result = circle_line_intersect(1.5, -15, 13.5, {3});
  EXPECT_EQ(0, static_cast<int>(result.size()));
}
TEST(CircleLineIntersectTest, NegRadius)
{
  std::vector<std::vector<int>> result = circle_line_intersect(5, 6, -13.5, {3, 19});

  EXPECT_EQ(0, static_cast<int>(result.size()));
}

TEST(BorderCornerTest, PosCenter)
{
  std::vector<std::vector<int>> result = get_border_corners({50, 55}, 6.4);

  EXPECT_EQ(2, static_cast<int>(result.size()));
  ASSERT_TRUE(
    static_cast<int>(result[0].size()) == 2 &&
    static_cast<int>(result[1].size()) == 2);
  ASSERT_EQ(result[0][0], 44);
  ASSERT_EQ(result[0][1], 49);
  ASSERT_EQ(result[1][0], 56);
  ASSERT_EQ(result[1][1], 61);
}
TEST(BorderCornerTest, NegCenter)
{
  std::vector<std::vector<int>> result = get_border_corners({-5, -20}, 6.4);

  EXPECT_EQ(0, static_cast<int>(result.size()));
}

TEST(BorderCornerTest, CloseCornerValues)
{
  std::vector<std::vector<int>> result = get_border_corners({5, 50}, 7.8);

  EXPECT_EQ(2, static_cast<int>(result.size()));
  ASSERT_TRUE(
    static_cast<int>(result[0].size()) == 2 &&
    static_cast<int>(result[1].size()) == 2);
  ASSERT_EQ(result[0][0], 0);
  ASSERT_EQ(result[0][1], 42);
  ASSERT_EQ(result[1][0], 13);
  ASSERT_EQ(result[1][1], 58);

  result = get_border_corners({5, 2}, 7.8);
  EXPECT_EQ(2, static_cast<int>(result.size()));
  ASSERT_TRUE(
    static_cast<int>(result[0].size()) == 2 &&
    static_cast<int>(result[1].size()) == 2);
  ASSERT_EQ(result[0][0], 0);
  ASSERT_EQ(result[0][1], 0);
  ASSERT_EQ(result[1][0], 13);
  ASSERT_EQ(result[1][1], 10);
}

TEST(BorderCornerTest, WrongCenterSize)
{
  std::vector<std::vector<int>> result = get_border_corners({}, 7.8);

  EXPECT_EQ(0, static_cast<int>(result.size()));
  result = get_border_corners({5}, 7.8);
  EXPECT_EQ(0, static_cast<int>(result.size()));
  result = get_border_corners({5, 2, 1}, 7.8);
  EXPECT_EQ(0, static_cast<int>(result.size()));
}

TEST(BorderCornerTest, NegRadius)
{
  std::vector<std::vector<int>> result = get_border_corners({5, 2}, -7.8);

  EXPECT_EQ(0, static_cast<int>(result.size()));
}

TEST(AngleBoundsTest, WithinBounds)
{
  EXPECT_NEAR(keep_angle_in_bounds(3.11), 3.11, 0.01);
  EXPECT_NEAR(keep_angle_in_bounds(2.55), 2.55, 0.01);
  EXPECT_NEAR(keep_angle_in_bounds(1.46), 1.46, 0.01);
  EXPECT_NEAR(keep_angle_in_bounds(0.66), 0.66, 0.01);
}

TEST(AngleBoundsTest, OutBoundsNeg)
{
  EXPECT_NEAR(0.732, keep_angle_in_bounds(-2.41), 0.01);
  EXPECT_NEAR(2.695, keep_angle_in_bounds(-6.73), 0.01);
  EXPECT_NEAR(2.582, keep_angle_in_bounds(-0.56), 0.01);
}

TEST(AngleBoundsTest, OutBoundsPos)
{
  EXPECT_NEAR(0.518, keep_angle_in_bounds(3.66), 0.01);
  EXPECT_NEAR(1.477, keep_angle_in_bounds(7.76), 0.01);
  EXPECT_NEAR(0.858, keep_angle_in_bounds(4), 0.01);
}
