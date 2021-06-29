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

#ifndef GRASP_PLANNER__COMMON__MATH_FUNCTIONS_HPP_
#define GRASP_PLANNER__COMMON__MATH_FUNCTIONS_HPP_

// Other Libraries
#include <Eigen/Core>

// For uuid
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/random_generator.hpp"
#include "boost/uuid/uuid_io.hpp"

#include <pcl/point_types.h>
namespace MathFunctions
{

struct Point
{
  Eigen::Vector3f reference_vector;
  Eigen::Vector3f reference_point;
  float projection_distance;
  float perpendicular_distance;
  int index;
  pcl::PointNormal pcl_npoint;
  Point(
    Eigen::Vector3f reference_vector_,
    Eigen::Vector3f reference_point_,
    float projection_distance_,
    float perpendicular_distance_)
  {
    reference_vector = reference_vector_;
    reference_point = reference_point_;
    projection_distance = projection_distance_;
    perpendicular_distance = perpendicular_distance_;
  }
};
float normalize(
  const float & target,
  const float & min,
  const float & max);

float normalizeInt(
  const int & target,
  const int & min,
  const int & max);

float getAngleBetweenVectors(
  const Eigen::Vector3f & vector_1,
  const Eigen::Vector3f & vector_2);

Eigen::Vector3f getPointInDirection(
  const Eigen::Vector3f & base_point,
  const Eigen::Vector3f & vector_direction,
  const float & distance);

Eigen::Vector3f getRotatedVector(
  const Eigen::Vector3f & target_vector,
  const float & angle,
  const char & axis);

MathFunctions::Point getPointInfo(
  const Eigen::Vector3f & target_point,
  const Eigen::Vector3f & vector_direction,
  const Eigen::Vector3f & point_on_vector);

std::string generate_task_id();

}  // namespace MathFunctions

#endif  // GRASP_PLANNER__COMMON__MATH_FUNCTIONS_HPP_
