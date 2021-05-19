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

namespace MathFunctions
{
float normalize(const float & target, const float & min, const float & max);

float normalizeInt(const int & target, const int & min, const int & max);

float getAngleBetweenVectors(Eigen::Vector3f vector_1, Eigen::Vector3f vector_2);

Eigen::Vector3f getPointInDirection(
  Eigen::Vector3f base_point, Eigen::Vector3f vector_direction,
  float distance);
Eigen::Vector3f getRotatedVector(
  Eigen::Vector3f target_vector,
  float angle, char axis);
}  // namespace MathFunctions

#endif  // GRASP_PLANNER__COMMON__MATH_FUNCTIONS_HPP_
