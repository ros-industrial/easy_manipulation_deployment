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

#ifndef EMD__GRASP_PLANNER__COMMON__MATH_FUNCTIONS_HPP_
#define EMD__GRASP_PLANNER__COMMON__MATH_FUNCTIONS_HPP_

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
  : reference_vector(reference_vector_),
    reference_point(reference_point_),
    projection_distance(projection_distance_),
    perpendicular_distance(perpendicular_distance_)
  {}
};

/**
 * Function to normalize float values
 *
 * \param[in] target Target Value
 * \param[in] min Min value
 * \param[in] max Max value
 */
float normalize(
  const float & target,
  const float & min,
  const float & max);

/**
 * Function to normalize int values
 *
 * \param[in] target Target Value
 * \param[in] min Min value
 * \param[in] max Max value
 */
float normalize_int(
  const int & target,
  const int & min,
  const int & max);

/**
 * Function to find angle between two vectors
 *
 * \param[in] vector_1 Vector 1
 * \param[in] vector_2 Vector 2
 */
float get_angle_between_vectors(
  const Eigen::Vector3f & vector_1,
  const Eigen::Vector3f & vector_2);

/**
 * Function to return a point in a certain direction and distance away from a base point
 *
 * \param[in] base_point Reference point from which calculations are made
 * \param[in] vector_direction Reference direction from which calcuations are made
 * \param[in] distance distance from the resultant point to te base point
 */
Eigen::Vector3f get_point_in_direction(
  const Eigen::Vector3f & base_point,
  const Eigen::Vector3f & vector_direction,
  const float & distance);

/**
 * Function to return a target vector rotated by a certain angle about a cetain axis
 *
 * \param[in] target_vector Target vector to rotate
 * \param[in] angle Angle to rotate by in radians
 * \param[in] axis Axis about which to rotate
 */
Eigen::Vector3f get_rotated_vector(
  const Eigen::Vector3f & target_vector,
  const float & angle,
  const char & axis);

/**
 * Function to return a MathFunctions::Point object with given parameters
 *
 * \param[in] target_point Target point coordinate
 * \param[in] vector_direction Vector direction
 * \param[in] point_on_vector Another point on the vector
 */
MathFunctions::Point get_point_info(
  const Eigen::Vector3f & target_point,
  const Eigen::Vector3f & vector_direction,
  const Eigen::Vector3f & point_on_vector);

/**
 * Function to generate a random string id
 */
std::string generate_task_id();

}  // namespace MathFunctions

#endif  // EMD__GRASP_PLANNER__COMMON__MATH_FUNCTIONS_HPP_
