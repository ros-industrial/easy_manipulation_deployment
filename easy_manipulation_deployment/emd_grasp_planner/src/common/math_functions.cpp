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
// Main PCL files

#include "emd/common/math_functions.hpp"
#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger & LOGGER = rclcpp::get_logger("EMD::MathFunctions");

float MathFunctions::normalize(
  const float & target,
  const float & min,
  const float & max)
{
  if (min == max) {
    if (target == min) {
      RCLCPP_WARN(LOGGER, "Possible Normalizing Error, Min, Max and target same value");
      return 1;
    } else {
      RCLCPP_WARN(LOGGER, "Error: Normalizing divides by zero.");
      return -1;
    }
  } else if (min > max) {
    RCLCPP_WARN(LOGGER, "Error: Min value more than Max, normalizing error");
    return -1;
  } else {
    return (target - min) / (max - min);
  }
}

float MathFunctions::normalize_int(
  const int & target,
  const int & min,
  const int & max)
{
  if (min == max) {
    if (target == min) {
      RCLCPP_WARN(LOGGER, "Possible Normalizing Error, Min, Max and target same value");
      return 1;
    } else {
      RCLCPP_WARN(LOGGER, "Error: Normalizing divides by zero.");
      return -1;
    }
  } else if (min > max) {
    RCLCPP_WARN(LOGGER, "Error: Min value more than Max, normalizing error");
    return -1;
  } else {
    return float(target - min) / float(max - min);
  }
}

float MathFunctions::get_angle_between_vectors(
  const Eigen::Vector3f & vector_1,
  const Eigen::Vector3f & vector_2)
{
  return 1e-8 + std::abs((vector_1.dot(vector_2)) / (vector_1.norm() * vector_2.norm()));
}

Eigen::Vector3f MathFunctions::get_point_in_direction(
  const Eigen::Vector3f & base_point,
  const Eigen::Vector3f & vector_direction,
  const float & distance)
{
  Eigen::Vector3f direction_normalized = vector_direction / vector_direction.norm();
  return base_point + distance * direction_normalized;
}

MathFunctions::Point MathFunctions::get_point_info(
  const Eigen::Vector3f & target_point,
  const Eigen::Vector3f & vector_direction,
  const Eigen::Vector3f & point_on_vector)
{
  Eigen::Vector3f target_vector = target_point - point_on_vector;
  float projection_distance = vector_direction.dot(target_vector);
  Eigen::Vector3f projected_point = MathFunctions::get_point_in_direction(
    point_on_vector, vector_direction, projection_distance);
  float perpendicular_distance = (projected_point - target_point).norm();
  return Point(vector_direction, point_on_vector, projection_distance, perpendicular_distance);
}
Eigen::Vector3f MathFunctions::get_rotated_vector(
  const Eigen::Vector3f & target_vector,
  const float & angle,
  const char & axis)
{
  Eigen::Vector3f result_vector;
  if (axis == 'x') {
    result_vector(0) = target_vector(0);
    result_vector(1) = target_vector(1) * cos(angle) - target_vector(2) * sin(angle);
    result_vector(2) = target_vector(1) * sin(angle) + target_vector(2) * cos(angle);
  } else if (axis == 'y') {
    result_vector(0) = target_vector(0) * cos(angle) - target_vector(1) * sin(angle);
    result_vector(1) = target_vector(1);
    result_vector(2) = target_vector(2) * -sin(angle) + target_vector(1) * cos(angle);
  } else if (axis == 'z') {
    result_vector(0) = target_vector(0) * cos(angle) - target_vector(1) * sin(angle);
    result_vector(1) = target_vector(0) * sin(angle) + target_vector(1) * cos(angle);
    result_vector(2) = target_vector(2);
  } else {
    RCLCPP_ERROR(LOGGER, "Invalid axis of rotation.");
    throw std::invalid_argument("Invalid axis of rotation.");
  }
  return result_vector;
}

/****************************************************************************************//**
 * Function that generates a unique ID for Grasp Tasks. Returns the unique ID
 *******************************************************************************************/
std::string MathFunctions::generate_task_id()
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  return boost::uuids::to_string(uuid);
}
