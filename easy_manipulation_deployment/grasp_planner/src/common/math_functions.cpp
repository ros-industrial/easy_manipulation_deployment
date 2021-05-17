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

#include "grasp_planner/common/math_functions.hpp"
#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger & LOGGER = rclcpp::get_logger("EMD::MathFunctions");

float MathFunctions::normalize(const float & target, const float & min, const float & max)
{
  if (min == max) {
    if(target == min){
      RCLCPP_WARN(LOGGER, "Possible Normalizing Error, Min, Max and target same value");
      return 1;
    }
    else{
      RCLCPP_WARN(LOGGER, "Error: Normalizing divides by zero.");
      return -1;
    }
  }
  else if(min > max){
    RCLCPP_WARN(LOGGER, "Error: Min value more than Max, normalizing error");
    return -1;
  }
  else {
    return (target - min) / (max - min);
  }
}

float MathFunctions::normalizeInt(const int & target, const int & min, const int & max)
{
  if (min == max) {
    if(target == min){
      RCLCPP_WARN(LOGGER, "Possible Normalizing Error, Min, Max and target same value");
      return 1;
    }
    else{
      RCLCPP_WARN(LOGGER, "Error: Normalizing divides by zero.");
      return -1;
    }
  }
  else if(min > max){
    RCLCPP_WARN(LOGGER, "Error: Min value more than Max, normalizing error");
    return -1;
  }
  else {
    return float(target - min) / float(max - min);
  }
}

float MathFunctions::getAngleBetweenVectors(Eigen::Vector3f vector_1, Eigen::Vector3f vector_2)
{
  return 1e-8 + std::abs((vector_1.dot(vector_2)) / (vector_1.norm() * vector_2.norm()));
}

Eigen::Vector3f MathFunctions::getPointInDirection(
  Eigen::Vector3f base_point,
  Eigen::Vector3f vector_direction,
  float distance)
{
  Eigen::Vector3f direction_normalized = vector_direction / vector_direction.norm();
  return base_point + distance * direction_normalized;
}

Eigen::Vector3f MathFunctions::getRotatedVector(Eigen::Vector3f target_vector, float angle, char axis)
{
  Eigen::Vector3f result_vector;
  if(axis == 'x'){
    result_vector(0) = target_vector(0); 
    result_vector(1) = target_vector(1) * cos(angle) - target_vector(2) * sin(angle);
    result_vector(2) = target_vector(1) * sin(angle) + target_vector(2) * cos(angle);
  }
  else if(axis == 'y'){
    result_vector(0) = target_vector(0) * cos(angle) - target_vector(1) * sin(angle);
    result_vector(1) = target_vector(1);
    result_vector(2) = target_vector(2) * -sin(angle) + target_vector(1) * cos(angle);
  }
  else if(axis == 'z'){
    result_vector(0) = target_vector(0) * cos(angle) - target_vector(1) * sin(angle);
    result_vector(1) = target_vector(0) * sin(angle) + target_vector(1) * cos(angle);
    result_vector(2) = target_vector(2);
  }
  else{
    RCLCPP_ERROR(LOGGER, "Invalid axis of rotation.");
    throw std::invalid_argument("Invalid axis of rotation.");
  }
  return result_vector;
}
