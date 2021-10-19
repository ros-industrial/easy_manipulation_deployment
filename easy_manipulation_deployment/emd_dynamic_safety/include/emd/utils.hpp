// Copyright 2020 ROS Industrial Consortium Asia Pacific
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

#ifndef EMD__UTILS_HPP_
#define EMD__UTILS_HPP_

#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"


namespace emd
{

template<typename T>
/// Get parameter (declare if doesn't allow overload)
/**
 * Referenced from moveit servo.
 */
inline void declare_or_get_param(
  T & output_value,
  const std::string & param_name,
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Logger & logger,
  const T & default_value = T{})
{
  try {
    if (node->has_parameter(param_name)) {
      node->get_parameter_or<T>(param_name, output_value, default_value);
    } else {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
      output_value = node->declare_parameter<T>(param_name, default_value);
#pragma GCC diagnostic pop
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    // Catch a <double> parameter written in the yaml as "1" being considered an <int>
    if (std::is_same<T, double>::value) {
      node->undeclare_parameter(param_name);
      output_value = static_cast<double>(node->declare_parameter<int>(param_name, 0));
    } else {
      RCLCPP_ERROR(
        logger,
        "Error getting parameter \'%s\', check parameter type in YAML file.",
        param_name.c_str());
      throw e;
    }
  }
  RCLCPP_INFO_STREAM(
    logger,
    "Found parameter - " << param_name << ": " << output_value);
}

}  // namespace emd

#endif  // EMD__UTILS_HPP_
