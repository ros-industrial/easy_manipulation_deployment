// Copyright 2021 ROS Industrial Consortium Asia Pacific
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

#ifndef EMD__INTERPOLATE_HPP_
#define EMD__INTERPOLATE_HPP_

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rclcpp/time.hpp"

namespace emd
{

namespace core
{

void interpolate_between_points(
  rclcpp::Duration time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
  rclcpp::Duration time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
  rclcpp::Duration sample_time,
  trajectory_msgs::msg::JointTrajectoryPoint & output);

/// Interpolate between two points
/*
 * Referenced from joint_trajectory_controller src/trajectory.cpp
 */
void interpolate_between_points(
  double time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
  double time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
  double sample_time,
  trajectory_msgs::msg::JointTrajectoryPoint & output);

}  // namespace core

}  // namespace emd

#endif  // EMD__INTERPOLATE_HPP_
