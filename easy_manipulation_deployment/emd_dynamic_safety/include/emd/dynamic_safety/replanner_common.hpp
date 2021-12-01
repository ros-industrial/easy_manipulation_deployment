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

#ifndef EMD__DYNAMIC_SAFETY__REPLANNER_COMMON_HPP_
#define EMD__DYNAMIC_SAFETY__REPLANNER_COMMON_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace dynamic_safety
{

struct ReplannerOption
{
  /// Planning framework (moveit or tesseract)
  std::string framework{"moveit"};

  /// Planner
  std::string planner{"ompl"};

  /// OMPL planner configuration id
  /// Example
  std::string ompl_planner_id;

  std::string joint_limits_parameter_server;
  std::string joint_limits_parameter_namespace;
  /// Planner parameter server to inherit from
  std::string planner_parameter_server;
  std::string planner_parameter_namespace;

  /// Time parameterization implemeration
  std::string time_parameterization{"totg"};

  /// Group for planning
  std::string group;

  /// deadline
  double deadline;
};

/// Flags for status
namespace ReplannerStatus
{
static const uint8_t
  IDLE = 0,
  ONGOING = 1,
  SUCCEED = 2,
  TIMEOUT = 3;    // Not used at the moment
}  // ReplannerStatus

/// Collision checking context to-be-inherited.
class ReplannerContext
{
public:
  ReplannerContext() {}
  virtual ~ReplannerContext() {}

  ReplannerContext(
    const std::string & /*robot_urdf*/,
    const std::string & /*robot_srdf*/,
    const ReplannerOption & /*option*/,
    const rclcpp::Node::SharedPtr & /*node*/) {}

  virtual void run(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint & start_point,
    const trajectory_msgs::msg::JointTrajectoryPoint & end_point,
    trajectory_msgs::msg::JointTrajectory & plan) = 0;

  virtual bool time_parameterize(
    trajectory_msgs::msg::JointTrajectory & plan,
    double scale = 1.0) = 0;

  virtual void update(
    const sensor_msgs::msg::JointState & joint_states) = 0;

  virtual void update(
    const moveit_msgs::msg::PlanningScene & scene_msg) = 0;
};

}  // namespace dynamic_safety

#endif  // EMD__DYNAMIC_SAFETY__REPLANNER_COMMON_HPP_
