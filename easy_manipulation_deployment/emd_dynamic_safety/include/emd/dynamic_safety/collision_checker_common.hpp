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

#ifndef EMD__DYNAMIC_SAFETY__COLLISION_CHECKER_COMMON_HPP_
#define EMD__DYNAMIC_SAFETY__COLLISION_CHECKER_COMMON_HPP_

#include <string>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"

namespace dynamic_safety
{

/// Collision checker options.
struct CollisionCheckerOption
{
  /// Planning framework (moveit or tesseract)
  std::string framework{"moveit"};

  /// Collision checking plugin
  std::string collision_checking_plugin{"fcl"};

  /// Group for detection
  std::string group;

  /// Whether to use distance based collison checking (not fully implemented).
  bool distance;

  /// Continuous collison checker (not implemented).
  bool continuous;

  /// Enable realtime configuration (not fully implemented).
  bool realtime;

  /// Steps (in the unit of time) between states taken from the trajectory (discrete only).
  double step;
  /// Thread count used for collision checking (discrete only).
  int thread_count;
};

/// Collision checking context to-be-inherited.
class CollisionCheckerContext
{
public:
  CollisionCheckerContext() {}
  virtual ~CollisionCheckerContext() {}

  CollisionCheckerContext(
    const std::string & /*robot_urdf*/,
    const std::string & /*robot_srdf*/,
    const std::string & /*collision_checking_plugin*/) {}

  virtual void configure(
    const CollisionCheckerOption & option) = 0;

  virtual void run_discrete(
    std::vector<std::string> joint_names,
    trajectory_msgs::msg::JointTrajectoryPoint point,
    uint8_t & result, double & distance) = 0;

  virtual void run_continuous(
    std::vector<std::string> joint_names,
    trajectory_msgs::msg::JointTrajectoryPoint point1,
    trajectory_msgs::msg::JointTrajectoryPoint point2,
    uint8_t & result, double & distance) = 0;

  virtual void update(
    const sensor_msgs::msg::JointState & joint_states) = 0;

  virtual void update(
    const moveit_msgs::msg::PlanningScene & scene_msgs) = 0;
};

}  // namespace dynamic_safety

#endif  // EMD__DYNAMIC_SAFETY__COLLISION_CHECKER_COMMON_HPP_
