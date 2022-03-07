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

#ifndef EMD__DYNAMIC_SAFETY__COLLISION_CHECKER_MOVEIT_HPP_
#define EMD__DYNAMIC_SAFETY__COLLISION_CHECKER_MOVEIT_HPP_

#include <string>
#include <vector>

#include "emd/dynamic_safety/collision_checker_common.hpp"
#include "moveit/collision_detection/collision_common.h"
#include "moveit/planning_scene/planning_scene.h"

namespace dynamic_safety_moveit
{

class MoveitCollisionCheckerContext : public dynamic_safety::CollisionCheckerContext
{
public:
  MoveitCollisionCheckerContext() {}
  virtual ~MoveitCollisionCheckerContext() {}

  MoveitCollisionCheckerContext(
    const std::string & robot_urdf,
    const std::string & robot_srdf,
    const std::string & collision_checking_plugin);

  void configure(
    const dynamic_safety::CollisionCheckerOption & option) override;

  void run_discrete(
    std::vector<std::string> joint_names,
    trajectory_msgs::msg::JointTrajectoryPoint point,
    uint8_t & result, double & distance) override;

  void run_continuous(
    std::vector<std::string> joint_names,
    trajectory_msgs::msg::JointTrajectoryPoint point1,
    trajectory_msgs::msg::JointTrajectoryPoint point2,
    uint8_t & result, double & distance) override;

  void update(
    const sensor_msgs::msg::JointState & joint_states) override;

  void update(
    const moveit_msgs::msg::PlanningScene & scene_msgs) override;

protected:
  planning_scene::PlanningScenePtr scene_;

  // collision request
  collision_detection::CollisionRequest collision_request_;
  collision_detection::CollisionResult collision_result_;
};

}  // namespace dynamic_safety_moveit

#endif  // EMD__DYNAMIC_SAFETY__COLLISION_CHECKER_MOVEIT_HPP_
