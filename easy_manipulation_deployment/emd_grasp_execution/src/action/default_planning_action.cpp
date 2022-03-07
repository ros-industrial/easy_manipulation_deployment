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

#include <memory>
#include <string>
#include <utility>

#include "emd/grasp_execution/grasp_execution.hpp"

namespace grasp_execution
{

bool GraspExecutionInterface::execute_plan(
  const std::string & action_description,
  const std::string & id,
  const std::string & planning_group)
{
  prompt_job_start(this->node_->get_logger(), id, action_description);

  bool result = squash_and_execute(planning_group);

  prompt_job_end(this->node_->get_logger(), result);

  // this->arms_[planning_group].traj.clear();

  // {    // Lock PlanningScene
  //   planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
  // }

  if (!result) {
    sleep(0.5);
    return false;
  }
  sleep(0.5);
  return true;
}

bool GraspExecutionInterface::plan_and_execute_job(
  const GraspExecutionContext & option,
  const std::string action_description,
  const std::string target_id,
  const geometry_msgs::msg::PoseStamped & target_pose)
{
  bool result = false;
  prompt_job_start(
    node_->get_logger(), target_id,
    "Plan to" + action_description);

  geometry_msgs::msg::PoseStamped copy_pose;
  to_frame(target_pose, copy_pose, robot_frame_);


  result = this->move_to(
    option,
    copy_pose,
    false);

  prompt_job_end(node_->get_logger(), result);

  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Plan to " + action_description + " failed!");
    return false;
  }

  // ------------------- Move to point above SKU --------------------------
  if (!execute_plan("Move to " + action_description, target_id, option.planning_group)) {
    RCLCPP_ERROR(node_->get_logger(), "Move to " + action_description + " failed!");
    return false;
  }
  return true;
}

bool GraspExecutionInterface::plan_and_execute_collision_job(
  const GraspExecutionContext & option,
  const std::string action_description,
  const std::string target_id,
  char axis,
  const geometry_msgs::msg::PoseStamped & target_pose)
{
  bool result = false;
  prompt_job_start(node_->get_logger(), target_id, "Plan to " + action_description);

  result = move_until_before_collide(
    option.planning_group, target_pose, option.ee_link,
    option.move_to_collide_step_size, static_cast<int>(option.clearance / 0.005), axis, false);
  prompt_job_end(node_->get_logger(), result);

  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Plan to " + action_description + " failed!");
    return false;
  }

  // ------------------- Move to point above SKU --------------------------
  if (!execute_plan("Move to " + action_description, target_id, option.planning_group)) {
    RCLCPP_ERROR(node_->get_logger(), "Move to " + action_description + " failed!");
    return false;
  }
  return true;
}
}  // namespace grasp_execution
