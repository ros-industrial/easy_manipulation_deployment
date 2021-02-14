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

#include "grasp_execution/grasp_execution.hpp"

namespace grasp_execution
{

bool GraspExecutionInterface::default_plan_pre_grasp(
  const std::string & planning_group,
  const std::string & ee_link,
  const geometry_msgs::msg::PoseStamped & grasp_pose,
  double clearance)
{
  if (clearance < 0) {
    return false;
  }

  // Move pre grasp pose in the length of clearance
  // wrt the grasp_ee frame's -z axis direction
  geometry_msgs::msg::PoseStamped pre_grasp_pose;
  to_frame(grasp_pose, pre_grasp_pose, robot_frame_);

  tf2::Transform base_to_ee;
  tf2::fromMsg(pre_grasp_pose.pose, base_to_ee);

  tf2::Transform ee_w_clearance;
  ee_w_clearance.setIdentity();
  ee_w_clearance.setOrigin(tf2::Vector3(0, 0, -clearance));

  tf2::toMsg(base_to_ee * ee_w_clearance, pre_grasp_pose.pose);

  bool result = move_to(planning_group, pre_grasp_pose, ee_link, false);

  if (!result) {
    return false;
  }

  result = move_until_before_collide(
    planning_group, pre_grasp_pose, ee_link,
    0.01, static_cast<int>(clearance / 0.01), 'z', false);

  return result;
}

bool GraspExecutionInterface::default_plan_transport(
  const std::string & planning_group,
  const std::string & ee_link,
  const geometry_msgs::msg::PoseStamped & release_pose,
  double clearance)
{
  // Move pre grasp pose / current pose in the length of clearance
  // wrt the grasp_ee frame's -z axis direction
  auto post_grasp_pose = get_curr_pose(ee_link);

  tf2::Transform base_to_ee;
  tf2::fromMsg(post_grasp_pose.pose, base_to_ee);

  tf2::Transform ee_w_clearance;
  ee_w_clearance.setIdentity();
  ee_w_clearance.setOrigin(tf2::Vector3(0, 0, -clearance));

  tf2::toMsg(base_to_ee * ee_w_clearance, post_grasp_pose.pose);

  bool result = cartesian_to(
    planning_group,
    {post_grasp_pose.pose}, ee_link, 0.01, 0, false);

  if (!result) {
    return false;
  }

  // Move pre grasp pose / current pose in the length of clearance
  // wrt the grasp_ee frame's -z axis direction
  geometry_msgs::msg::PoseStamped pre_release_pose;
  to_frame(release_pose, pre_release_pose, robot_frame_);

  tf2::fromMsg(pre_release_pose.pose, base_to_ee);

  ee_w_clearance.setIdentity();
  ee_w_clearance.setOrigin(tf2::Vector3(0, 0, -clearance));

  tf2::toMsg(base_to_ee * ee_w_clearance, pre_release_pose.pose);

  result = move_to(
    planning_group, pre_release_pose, ee_link, false);

  if (!result) {
    return false;
  }

  result = move_until_before_collide(
    planning_group, pre_release_pose, ee_link,
    0.01, static_cast<int>(clearance / 0.01), 'z', false);

  return result;
}

bool GraspExecutionInterface::default_plan_post_release(
  const std::string & planning_group,
  const std::string & ee_link,
  bool home,
  double clearance)
{
  if (clearance < 0) {
    return false;
  }

  auto release_pose = get_curr_pose(ee_link);

  // Move pre grasp pose in the length of clearance
  // wrt the grasp_ee frame's -z axis direction
  geometry_msgs::msg::PoseStamped post_release_pose;
  to_frame(release_pose, post_release_pose, robot_frame_);

  tf2::Transform base_to_ee;
  tf2::fromMsg(post_release_pose.pose, base_to_ee);

  tf2::Transform ee_w_clearance;
  ee_w_clearance.setIdentity();
  ee_w_clearance.setOrigin(tf2::Vector3(0, 0, -clearance));

  tf2::toMsg(base_to_ee * ee_w_clearance, post_release_pose.pose);

  bool result = cartesian_to(
    planning_group,
    {post_release_pose.pose}, ee_link, 0.01, 0, false);

  if (!result) {
    return false;
  }

  if (home) {
    result = move_to(planning_group, "home", false);
  }

  return result;
}

}  // namespace grasp_execution
