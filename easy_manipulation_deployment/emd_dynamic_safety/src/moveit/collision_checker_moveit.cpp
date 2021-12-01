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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "emd/dynamic_safety/collision_checker_moveit.hpp"
#include "moveit/collision_detection_fcl/collision_detector_allocator_fcl.h"

// Commented out due to library confliction with tesseract
#ifndef END_DYNAMIC_SAFETY_TESSERACT
#include "moveit/collision_detection_bullet/collision_detector_allocator_bullet.h"
#endif

namespace dynamic_safety_moveit
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dynamic_safety_moveit.collision_checker");

MoveitCollisionCheckerContext::MoveitCollisionCheckerContext(
  const std::string & robot_urdf,
  const std::string & robot_srdf,
  const std::string & collision_checking_plugin)
: dynamic_safety::CollisionCheckerContext(robot_urdf, robot_srdf, collision_checking_plugin)
{
  urdf::ModelSharedPtr umodel = std::make_shared<urdf::Model>();
  srdf::ModelSharedPtr smodel = std::make_shared<srdf::Model>();
  if (umodel->initString(robot_urdf)) {
    if (!smodel->initString(*umodel, robot_srdf)) {
      RCLCPP_ERROR(LOGGER, "Unable to parse SRDF");
      // TODO(anyone): exception handling
    }
  } else {
    RCLCPP_ERROR(LOGGER, "Unable to parse URDF");
    // TODO(anyone): exception handling
  }
  // Construct planning scene
  scene_ = std::make_shared<planning_scene::PlanningScene>(umodel, smodel);

  // Load collision_checking_plugin
  // TODO(anyone): use the mapping file
  // auto loader = pluginlib::ClassLoader<collision_detection::CollisionPlugin>(
  //     "moveit_core", "collision_detection::CollisionPlugin");
  auto to_all_lower = [](std::string in) -> std::string {
      std::transform(in.begin(), in.end(), in.begin(), ::tolower);
      return in;
    };

  std::string plugin_name;
  // TODO(anyone): use plugin loader after release of fix
  //               https://github.com/ros-planning/moveit2/pull/658
  if (to_all_lower(collision_checking_plugin) == "fcl") {
    scene_->allocateCollisionDetector(
      collision_detection::CollisionDetectorAllocatorFCL::create()
    );
  } else if (to_all_lower(collision_checking_plugin) == "bullet") {
#ifndef EMD_DYNAMIC_SAFETY_TESSERACT
    scene_->allocateCollisionDetector(
      collision_detection::CollisionDetectorAllocatorBullet::create()
    );
#endif
  }
}

void MoveitCollisionCheckerContext::configure(
  const dynamic_safety::CollisionCheckerOption & option)
{
  collision_request_.group_name = option.group;
  collision_request_.distance = option.distance;
  collision_request_.contacts = true;
}

void MoveitCollisionCheckerContext::run_discrete(
  std::vector<std::string> joint_names,
  trajectory_msgs::msg::JointTrajectoryPoint point,
  uint8_t & result, double & distance)
{
  moveit::core::RobotState current_state = scene_->getCurrentState();

  // Update state
  for (size_t i = 0; i < joint_names.size(); i++) {
    // TODO(anyone): multi-axis joint
    current_state.setJointPositions(joint_names[i], {point.positions[i]});
  }
  // Check robot collision
  current_state.updateCollisionBodyTransforms();
  // scene_->checkCollision(collision_request_, collision_result_, state);
  result = false;
  collision_result_.clear();
  scene_->getCollisionEnv()->checkRobotCollision(
    collision_request_, collision_result_, current_state, scene_->getAllowedCollisionMatrix());
  distance = collision_result_.distance;
  result |= collision_result_.collision;
  // collision_result_.print();

  collision_result_.clear();

  scene_->getCollisionEnvUnpadded()->checkSelfCollision(
    collision_request_, collision_result_, current_state, scene_->getAllowedCollisionMatrix());

  distance = std::min<double>(distance, collision_result_.distance);
  result |= collision_result_.collision;
  // collision_result_.print();

  collision_result_.clear();
}

void MoveitCollisionCheckerContext::run_continuous(
  std::vector<std::string> joint_names,
  trajectory_msgs::msg::JointTrajectoryPoint point1,
  trajectory_msgs::msg::JointTrajectoryPoint point2,
  uint8_t & result, double & distance)
{
  moveit::core::RobotState start_state = scene_->getCurrentState();
  moveit::core::RobotState end_state = scene_->getCurrentState();

  // Update state
  for (size_t i = 0; i < joint_names.size(); i++) {
    // TODO(anyone): multi-axis joint
    start_state.setJointPositions(joint_names[i], {point1.positions[i]});
    end_state.setJointPositions(joint_names[i], {point2.positions[i]});
  }

  // Check robot collision
  start_state.updateCollisionBodyTransforms();
  end_state.updateCollisionBodyTransforms();
  result = false;
  collision_result_.clear();
  scene_->getCollisionEnv()->checkRobotCollision(
    collision_request_, collision_result_, start_state, end_state,
    scene_->getAllowedCollisionMatrix());
  distance = collision_result_.distance;
  result |= collision_result_.collision;
  // collision_result_.print();

  collision_result_.clear();
  scene_->getCollisionEnvUnpadded()->checkSelfCollision(
    collision_request_, collision_result_, start_state, scene_->getAllowedCollisionMatrix());

  distance = std::min<double>(distance, collision_result_.distance);
  result |= collision_result_.collision;
  // collision_result_.print();

  collision_result_.clear();
}

void MoveitCollisionCheckerContext::update(const sensor_msgs::msg::JointState & joint_states)
{
  auto & current_state = scene_->getCurrentStateNonConst();
  // Update state
  for (size_t i = 0; i < joint_states.name.size(); i++) {
    // TODO(anyone): multi-axis joint
    if (current_state.getJointModel(joint_states.name[i])) {
      current_state.setJointPositions(joint_states.name[i], {joint_states.position[i]});
    }
  }
}

void MoveitCollisionCheckerContext::update(const moveit_msgs::msg::PlanningScene & scene_msgs)
{
  scene_->processPlanningSceneWorldMsg(scene_msgs.world);
}

}  // namespace dynamic_safety_moveit
