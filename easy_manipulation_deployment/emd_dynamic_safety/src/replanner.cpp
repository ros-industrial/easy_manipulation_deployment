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

#include <memory>
#include <utility>

#include "emd/dynamic_safety/replanner.hpp"


namespace dynamic_safety
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dynamic_safety.replanner");

void Replanner::configure(
  const planning_scene::PlanningScenePtr & scene,
  const robot_trajectory::RobotTrajectoryPtr & rt,
  const rclcpp::Node::SharedPtr & node,
  const Option & option)
{
  current_traj_ = std::make_shared<robot_trajectory::RobotTrajectory>(*rt, true);
  start_state_ = std::make_shared<moveit::core::RobotState>(current_traj_->getRobotModel());
  current_state_ = std::make_shared<moveit::core::RobotState>(current_traj_->getRobotModel());

  scene_ = planning_scene::PlanningScene::clone(scene);

  planner_ = std::make_shared<planning_pipeline::PlanningPipeline>(
    current_traj_->getRobotModel(),
    node,
    option.planner_name);

  started_ = false;
  result_ = false;
}

void Replanner::start(
  double start_time_point,
  double deadline)
{
  if (planner_monitor_thr_) {
    planner_monitor_thr_->join();
    planner_monitor_thr_.reset();
  }

  planner_monitor_thr_ = std::make_shared<std::thread>(
    &Replanner::_replan,
    this,
    start_time_point);

  deadline_ = deadline;

  started_ = true;
}

void Replanner::update(
  const sensor_msgs::msg::JointState::SharedPtr & msg)
{
  for (size_t i = 0; i < msg->name.size(); i++) {
    // TODO(anyone): multi-axis joint
    current_state_->setJointPositions(msg->name[i], {msg->position[i]});

    const auto & jm = current_state_->getJointModel(msg->name[i]);
    if (!msg->velocity.empty()) {
      current_state_->setJointVelocities(jm, &(msg->velocity[i]));
    }
    if (!msg->effort.empty()) {
      current_state_->setJointEfforts(jm, &(msg->effort[i]));
    }
  }
  scene_->setCurrentState(*current_state_);
}

void Replanner::reset()
{
  started_ = false;
}

void Replanner::_replan(double start_time_point)
{
  std::chrono::time_point<std::chrono::steady_clock> replan_start_time =
    std::chrono::steady_clock::now();

  if (planner_thr_) {
    planner_thr_->join();
    planner_thr_.reset();
  }

  std::promise<bool> sig;
  auto replan_future = sig.get_future();

  planner_thr_ = std::make_shared<std::thread>(
    &Replanner::_replan_internal,
    this,
    start_time_point,
    std::move(sig));

  if (replan_future.wait_until(
      replan_start_time + std::chrono::nanoseconds(
        static_cast<int>(deadline_ * 1e9))) ==
    std::future_status::ready &&
    replan_future.get())
  {
    double time_taken =
      static_cast<double>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - replan_start_time).count()) / 1e9;
    RCLCPP_INFO(LOGGER, "Replanning takes %fs", time_taken);
    result_ = true;
  } else {
    RCLCPP_INFO(LOGGER, "Replanning Didn't finish in time or failed");
    started_ = false;
  }
}


void Replanner::_replan_internal(
  double start_time_point,
  std::promise<bool> && sig)
{
  RCLCPP_INFO(LOGGER, "Replanning started!");
  current_traj_->getStateAtDurationFromStart(start_time_point, start_state_);

  RCLCPP_INFO(LOGGER, "Create start state!");
  // Update the current state group to start state
  const auto & joints = current_traj_->getGroup()->getVariableNames();
  for (size_t i = 0; i < joints.size(); i++) {
    current_state_->setJointPositions(joints[i], start_state_->getJointPositions(joints[i]));
  }
  moveit_msgs::msg::RobotState msg;
  moveit::core::robotStateToRobotStateMsg(*current_state_, msg);
  scene_->setCurrentState(msg);

  RCLCPP_INFO(LOGGER, "Prepare request");
  req_.group_name = current_traj_->getGroupName();
  req_.planner_id = "";
  req_.num_planning_attempts = 1;
  req_.allowed_planning_time = 0.3;
  req_.max_velocity_scaling_factor = 1;
  req_.max_acceleration_scaling_factor = 1;
  req_.goal_constraints = {
    kinematic_constraints::constructGoalConstraints(
      current_traj_->getLastWayPoint(), current_traj_->getGroup())};
  RCLCPP_INFO(LOGGER, "Generate plan");
  planner_->generatePlan(
    scene_, req_, res_);
  bool result = (res_.error_code_.val == res_.error_code_.SUCCESS);
  if (result) {
    current_traj_ =
      std::make_shared<robot_trajectory::RobotTrajectory>(*res_.trajectory_, true);
  }
  sig.set_value(result);
}

}  // namespace dynamic_safety
