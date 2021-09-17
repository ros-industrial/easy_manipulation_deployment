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

#include <string>

#include "emd/grasp_execution/moveit2/executor/default_executor.hpp"

namespace grasp_execution
{

namespace moveit2
{

bool DefaultExecutor::load(
  const moveit_cpp::MoveItCppPtr & moveit_cpp,
  const std::string & /*name*/)
{
  trajectory_execution_manager_ = moveit_cpp->getTrajectoryExecutionManagerNonConst();
  // Force explicit bool operator
  return trajectory_execution_manager_ ? true : false;
}

// Referenced from moveit_cpp.cpp
// https://github.com/ros-planning/moveit2/blob/2499a72f7388a371905eaef72685fcfaae04335a/moveit_ros/planning_interface/moveit_cpp/src/moveit_cpp.cpp#L270-L297
bool DefaultExecutor::run(
  const robot_trajectory::RobotTrajectory & robot_trajectory)
{
  const auto & group_name = robot_trajectory.getGroupName();

  // Check if there are controllers that can handle the execution
  if (!trajectory_execution_manager_->ensureActiveControllersForGroup(group_name)) {
    RCLCPP_ERROR(
      this->logger_,
      "Execution failed! No active controllers configured for group '%s'",
      group_name.c_str());
    return false;
  }

  // Execute trajectory
  moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
  robot_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);

  trajectory_execution_manager_->push(robot_trajectory_msg);
  trajectory_execution_manager_->execute();

  // ALWAYS BLOCKING !!!
  auto status = trajectory_execution_manager_->waitForExecution();

  // Allow timeout
  // TODO(anyone): fix doesn't finish in time problem.
  return (status == moveit_controller_manager::ExecutionStatus::SUCCEEDED) ||
         (status == moveit_controller_manager::ExecutionStatus::TIMED_OUT);
}

}  // namespace moveit2

}  // namespace grasp_execution

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  grasp_execution::moveit2::DefaultExecutor, grasp_execution::moveit2::Executor)
