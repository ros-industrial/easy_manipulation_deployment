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

#ifndef GRASP_EXECUTION__DYNAMIC_SAFETY__REPLANNER_HPP_
#define GRASP_EXECUTION__DYNAMIC_SAFETY__REPLANNER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "moveit/planning_pipeline/planning_pipeline.h"
#include "moveit/kinematic_constraints/utils.h"
#include "moveit/robot_state/conversions.h"

namespace grasp_execution
{

namespace dynamic_safety
{

class Replanner
{
public:
  struct Option
  {
    std::string planner_name;
  };

  void configure(
    const planning_scene::PlanningScenePtr & scene,
    const robot_trajectory::RobotTrajectoryPtr & rt,
    const rclcpp::Node::SharedPtr & node,
    const Option & option);

  void start(
    double start_time_point,
    double deadline);

  void update(
    const sensor_msgs::msg::JointState::SharedPtr & msg);

  bool started() const
  {
    return static_cast<bool>(started_);
  }

  bool get_result() const
  {
    return static_cast<bool>(result_);
  }

  void reset();

  const robot_trajectory::RobotTrajectoryPtr & get_trajectory() const
  {
    return current_traj_;
  }

private:
  void _replan(double start_time_point);

  void _replan_internal(
    double start_time_point,
    std::promise<bool> && sig);

  robot_trajectory::RobotTrajectoryPtr current_traj_;
  moveit::core::RobotStatePtr start_state_;
  moveit::core::RobotStatePtr current_state_;
  planning_scene::PlanningScenePtr scene_;

  planning_pipeline::PlanningPipelinePtr planner_;

  planning_interface::MotionPlanRequest req_;
  planning_interface::MotionPlanResponse res_;

  std::shared_ptr<std::thread> planner_thr_;
  std::shared_ptr<std::thread> planner_monitor_thr_;

  double deadline_;

  std::atomic_bool started_;
  std::atomic_bool result_;
};

}  // namespace dynamic_safety

}  // namespace grasp_execution

#endif  // GRASP_EXECUTION__DYNAMIC_SAFETY__REPLANNER_HPP_
