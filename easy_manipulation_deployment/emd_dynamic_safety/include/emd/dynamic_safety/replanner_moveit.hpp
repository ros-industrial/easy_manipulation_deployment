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

#ifndef EMD__DYNAMIC_SAFETY__REPLANNER_MOVEIT_HPP_
#define EMD__DYNAMIC_SAFETY__REPLANNER_MOVEIT_HPP_

#include <string>
#include <vector>

#include "emd/dynamic_safety/replanner_common.hpp"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/planning_interface/planning_interface.h"
#include "moveit/kinematic_constraints/utils.h"

#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit/trajectory_processing/iterative_spline_parameterization.h"

namespace dynamic_safety_moveit
{

class MoveitReplannerContext : public dynamic_safety::ReplannerContext
{
public:
  MoveitReplannerContext() {}

  virtual ~MoveitReplannerContext() {}

  MoveitReplannerContext(
    const std::string & robot_urdf,
    const std::string & robot_srdf,
    const dynamic_safety::ReplannerOption & option,
    const rclcpp::Node::SharedPtr & node);

  void run(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint & start_point,
    const trajectory_msgs::msg::JointTrajectoryPoint & end_point,
    trajectory_msgs::msg::JointTrajectory & plan) override;

  bool time_parameterize(
    trajectory_msgs::msg::JointTrajectory & plan, double scale = 1.0) override;

  void update(
    const sensor_msgs::msg::JointState & joint_states) override;

  void update(
    const moveit_msgs::msg::PlanningScene & scene_msg) override;

protected:
  bool _time_parameterization(robot_trajectory::RobotTrajectory & trajectory, double scale = 1.0);

  planning_scene::PlanningScenePtr scene_;
  std::mutex scene_mtx_;
  planning_interface::PlannerManagerPtr planning_manager_;
  planning_interface::MotionPlanRequest planning_request_;
  std::string time_parameterization_;
  trajectory_processing::IterativeParabolicTimeParameterization iptp_;
  trajectory_processing::IterativeSplineParameterization isp_;
  trajectory_processing::TimeOptimalTrajectoryGeneration totg_;
};
}  // namespace dynamic_safety_moveit

#endif  // EMD__DYNAMIC_SAFETY__REPLANNER_MOVEIT_HPP_
