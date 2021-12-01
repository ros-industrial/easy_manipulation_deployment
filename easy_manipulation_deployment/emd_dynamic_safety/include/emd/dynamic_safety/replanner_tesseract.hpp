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

#ifndef EMD__DYNAMIC_SAFETY__REPLANNER_TESSERACT_HPP_
#define EMD__DYNAMIC_SAFETY__REPLANNER_TESSERACT_HPP_

#include <string>
#include <vector>

#include "emd/dynamic_safety/replanner_common.hpp"
#include "tesseract_environment/core/environment.h"
#include "tesseract_motion_planners/core/types.h"

// OMPL
#include "tesseract_motion_planners/ompl/ompl_motion_planner.h"

// Trajopt
#include "tesseract_motion_planners/trajopt/trajopt_motion_planner.h"

// TrajoptIFOPT
#include "tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_motion_planner.h"

// Time parameterization
#include "tesseract_time_parameterization/iterative_spline_parameterization.h"
#include "tesseract_time_parameterization/time_optimal_trajectory_generation.h"

namespace dynamic_safety_tesseract
{

class TesseractReplannerContext : public dynamic_safety::ReplannerContext
{
public:
  TesseractReplannerContext() {}

  virtual ~TesseractReplannerContext() {}

  TesseractReplannerContext(
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
    trajectory_msgs::msg::JointTrajectory & plan,
    double scale = 1.0) override;

  void update(
    const sensor_msgs::msg::JointState & joint_states) override;

  void update(
    const moveit_msgs::msg::PlanningScene &) override {}

protected:
  trajectory_msgs::msg::JointTrajectory to_joint_trajectory_msg(
    const tesseract_planning::CompositeInstruction & result);

  void to_composite_instructions(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    tesseract_planning::CompositeInstruction & program);

  tesseract_environment::Environment::Ptr env_;
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin_;
  tesseract_kinematics::InverseKinematics::Ptr inv_kin_;
  tesseract_planning::PlannerRequest planning_request_;
  tesseract_planning::ManipulatorInfo manip_;

  // OMPL
  tesseract_planning::OMPLMotionPlanner ompl_planner_;

  // TrajOpt
  tesseract_planning::TrajOptMotionPlanner trajopt_planner_;

  // TrajOpt-IFOPT
  tesseract_planning::TrajOptIfoptMotionPlanner trajopt_ifopt_planner_;

  // Time parameterization
  tesseract_planning::IterativeSplineParameterization isp_;
  tesseract_planning::TimeOptimalTrajectoryGeneration totg_;

  std::string time_parameterization_;
  std::string planner_;

  std::vector<double> joint_vel_limits_;
  std::vector<double> joint_accel_limits_;
};
}  // namespace dynamic_safety_tesseract

#endif  // EMD__DYNAMIC_SAFETY__REPLANNER_TESSERACT_HPP_
