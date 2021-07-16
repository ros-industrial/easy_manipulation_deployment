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
#include <string>
#include <vector>

#include "grasp_execution/dynamic_safety/dynamic_safety.hpp"
#include "grasp_execution/moveit2/executor.hpp"
#include "moveit/kinematic_constraints/utils.h"
#include "moveit/robot_state/conversions.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"

namespace grasp_execution
{

namespace moveit2
{

class DynamicSafetyAsyncExecutor : public grasp_execution::moveit2::Executor
{
public:
  DynamicSafetyAsyncExecutor()
  : Executor(),
    is_configured_(false),
    logger_(rclcpp::get_logger("dynamic_safety_async_executor"))
  {
  }

  ~DynamicSafetyAsyncExecutor() {}

  bool load(
    const moveit_cpp::MoveItCppPtr & moveit_cpp,
    const std::string & name) override
  {
    this->main_context_ = moveit_cpp;
    auto option = grasp_execution::dynamic_safety::Option().load(main_context_->getNode());
    const auto & command_type = option.next_point_publisher_options.command_out_type;
    if (command_type == "trajectory_msgs/JointTrajectory") {
      option.next_point_publisher_options.command_out_topic =
        name + "/joint_trajectory";
    } else {  // if (command_type == "std_msgs/Float64MultiArray"){
      option.next_point_publisher_options.command_out_topic =
        name + "/command";
    }
    safety_officer_ = std::make_unique<grasp_execution::dynamic_safety::DynamicSafety>(option);

    return true;
  }

  void configure(
    const robot_trajectory::RobotTrajectory & robot_trajectory) override
  {
    safety_officer_->configure(
      main_context_->getPlanningSceneMonitor()->getPlanningScene(),
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_trajectory),
      main_context_->getNode());
    is_configured_ = true;
  }

  bool run(
    const robot_trajectory::RobotTrajectory & robot_trajectory) override
  {
    if (!is_configured_) {
      configure(robot_trajectory);
    }
    safety_officer_->start();

    safety_officer_->wait();

    is_configured_ = false;

    return true;
  }

private:
  const rclcpp::Logger logger_;

  moveit_cpp::MoveItCppPtr main_context_;

  grasp_execution::dynamic_safety::DynamicSafety::UniquePtr safety_officer_;

  bool is_configured_;
};

}  // namespace moveit2

}  // namespace grasp_execution

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  grasp_execution::moveit2::DynamicSafetyAsyncExecutor, grasp_execution::moveit2::Executor)
