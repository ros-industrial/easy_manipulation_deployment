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

#ifndef EMD__GRASP_EXECUTION__MOVEIT2__EXECUTOR__DEFAULT_EXECUTOR_HPP_
#define EMD__GRASP_EXECUTION__MOVEIT2__EXECUTOR__DEFAULT_EXECUTOR_HPP_

#include <string>

#include "emd/grasp_execution/moveit2/executor.hpp"

namespace grasp_execution
{

namespace moveit2
{

class DefaultExecutor : public Executor
{
public:
  DefaultExecutor()
  : Executor(),
    logger_(rclcpp::get_logger("default_executor")) {}

  ~DefaultExecutor() {}

  bool load(
    const moveit_cpp::MoveItCppPtr & moveit_cpp,
    const std::string & /*name*/) override;

  bool run(
    const robot_trajectory::RobotTrajectory & robot_trajectory) override;

private:
  const rclcpp::Logger logger_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
};

}  // namespace moveit2

}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__MOVEIT2__EXECUTOR__DEFAULT_EXECUTOR_HPP_
