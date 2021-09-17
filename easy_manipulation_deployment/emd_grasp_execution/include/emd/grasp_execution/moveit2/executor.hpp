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

#ifndef EMD__GRASP_EXECUTION__MOVEIT2__EXECUTOR_HPP_
#define EMD__GRASP_EXECUTION__MOVEIT2__EXECUTOR_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/macros.hpp"
#include "moveit/moveit_cpp/moveit_cpp.h"

namespace grasp_execution
{

namespace moveit2
{

class Executor
{
public:
  // cppcheck-suppress unknownMacro
  RCLCPP_SMART_PTR_DEFINITIONS(Executor)

  Executor() {}

  virtual bool load(
    const moveit_cpp::MoveItCppPtr & moveit_cpp,
    const std::string & name) = 0;

  ~Executor() {}

  virtual void configure(
    const robot_trajectory::RobotTrajectory &) {}

  virtual bool run(
    const robot_trajectory::RobotTrajectory &) = 0;

private:
  // cppcheck-suppress unknownMacro
  RCLCPP_DISABLE_COPY(Executor)
};

}  // namespace moveit2

}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__MOVEIT2__EXECUTOR_HPP_
