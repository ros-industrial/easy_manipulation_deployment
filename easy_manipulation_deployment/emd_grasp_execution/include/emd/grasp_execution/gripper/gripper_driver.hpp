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

#ifndef EMD__GRASP_EXECUTION__GRIPPER__GRIPPER_DRIVER_HPP_
#define EMD__GRASP_EXECUTION__GRIPPER__GRIPPER_DRIVER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace grasp_execution
{

namespace gripper
{

class GripperDriver
{
public:
  // cppcheck-suppress unknownMacro
  RCLCPP_SMART_PTR_DEFINITIONS(GripperDriver)

  GripperDriver() {}

  // TODO(anyone): Harden gripper driver interface
  virtual bool load(const std::string & name) = 0;

  ~GripperDriver() {}

  // TODO(anyone): Harden gripper driver interface
  virtual bool activate() = 0;

  // TODO(anyone): Harden gripper driver interface
  virtual bool deactivate() = 0;

private:
  // cppcheck-suppress unknownMacro
  RCLCPP_DISABLE_COPY(GripperDriver)
};

}  // namespace gripper

}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__GRIPPER__GRIPPER_DRIVER_HPP_
