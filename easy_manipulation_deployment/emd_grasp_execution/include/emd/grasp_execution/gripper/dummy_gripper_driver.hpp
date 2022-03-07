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

#ifndef EMD__GRASP_EXECUTION__GRIPPER__DUMMY_GRIPPER_DRIVER_HPP_
#define EMD__GRASP_EXECUTION__GRIPPER__DUMMY_GRIPPER_DRIVER_HPP_

#include <string>

#include "emd/grasp_execution/gripper/gripper_driver.hpp"

namespace grasp_execution
{

namespace gripper
{

class DummyGripperDriver : public GripperDriver
{
public:
  DummyGripperDriver()
  : GripperDriver(),
    logger_(rclcpp::get_logger("dummy_gripper_driver")) {}

  ~DummyGripperDriver() {}

  bool load(const std::string & /*name*/) override;

  bool activate() override;

  bool deactivate() override;

private:
  const rclcpp::Logger logger_;
};

}  // namespace gripper

}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__GRIPPER__DUMMY_GRIPPER_DRIVER_HPP_
