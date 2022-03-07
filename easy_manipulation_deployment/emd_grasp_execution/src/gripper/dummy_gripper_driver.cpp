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

#include "emd/grasp_execution/gripper/dummy_gripper_driver.hpp"

namespace grasp_execution
{

namespace gripper
{

bool DummyGripperDriver::load(
  const std::string & /*name*/)
{
  RCLCPP_INFO(this->logger_, "Loading dummy gripper driver, sleep 1s.....");
  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(this->logger_, "Dummy gripper driver loaded");
  return true;
}

bool DummyGripperDriver::activate()
{
  RCLCPP_INFO(this->logger_, "Activate dummy gripper driver, sleep 2s.....");
  rclcpp::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(this->logger_, "Dummy gripper driver activated");
  return true;
}

bool DummyGripperDriver::deactivate()
{
  RCLCPP_INFO(this->logger_, "Deactivate dummy gripper driver, sleep 2s.....");
  rclcpp::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(this->logger_, "Dummy gripper driver deactivated");
  return true;
}

}  // namespace gripper

}  // namespace grasp_execution

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  grasp_execution::gripper::DummyGripperDriver, grasp_execution::gripper::GripperDriver)
