// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
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

#ifndef EMD__EE_EXECUTION_INTERFACE_HPP_
#define EMD__EE_EXECUTION_INTERFACE_HPP_


static const rclcpp::Logger & LOGGER_EE_INTERFACE = rclcpp::get_logger(
  "end_effector_execution_interface");
namespace emd
{
struct EndEffectorExecutionContext
{
};

class EndEffectorExecutioninterface
{
public:
  EndEffectorExecutioninterface(){}
  ~EndEffectorExecutioninterface() {}
  template<typename V>
  bool grasp_object(
    V execution_interface,
    const std::string ee_link,
    const std::string target_id)
  {
    using type = std::remove_pointer_t<V>;
    static_assert(
      std::is_base_of<grasp_execution::moveit2::MoveitCppGraspExecution, type>::value,
      "Execution_interface should inherit from grasp_execution::GraspExecutionInterface");
    // ------------------- Attach grasp object to robot --------------------------
    RCLCPP_INFO(LOGGER_EE_INTERFACE, "Attaching to robot ee frame: [" + ee_link + "]");

    execution_interface->attach_object_to_ee(target_id, ee_link);
    return true;
  }
  template<typename V>
  bool release_object(
    V execution_interface,
    const std::string ee_link,
    const std::string target_id)
  {
    using type = std::remove_pointer_t<V>;
    static_assert(
      std::is_base_of<grasp_execution::moveit2::MoveitCppGraspExecution, type>::value,
      "Execution_interface should inherit from grasp_execution::GraspExecutionInterface");

    RCLCPP_INFO(LOGGER_EE_INTERFACE, "Detaching from robot ee frame: [" + ee_link + "]");
    execution_interface->detach_object_from_ee(target_id, ee_link);

    return true;
  }
};
}

#endif  // EMD__EE_EXECUTION_INTERFACE_HPP_
