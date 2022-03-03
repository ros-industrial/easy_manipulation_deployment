// Copyright 2022 ROS Industrial Consortium Asia Pacific
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
#include <filesystem>
#include <fstream>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "json/json.h"
#include "rclcpp/rclcpp.hpp"

#include "emd/waypoint_execution/waypoint.hpp"
#include "emd/grasp_execution/moveit2/moveit_cpp_if.hpp"
#include "emd/end_effector/ee_execution_interface.hpp"

#ifndef WAYPOINT_PARSER__HPP_
#define WAYPOINT_PARSER__HPP_

static const rclcpp::Logger & LOGGER_WAYPOINT = rclcpp::get_logger("waypoint_parser");
namespace emd
{
namespace WayPointParser
{
namespace internal
{
bool add_rpy_to_pose(
  geometry_msgs::msg::PoseStamped & target_pose,
  float roll,
  float pitch,
  float yaw);

bool get_json_offset(Json::Value json_object, emd::WayPoint & waypoint);
}


bool load_waypoints(
  std::vector<emd::WayPoint> & waypoints,
  const std::string waypoint_filepath);

template<typename V, typename T, typename U>
bool process_waypoint(
  V execution_interface,
  U end_effector_interface,
  T execution_interface_options,
  const emd::WayPoint & waypoint,
  const std::string & target_id)
{
  using type = std::remove_pointer_t<V>;
  static_assert(
    std::is_base_of<grasp_execution::moveit2::MoveitCppGraspExecution, type>::value,
    "Execution_interface should inherit from grasp_execution::GraspExecutionInterface");
  static_assert(
    std::is_base_of<emd::EndEffectorExecutioninterface, std::remove_pointer_t<U>>::value,
    "End Effector should inherit from emd::EndEffectorExecutioninterface");
  static_assert(
    std::is_base_of<grasp_execution::GraspExecutionContext, T>::value,
    "Execution_interface_options should inherit from grasp_execution::GraspExecutionContext");
  auto target_pose = execution_interface->get_curr_pose(
    execution_interface_options.ee_link);

  if (waypoint.is_relative) {
    target_pose.pose.position.x += waypoint.position[0];
    target_pose.pose.position.y += waypoint.position[1];
    target_pose.pose.position.z += waypoint.position[2];

    if (!emd::WayPointParser::internal::add_rpy_to_pose(
        target_pose,
        waypoint.rpy[0],
        waypoint.rpy[1],
        waypoint.rpy[2]))
    {
      return false;
    }
  } else {
    target_pose.pose.orientation.x = 0;
    target_pose.pose.orientation.y = 0;
    target_pose.pose.orientation.z = 0;
    target_pose.pose.orientation.w = 1;
    target_pose.pose.position.x = waypoint.position[0];
    target_pose.pose.position.y = waypoint.position[1];
    target_pose.pose.position.z = waypoint.position[2];

    if (!emd::WayPointParser::internal::add_rpy_to_pose(
        target_pose,
        waypoint.rpy[0],
        waypoint.rpy[1],
        waypoint.rpy[2]))
    {
      return false;
    }
  }

  if (waypoint.action_type.compare("move") == 0) {

    // auto target_pose = execution_interface->get_curr_pose(
    //   execution_interface_options.ee_link);

    // target_pose.pose.position.x += waypoint.position[0];
    // target_pose.pose.position.y += waypoint.position[1];
    // target_pose.pose.position.z += waypoint.position[2];

    // if (!add_rpy_to_pose(
    //     target_pose,
    //     waypoint.rpy[0],
    //     waypoint.rpy[1],
    //     waypoint.rpy[2]))
    // {
    //   return false;
    // }

    if (!execution_interface->plan_and_execute_job(
        execution_interface_options,
        waypoint.action_description,
        target_id,
        target_pose))
    {
      RCLCPP_ERROR(LOGGER_WAYPOINT, "Planning Error");
      return false;
    }
  } else if (waypoint.action_type.compare("collision") == 0) {

    char collision_axis;
    if (waypoint.collision_axis_direction == std::vector<int>{1, 0, 0}) {
      collision_axis = 'x';
    } else if (waypoint.collision_axis_direction == std::vector<int>{0, 1, 0}) {
      collision_axis = 'y';
    } else if (waypoint.collision_axis_direction == std::vector<int>{0, 0, 1}) {
      collision_axis = 'z';
    } else {
      return false;
    }

    if (!execution_interface->plan_and_execute_collision_job(
        execution_interface_options,
        waypoint.action_description,
        target_id,
        collision_axis,
        target_pose))
    {
      RCLCPP_ERROR(LOGGER_WAYPOINT, "Collision Planning Error");
      return false;
    }
  } else if (waypoint.action_type.compare("grasp") == 0) {
    if (!end_effector_interface.grasp_object(
        execution_interface,
        execution_interface_options.ee_link,
        target_id))
    {
      RCLCPP_ERROR(LOGGER_WAYPOINT, "Error Grasping Object");
      return false;
    }

  } else if (waypoint.action_type.compare("release") == 0) {
    if (!end_effector_interface.release_object(
        execution_interface,
        execution_interface_options.ee_link,
        target_id))
    {
      RCLCPP_ERROR(LOGGER_WAYPOINT, "Error Releasing Object");
      return false;
    }
  } else {
    RCLCPP_ERROR(LOGGER_WAYPOINT, "Wrong Action Type");
    return false;
  }
  return true;
}

template<typename V, typename T, typename U>
bool process_waypoints(
  V execution_interface,
  U end_effector_interface,
  T execution_interface_options,
  const std::vector<emd::WayPoint> & waypoints,
  const std::string & target_id)
{
  for (auto waypoint : waypoints) {
    if (!process_waypoint(
        execution_interface,
        end_effector_interface,
        execution_interface_options,
        waypoint,
        target_id))
    {
      return false;
    }
  }
  return true;
}

}
}

#endif // WAYPOINT_PARSER__HPP_
