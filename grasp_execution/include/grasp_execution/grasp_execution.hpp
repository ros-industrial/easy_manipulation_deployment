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

#ifndef GRASP_EXECUTION__GRASP_EXECUTION_HPP_
#define GRASP_EXECUTION__GRASP_EXECUTION_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"

#include "grasp_planning/msg/grasp_pose.hpp"

namespace grasp_execution
{

class GraspExecution
{
public:
  using PlannerT = moveit::planning_interface::PlanningComponent;
  using PlannerPtrT = moveit::planning_interface::PlanningComponentPtr;

  explicit GraspExecution(const rclcpp::Node::SharedPtr & node);

  ~GraspExecution();

  bool init(const std::string & planning_group, const std::string & _ee_link = "");

  void workflow(
    grasp_planning::msg::GraspPose::UniquePtr msg,
    std::promise<bool> && _sig);

  geometry_msgs::msg::Pose get_object_pose(const std::string & object_id) const;

  geometry_msgs::msg::PoseStamped get_curr_pose(const std::string & link_name) const;

  bool move_to(const geometry_msgs::msg::PoseStamped & pose, const std::string & link);

  bool move_to(const moveit::core::RobotState & state);

  bool cartesian_to(
    const std::vector<geometry_msgs::msg::Pose> & _waypoints,
    const std::string & _link, double step, double jump_threshold = 0);

  bool move_until_before_collide(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & link, double step_size, int max_attempts,
    char axis);

  void attach_object_to_ee(const moveit_msgs::msg::CollisionObject & object);

  void detach_object_to_ee(const moveit_msgs::msg::CollisionObject & object);

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<grasp_planning::msg::GraspPose>::SharedPtr grasp_planning_sub_;

  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
  std::unordered_map<std::string, PlannerPtrT> arms_;

  std::shared_ptr<std::thread> execution_thread_ptr_;
  std::future<bool> execution_thread_future_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string robot_frame_;
  std::string ee_frame_;
  std::string ee_link_;

  std::vector<std::string> ignore_links_;
};

}  // namespace grasp_execution

#endif  // GRASP_EXECUTION__GRASP_EXECUTION_HPP_
