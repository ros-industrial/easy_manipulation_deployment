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

#ifndef GRASP_EXECUTION__MOVEIT_CPP_IF_HPP_
#define GRASP_EXECUTION__MOVEIT_CPP_IF_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "grasp_execution/grasp_execution.hpp"

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"

#include "grasp_planning/msg/grasp_pose.hpp"

namespace grasp_execution
{

struct JmgContext
{
  JmgContext() {}

  JmgContext(
    const std::string & _name,
    const moveit::planning_interface::MoveItCppPtr _moveit_cpp,
    const std::string & _ee_link)
  {
    ee_link = _ee_link;
    planner = std::make_shared<moveit::planning_interface::PlanningComponent>(
      _name, _moveit_cpp);
  }

  // TODO(Briancbn): Replace with end-effector abstraction class
  std::string ee_link;
  moveit::planning_interface::PlanningComponentPtr planner;
  std::vector<robot_trajectory::RobotTrajectoryPtr> traj;
};

class MoveitCppGraspExecution : public GraspExecutionInterface
{
public:
  explicit MoveitCppGraspExecution(
    const rclcpp::Node::SharedPtr & node,
    const std::string & grasp_poses_topic = "grasp_poses",
    size_t planning_concurrency = 1,
    size_t execution_concurrency = 0);

  ~MoveitCppGraspExecution();

  bool init(const std::string & planning_group, const std::string & _ee_link = "");

  void planning_workflow(
    const grasp_planning::msg::GraspPose::SharedPtr & msg) override {}

  void execution_workflow(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr & msg) override {}

  void order_schedule(
    const grasp_planning::msg::GraspPose::SharedPtr & msg) override {}

  geometry_msgs::msg::Pose get_object_pose(const std::string & object_id) const override;

  geometry_msgs::msg::PoseStamped get_curr_pose(const std::string & link_name) const override;

  bool move_to(
    const std::string & planning_group,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & link,
    bool execute = true) override;

  bool move_to(
    const std::string & planning_group,
    const moveit::core::RobotState & state,
    bool execute = true);

  bool move_to(
    const std::string & planning_group,
    const sensor_msgs::msg::JointState & state,
    bool execute = true) override {return true;}

  bool cartesian_to(
    const std::string & planning_group,
    const std::vector<geometry_msgs::msg::Pose> & _waypoints,
    const std::string & _link, double step, double jump_threshold = 0,
    bool execute = true) override;

  bool move_until_before_collide(
    const std::string & planning_group,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & link, double step_size, int max_attempts,
    char axis,
    bool execute = true) override;

  void attach_object_to_ee(
    const moveit_msgs::msg::CollisionObject & object,
    const std::string & ee_link);

  void detach_object_from_ee(
    const moveit_msgs::msg::CollisionObject & object,
    const std::string & ee_link);

  // TODO(Briancbn): Generic type function call
  void attach_object_to_ee(
    const shape_msgs::msg::SolidPrimitive & object,
    const std::string & ee_link) override {}

  // TODO(Briancbn): Generic type function call
  void detach_object_from_ee(
    const shape_msgs::msg::SolidPrimitive & object,
    const std::string & ee_link) override {}

protected:
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
  std::unordered_map<std::string, JmgContext> arms_;

  std::string robot_frame_;
};

}  // namespace grasp_execution

#endif  // GRASP_EXECUTION__MOVEIT_CPP_IF_HPP_