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

#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "grasp_execution/grasp_execution.hpp"

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"

#include "emd_msgs/msg/grasp_task.hpp"

#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"

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
  std::deque<robot_trajectory::RobotTrajectoryPtr> traj;
};

class MoveitCppGraspExecution : public GraspExecutionInterface
{
public:
  explicit MoveitCppGraspExecution(
    const rclcpp::Node::SharedPtr & node,
    const std::string & grasp_poses_topic = "grasp_request",
    size_t planning_concurrency = 1,
    size_t execution_concurrency = 0);

  ~MoveitCppGraspExecution();

  bool init(const std::string & planning_group, const std::string & _ee_link = "");

  void order_schedule(
    const emd_msgs::msg::GraspTask::SharedPtr & msg) override {}

  void register_target_objects(
    const emd_msgs::msg::GraspTask::SharedPtr & msg) override;

  geometry_msgs::msg::Pose get_object_pose(const std::string & object_id) const override;

  geometry_msgs::msg::PoseStamped get_curr_pose(const std::string & link_name) const override;

  moveit::core::RobotStatePtr get_curr_state() const;

  bool move_to(
    const std::string & planning_group,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & link,
    bool execute = true) override;

  bool move_to(
    const std::string & planning_group,
    const moveit::core::RobotState & state,
    bool execute = true);

  bool home(
    const std::string & planning_group) override {return true;}

  // Skipped
  // TODO(Briancbn): Generic type function call
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

  void attach_object_to_ee(
    const std::string & target_id,
    const std::string & ee_link) override;

  void detach_object_from_ee(
    const std::string & target_id,
    const std::string & ee_link) override;

  void remove_object(
    const std::string & target_id) override;

protected:
  void squash_trajectories(
    const std::string & planning_group,
    int start_idx = 0, int end_idx = -1,
    bool time_parameterization = false);

  void print_trajectory_ros(
    const rclcpp::Logger & logger,
    const robot_trajectory::RobotTrajectoryPtr & traj)
  {
    std::ostringstream oss;
    print_trajectory(traj, oss);
    RCLCPP_INFO(logger, oss.str());
  }

  void print_trajectory(
    const robot_trajectory::RobotTrajectoryPtr & traj,
    std::ostream & _out = std::cout);

  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
  std::unordered_map<std::string, JmgContext> arms_;
};

}  // namespace grasp_execution

#endif  // GRASP_EXECUTION__MOVEIT_CPP_IF_HPP_
