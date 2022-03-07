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

#ifndef EMD__GRASP_EXECUTION__MOVEIT2__MOVEIT_CPP_IF_HPP_
#define EMD__GRASP_EXECUTION__MOVEIT2__MOVEIT_CPP_IF_HPP_

#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "emd/grasp_execution/grasp_execution.hpp"
#include "emd/grasp_execution/moveit2/executor.hpp"

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"

#include "pluginlib/class_loader.hpp"

namespace grasp_execution
{

namespace moveit2
{

struct JmgContext
{
  moveit_cpp::PlanningComponentPtr planner;
  std::deque<robot_trajectory::RobotTrajectoryPtr> traj;
  std::unordered_map<std::string, Executor::UniquePtr> executors;
  std::unordered_map<std::string, gripper::GripperDriver::UniquePtr> grippers;
  WorkcellContext::Gripper default_ee;
};

class MoveitCppGraspExecution : public GraspExecutionInterface
{
public:
  explicit MoveitCppGraspExecution(
    const rclcpp::Node::SharedPtr & node,
    size_t planning_concurrency = 1,
    size_t execution_concurrency = 0);

  ~MoveitCppGraspExecution();

  /// Documentation inherited
  bool init(const std::string & planning_group) override;

  /// Documentation inherited
  bool init_from_yaml(const std::string & path) override;

  /// Documentation inherited
  bool load_execution_method(
    const std::string & group_name,
    const std::string & execution_method,
    const std::string & execution_plugin,
    const std::string & execution_controller = "") override;

  /// Documentation inherited
  bool load_ee(
    const std::string & group_name,
    const std::string & ee_name,
    const std::string & ee_brand,
    const std::string & ee_link,
    double ee_clearance,
    const std::string & ee_driver_plugin,
    const std::string & ee_driver_controller = "") override;

  [[deprecated("Replaced by init(group_name) and load_ee, "
      "which has an improved interface")]]
  bool init(
    const std::string & planning_group,
    const std::string & _ee_link,
    const std::string & execution_method = "default",
    const std::string & execution_type = "",
    const std::string & controller_name = "");

  std::string register_target_object_mesh(
    const std::string & mesh_filepath,
    const geometry_msgs::msg::PoseStamped & target_object_pose,
    int index,
    const std::string & task_id,
    const std::vector<std::string> & disabled_links = {});

  void register_target_object(
    const shape_msgs::msg::SolidPrimitive & target_object_shape,
    const geometry_msgs::msg::PoseStamped & target_object_pose,
    const int & index,
    const std::string & task_id,
    const std::vector<std::string> & disabled_links = {}) override;

  geometry_msgs::msg::Pose get_object_pose(const std::string & object_id) const override;

  geometry_msgs::msg::PoseStamped get_curr_pose(const std::string & link_name) const override;

  moveit::core::RobotStatePtr get_curr_state() const;

  [[deprecated("Use the full configuration function move_to")]]
  bool move_to(
    const std::string & planning_group,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & link,
    bool execute = true)
  {
    return move_to(0.01, 20, 5, 5, planning_group, pose, link, execute);
  }

  bool move_to(
    const float & cartesian_step_size,
    const int & backtrack_steps,
    const int & hybrid_max_attempts,
    const int & non_deterministic_max_attempts,
    const std::string & planning_group,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & link,
    bool execute = true) override;


  bool move_to(
    const GraspExecutionContext & option,
    const geometry_msgs::msg::PoseStamped & pose,
    bool execute = true) override;

  [[deprecated("Use the full configuration function move_to")]]
  bool move_to(
    const std::string & planning_group,
    const moveit::core::RobotState & state,
    bool execute = true)
  {
    return move_to(5, planning_group, state, execute);
  }

  bool move_to(
    const int & non_deterministic_max_attempts,
    const std::string & planning_group,
    const moveit::core::RobotState & state,
    bool execute = true);

  [[deprecated("Use the full configuration function move_to")]]
  bool move_to(
    const std::string & planning_group,
    const std::string & named_state,
    bool execute = true)
  {
    return move_to(5, planning_group, named_state, execute);
  }
  bool move_to(
    const int & non_deterministic_max_attempts,
    const std::string & planning_group,
    const std::string & named_state,
    bool execute = true) override;

  bool home(
    const std::string &) override {return true;}

  bool move_to(
    const std::string &,
    const sensor_msgs::msg::JointState &,
    bool) override;

  double cartesian_to(
    const std::string & planning_group,
    moveit::core::RobotState & start_state,
    const std::vector<geometry_msgs::msg::Pose> & _waypoints,
    robot_trajectory::RobotTrajectoryPtr & traj,
    const std::string & _link, double step, double jump_threshold = 0);

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

  bool execute(
    const robot_trajectory::RobotTrajectoryPtr & traj,
    const std::string & method = "default");

  bool squash_and_execute(
    const std::string & group,
    const std::string & method = "default",
    const double velocity = 1.0) override;

protected:
  void squash_trajectories(
    const std::string & planning_group,
    const double velocity,
    int start_idx = 0, int end_idx = -1,
    bool time_parameterization = true);

  void print_trajectory_ros(
    const rclcpp::Logger & logger,
    const robot_trajectory::RobotTrajectoryPtr & traj)
  {
    std::ostringstream oss;
    oss << std::endl;
    print_trajectory(traj, oss);
    RCLCPP_INFO(logger, oss.str().c_str());
  }

  void print_trajectory(
    const robot_trajectory::RobotTrajectoryPtr & traj,
    std::ostream & _out = std::cout);

  moveit_cpp::MoveItCppPtr moveit_cpp_;
  std::unordered_map<std::string, moveit2::JmgContext> arms_;
  moveit2::Executor::UniquePtr default_executor_;

  std::shared_ptr<pluginlib::ClassLoader<
      grasp_execution::moveit2::Executor>> executor_loader_;
};
}  // namespace moveit2

}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__MOVEIT2__MOVEIT_CPP_IF_HPP_
