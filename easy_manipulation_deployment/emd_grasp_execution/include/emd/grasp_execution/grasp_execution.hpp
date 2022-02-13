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

#ifndef EMD__GRASP_EXECUTION__GRASP_EXECUTION_HPP_
#define EMD__GRASP_EXECUTION__GRASP_EXECUTION_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "emd/grasp_execution/context.hpp"
#include "emd/grasp_execution/exception.hpp"
#include "emd/grasp_execution/gripper/gripper_driver.hpp"
#include "emd/grasp_execution/core/scheduler.hpp"
#include "emd/grasp_execution/utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "pluginlib/class_loader.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace grasp_execution
{
/// Collision checker options.
struct GraspExecutionContext
{
  /// World frame of workcell
  std::string world_frame;

  /// Name of planning group
  std::string planning_group;

  /// Name of end effector link
  std::string ee_link;

  /// How far forward to plan for a trajectory during move unitl collide workflow
  double move_to_collide_step_size;

  /// TODO(Anyone): Better explanation of option
  float cartesian_step_size;

  /// TODO(Anyone): Better explanation of option
  float backtrack_steps;

  /// TODO(Anyone): Better explanation of option
  int hybrid_max_attempts;

  /// TODO(Anyone): Better explanation of option
  int non_deterministic_max_attempts;

  /// TODO(Anyone): Better explanation of option
  float clearance;
};

class GraspExecutionInterface
{
public:
  GraspExecutionInterface(
    const rclcpp::Node::SharedPtr & node,
    size_t planning_concurrency = 1,
    size_t execution_concurrency = 0
  );

  virtual ~GraspExecutionInterface();

  /// Initialize group.
  /**
   * Create an empty planning group.
   * \param[in] planning_group group name.
   * \return true if initialized succesfully, otherwise false.
   */
  virtual bool init(const std::string & planning_group)
  {
    workcell_context_->init_group(planning_group);
    return true;
  }

  /// Initialize context from yaml.
  /**
   * YAML configuration file format.
   * ```yaml
   * workcell:
   *   - group: manipulator1
   *     prefix: ur3e_
   *     executors:
   *       default:
   *         plugin: grasp_execution/DefaultExecutor
   *       <custom-executor>:
   *         plugin: <package-name>/<plugin-name>
   *         controller: <controller-name>
   *     end_effectors:
   *       - brand: robotiq_85_2f_gripper
   *         link: ur3e_gripper_link
   *         driver:
   *           plugin: <package-name>/<plugin-name>
   *           controller: <controller-name>
   *
   *   - group: manipulator2
   *     prefix: ur5_
   *     executors:
   *       default:
   *         plugin: grasp_execution/DefaultExecutor
   *       <custom-executor>:
   *         plugin: <package-name>/<plugin-name>
   *         controller: <controller-name>
   *     end_effectors:
   *       - brand: vg10_gripper
   *         link: ur5_gripper_link
   *         driver:
   *           plugin: <package-name>/<plugin-name>
   *           controller: <controller-name>
   * ```
   *
   * \param[in] path file path to the yaml configuration.
   * \return true if loaded successfully, otherwise false.
   */
  virtual bool init_from_yaml(const std::string & path)
  {
    try {
      workcell_context_->init_from_yaml(path);
      return true;
    } catch (const ContextLoadingException & ex) {
      RCLCPP_ERROR(rclcpp::get_logger("grasp_execution"), ex.what());
      return false;
    }
  }

  /// Load execution method for a group.
  /**
   * This method can be called multiple times to load multiple
   * execution method.
   * \param[in] group_name group name.
   * \param[in] execution_method execution method name.
   * \param[in] execution_plugin execution method plugin name.
   * \param[in] execution_controller execution method controller.
   * \return true if loaded succesfully, otherwise false
   */
  virtual bool load_execution_method(
    const std::string & group_name,
    const std::string & execution_method,
    const std::string & execution_plugin,
    const std::string & execution_controller = "")
  {
    workcell_context_->load_execution_method(
      group_name, execution_method, execution_plugin, execution_controller);
    return true;
  }

  /// Load end effector for a group.
  /**
   * This method can be called multiple times to load multiple
   * end effectors method.
   * \param[in] group_name group name.
   * \param[in] ee_brand end effector brand.
   * \param[in] ee_link end effector link for planning.
   * \param[in] ee_driver_plugin end effector driver plugin.
   * \param[in] ee_driver_controller end effector driver controller.
   * \return true if loaded succesfully, otherwise false
   */
  virtual bool load_ee(
    const std::string & group_name,
    const std::string & ee_name,
    const std::string & ee_brand,
    const std::string & ee_link,
    double ee_clearance,
    const std::string & ee_driver_plugin,
    const std::string & ee_driver_controller = "")
  {
    workcell_context_->load_ee(
      group_name, ee_name, ee_brand, ee_link, ee_clearance,
      ee_driver_plugin, ee_driver_controller);
    return true;
  }

  bool execute_plan(
    const std::string & action_description,
    const std::string & id,
    const std::string & planning_group);

  bool plan_and_execute_job(
    const GraspExecutionContext & option,
    const std::string action_description,
    const std::string target_id,
    const geometry_msgs::msg::PoseStamped & target_pose);

  bool plan_and_execute_collision_job(
    const GraspExecutionContext & option,
    const std::string action_description,
    const std::string target_id,
    char axis,
    const geometry_msgs::msg::PoseStamped & target_pose);

  virtual bool home(
    const std::string & planning_group) = 0;

  virtual void register_target_object(
    const shape_msgs::msg::SolidPrimitive & target_object_shape,
    const geometry_msgs::msg::PoseStamped & target_object_pose,
    const int & index,
    const std::string & task_id,
    const std::vector<std::string> & disabled_links = {}) = 0;

  const WorkcellContext & get_workcell_context() const
  {
    return *workcell_context_;
  }

  virtual geometry_msgs::msg::Pose get_object_pose(const std::string & object_id) const = 0;

  virtual geometry_msgs::msg::PoseStamped get_curr_pose(const std::string & link_name) const = 0;

  virtual bool move_to(
    const float & cartesian_step_size,
    const int & backtrack_steps,
    const int & hybrid_max_attempts,
    const int & non_deterministic_max_attempts,
    const std::string & planning_group,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & link,
    bool execute = true) = 0;

  virtual bool move_to(
    const GraspExecutionContext & option,
    const geometry_msgs::msg::PoseStamped & pose,
    bool execute = true) = 0;

  virtual bool move_to(
    const int & non_deterministic_max_attempts,
    const std::string & planning_group,
    const std::string & named_state,
    bool execute = true) = 0;

  virtual bool move_to(
    const std::string & planning_group,
    const sensor_msgs::msg::JointState & state,
    bool execute = true) = 0;

  virtual bool cartesian_to(
    const std::string & planning_group,
    const std::vector<geometry_msgs::msg::Pose> & _waypoints,
    const std::string & _link, double step, double jump_threshold = 0,
    bool execute = true) = 0;

  virtual bool move_until_before_collide(
    const std::string & planning_group,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & link, double step_size, int max_attempts,
    char axis,
    bool execute = true) = 0;

  virtual void attach_object_to_ee(
    const std::string & target_id,
    const std::string & ee_link) = 0;

  virtual void detach_object_from_ee(
    const std::string & target_id,
    const std::string & ee_link) = 0;

  virtual void remove_object(
    const std::string & target_id) = 0;

  virtual bool squash_and_execute(
    const std::string & group,
    const std::string & method = "default",
    const double velocity = 1.0) = 0;

  virtual void prompt_job_start(
    const std::string & job_id,
    const std::string & message,
    char separator = '>',
    std::ostream & out = std::cout);

  void prompt_job_start(
    const rclcpp::Logger & logger,
    const std::string & job_id,
    const std::string & message,
    char separator = '>')
  {
    std::ostringstream oss;
    oss << std::endl;
    prompt_job_start(job_id, message, separator, oss);
    RCLCPP_INFO(logger, oss.str().c_str());
  }

  virtual void prompt_job_end(
    bool end_result = true,
    char separator = '<',
    std::ostream & out = std::cout);

  void prompt_job_end(
    const rclcpp::Logger & logger,
    bool end_result = true,
    char separator = '<')
  {
    std::ostringstream oss;
    oss << std::endl;
    prompt_job_end(end_result, separator, oss);
    RCLCPP_INFO(logger, oss.str().c_str());
  }

protected:
  std::string gen_target_object_id(
    const shape_msgs::msg::SolidPrimitive & target_object_shape,
    std::string task_id,
    size_t index) const;

  std::string gen_target_mesh_id(
    const std::string & task_id,
    size_t index) const;

  void to_frame(
    const geometry_msgs::msg::PoseStamped & in_,
    geometry_msgs::msg::PoseStamped & out_,
    const std::string & _target_frame)
  {
    ::grasp_execution::to_frame(in_, out_, _target_frame, *tf_buffer_);
  }

  std::string robot_frame_;

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  core::Scheduler planning_scheduler;
  core::Scheduler execution_scheduler;

  std::shared_ptr<pluginlib::ClassLoader<
      grasp_execution::gripper::GripperDriver>> gripper_driver_loader_;

private:
  std::unique_ptr<WorkcellContext> workcell_context_;
  // rclcpp::Subscription<emd_msgs::msg::GraspTask>::SharedPtr grasp_task_sub_;
  // rclcpp::Service<emd_msgs::srv::GraspRequest>::SharedPtr grasp_req_service_;
};

}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__GRASP_EXECUTION_HPP_
