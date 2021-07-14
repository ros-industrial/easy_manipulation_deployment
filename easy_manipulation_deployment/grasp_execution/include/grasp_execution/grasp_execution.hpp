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

#include "grasp_execution/context.hpp"
#include "grasp_execution/exception.hpp"
#include "grasp_execution/gripper/gripper_driver.hpp"
#include "grasp_execution/core/scheduler.hpp"
#include "grasp_execution/utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "emd_msgs/msg/grasp_task.hpp"
#include "emd_msgs/srv/grasp_request.hpp"
#include "pluginlib/class_loader.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace grasp_execution
{

class GraspExecutionInterface
{
public:
  GraspExecutionInterface(
    const rclcpp::Node::SharedPtr & node,
    const std::string & grasp_task_topic = "grasp_tasks",
    const std::string & grasp_req_topic = "grasp_requests",
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
    const std::string & ee_brand,
    const std::string & ee_name,
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

  bool default_plan_pre_grasp(
    const std::string & planning_group,
    const std::string & ee_link,
    const geometry_msgs::msg::PoseStamped & grasp_pose,
    double clearance);

  bool default_plan_transport(
    const std::string & planning_group,
    const std::string & ee_link,
    const geometry_msgs::msg::PoseStamped & release_pose,
    double clearance);

  bool default_plan_post_release(
    const std::string & planning_group,
    const std::string & ee_link,
    bool home,
    double clearance);

  virtual bool home(
    const std::string & planning_group) = 0;

  virtual void register_target_objects(
    const emd_msgs::msg::GraspTask::SharedPtr & msg,
    const std::vector<std::string> & disabled_links = {}) = 0;

  virtual void order_schedule(
    const emd_msgs::msg::GraspTask::SharedPtr & msg,
    bool blocking = false) = 0;

  const WorkcellContext & get_workcell_context() const
  {
    return *workcell_context_;
  }

  virtual geometry_msgs::msg::Pose get_object_pose(const std::string & object_id) const = 0;

  virtual geometry_msgs::msg::PoseStamped get_curr_pose(const std::string & link_name) const = 0;

  virtual bool move_to(
    const std::string & planning_group,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & link,
    bool execute = true) = 0;

  virtual bool move_to(
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
    RCLCPP_INFO(logger, oss.str());
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
    RCLCPP_INFO(logger, oss.str());
  }

protected:
  std::string gen_target_object_id(
    const emd_msgs::msg::GraspTask::SharedPtr & msg,
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
  rclcpp::Subscription<emd_msgs::msg::GraspTask>::SharedPtr grasp_task_sub_;
  rclcpp::Service<emd_msgs::srv::GraspRequest>::SharedPtr grasp_req_service_;

  std::unique_ptr<WorkcellContext> workcell_context_;
};

}  // namespace grasp_execution

#endif  // GRASP_EXECUTION__GRASP_EXECUTION_HPP_
