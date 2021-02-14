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

#include "grasp_execution/scheduler.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "grasp_execution/utils.hpp"
#include "emd_msgs/msg/grasp_task.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace grasp_execution
{

class GraspExecutionInterface
{
public:
  GraspExecutionInterface(
    const rclcpp::Node::SharedPtr & node,
    const std::string & grasp_task_topic = "grasp_request",
    size_t planning_concurrency = 1,
    size_t execution_concurrency = 0
  );

  virtual ~GraspExecutionInterface();

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
    const emd_msgs::msg::GraspTask::SharedPtr & msg) = 0;

  virtual void order_schedule(
    const emd_msgs::msg::GraspTask::SharedPtr & msg) = 0;

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
    const std::string & method = "default") = 0;

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

  Scheduler planning_scheduler;
  Scheduler execution_scheduler;

private:
  rclcpp::Subscription<emd_msgs::msg::GraspTask>::SharedPtr grasp_task_sub_;
};

}  // namespace grasp_execution

#endif  // GRASP_EXECUTION__GRASP_EXECUTION_HPP_
