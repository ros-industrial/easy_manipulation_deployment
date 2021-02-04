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

#include "grasp_planning/msg/grasp_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace grasp_execution
{

class GraspExecutionInterface
{
public:
  GraspExecutionInterface(
    const rclcpp::Node::SharedPtr & node,
    const std::string & grasp_poses_topic = "grasp_poses",
    size_t planning_concurrency = 1,
    size_t execution_concurrency = 0
  )
  : node_(node),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
    tf_listener_(*tf_buffer_),
    planning_scheduler(planning_concurrency),
    execution_scheduler(execution_concurrency)
  {
    grasp_planning_sub_ = node_->create_subscription<grasp_planning::msg::GraspPose>(
      grasp_poses_topic, 10,
      [ = ](grasp_planning::msg::GraspPose::UniquePtr msg) {
        order_schedule(std::move(msg));
      }
    );
  }

  virtual ~GraspExecutionInterface() {}

  virtual void planning_workflow(
    const grasp_planning::msg::GraspPose::SharedPtr & msg) = 0;

  virtual void execution_workflow(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr & msg) = 0;

  virtual void order_schedule(
    const grasp_planning::msg::GraspPose::SharedPtr & msg) = 0;


  virtual geometry_msgs::msg::Pose get_object_pose(const std::string & object_id) const = 0;

  virtual geometry_msgs::msg::PoseStamped get_curr_pose(const std::string & link_name) const = 0;

  virtual bool move_to(
    const std::string & planning_group,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & link,
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

  // TODO(Briancbn): Generic type function call
  virtual void attach_object_to_ee(
    const shape_msgs::msg::SolidPrimitive & object,
    const std::string & ee_link) = 0;

  // TODO(Briancbn): Generic type function call
  virtual void detach_object_from_ee(
    const shape_msgs::msg::SolidPrimitive & object,
    const std::string & ee_link) = 0;

protected:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Scheduler planning_scheduler;
  Scheduler execution_scheduler;

private:
  rclcpp::Subscription<grasp_planning::msg::GraspPose>::SharedPtr grasp_planning_sub_;
};

}  // namespace grasp_execution

#endif  // GRASP_EXECUTION__GRASP_EXECUTION_HPP_
