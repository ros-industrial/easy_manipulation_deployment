// Copyright 2021 ROS Industrial Consortium Asia Pacific
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

#include <memory>
#include <string>

#include "emd/dynamic_safety/next_point_publisher.hpp"


namespace dynamic_safety
{

static const rclcpp::Logger & LOGGER = rclcpp::get_logger("dynamic_safety.next_point_publisher");

NextPointPublisher::NextPointPublisher()
{
}

NextPointPublisher::~NextPointPublisher()
{
}

void NextPointPublisher::configure(
  const robot_trajectory::RobotTrajectoryPtr & traj,
  const Option & option,
  const rclcpp::Node::SharedPtr & node,
  double rate)
{
  node_ = node;

  // Deep copy trajectory
  traj_ = std::make_shared<robot_trajectory::RobotTrajectory>(*traj, true);

  // Prepare command
  point_ = std::make_shared<moveit::core::RobotState>(traj_->getRobotModel());
  int dof = traj_->getGroup()->getVariableCount();
  command_out_.resize(dof);

  command_out_type_ = _from_string(option.command_out_type);
  command_out_topic_ = option.command_out_topic;

  publish_joint_position_ = option.publish_joint_position;
  publish_joint_velocity_ = option.publish_joint_velocity;
  publish_joint_effort_ = option.publish_joint_effort;

  // Set rate and deadline
  period_ = 1.0 / rate;

  command_out_array_.data.resize(dof);
  command_out_traj_.header.frame_id = point_->getRobotModel()->getRootLinkName();
  command_out_traj_.header.stamp = rclcpp::Time(0);
  command_out_traj_.joint_names = traj_->getGroup()->getVariableNames();
  command_out_traj_.points.resize(1);
  command_out_traj_.points.front().time_from_start = rclcpp::Duration::from_seconds(period_);
  command_out_traj_.points.front().positions.resize(dof);
  command_out_traj_.points.front().velocities.resize(dof);
  command_out_traj_.points.front().effort.resize(dof);

  status_ = IDLE;
}

void NextPointPublisher::start(double scale)
{
  auto qos = rclcpp::QoS(2);  // .deadline(
  //   rclcpp::Duration::from_seconds(period_));

  rclcpp::PublisherOptions command_out_option;
  // command_out_option.event_callbacks.deadline_callback =
  //   std::bind(&NextPointPublisher::_deadline_cb, this, std::placeholders::_1);

  switch (command_out_type_) {
    case Command::ARRAY:
      command_out_array_pub_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        command_out_topic_,
        qos,
        command_out_option);
      break;
    case Command::TRAJECTORY:
      command_out_traj_pub_ =
        node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        command_out_topic_,
        qos,
        command_out_option);
      break;
  }

  start_time_ = clock::now();
  scale_time_ = start_time_;
  time_point_ = 0;
  scale_ = scale;
  remaining_steps_to_scale_ = 0;
  scale_step_ = 0;
  status_ = RUNNING;
}

void NextPointPublisher::scale(double scale, double time_to_scale)
{
  if (std::abs(scale - get_scale()) > 1e-3) {
    double original_scale = get_scale();
    remaining_steps_to_scale_ = static_cast<int>(time_to_scale / period_);
    if (remaining_steps_to_scale_ == 0) {
      this->_scale_impl(scale);
    }
    scale_step_ = (scale_ - scale) / static_cast<double>(remaining_steps_to_scale_);
    RCLCPP_WARN(
      LOGGER, "Current scale: %f, New scale: %f, remaining steps: %d, scale steps: %f",
      original_scale, scale, remaining_steps_to_scale_, scale_step_);
  }
}

void NextPointPublisher::stop()
{
  time_point_ = current_point();
  traj_->getStateAtDurationFromStart(time_point_, point_);

  _send_command();
}

void NextPointPublisher::halt()
{
  stop();
  status_ = FAILED;
}

void NextPointPublisher::reset()
{
  status_ = IDLE;
  command_out_array_pub_.reset();
}

void NextPointPublisher::update_traj(const robot_trajectory::RobotTrajectoryPtr & traj)
{
  // Deep copy trajectory
  traj_ = std::make_shared<robot_trajectory::RobotTrajectory>(*traj, true);
  start_time_ = clock::now();
  scale_time_ = start_time_;
  scale_ = 1;
  time_point_ = 0;

  remaining_steps_to_scale_ = 0;
  scale_step_ = 0;
  RCLCPP_INFO(LOGGER, "Receive new trajectory");
}

void NextPointPublisher::run_once()
{
  _next_point();
  _send_command();
}

const double & NextPointPublisher::current_point()
{
  time_point_ = static_cast<double>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      (clock::now() - scale_time_) * scale_ + (scale_time_ - start_time_)
    ).count()
    ) / 1e9;
  return time_point_;
}

void NextPointPublisher::_next_point()
{
  if (remaining_steps_to_scale_ > 0) {
    _scale_impl(scale_ - scale_step_);
    remaining_steps_to_scale_--;
  }
  time_point_ = current_point();
  // RCLCPP_INFO(LOGGER, "time point: %f", time_point_);
  if (time_point_ < traj_->getDuration()) {
    traj_->getStateAtDurationFromStart(time_point_ + period_ * scale_, point_);
  } else {
    status_ = SUCCEEDED;
    // TODO(Briancbn): validate ending condition;
  }
}

void NextPointPublisher::_send_command()
{
  const auto & joints = traj_->getGroup()->getVariableNames();
  for (size_t i = 0; i < command_out_.size(); i++) {
    command_out_[i] = *point_->getJointPositions(joints[i]);
  }
  switch (command_out_type_) {
    case Command::ARRAY:
      command_out_array_.data = command_out_;
      command_out_array_pub_->publish(command_out_array_);
      break;
    case Command::TRAJECTORY:
      command_out_traj_.points.front().positions = command_out_;
      command_out_traj_pub_->publish(command_out_traj_);
      break;
  }
}

void NextPointPublisher::_scale_impl(double scale)
{
  start_time_ += std::chrono::duration_cast<std::chrono::nanoseconds>(
    (clock::now() - scale_time_) * (1.0 - scale_));
  scale_ = scale;
  scale_time_ = clock::now();
}

NextPointPublisher::Command NextPointPublisher::_from_string(
  const std::string & command_type)
{
  if (command_type == "trajectory_msgs/JointTrajectory") {
    return Command::TRAJECTORY;
  } else {  // if (command_type == "std_msgs/Float64MultiArray"){
    return Command::ARRAY;
  }
}

void NextPointPublisher::_deadline_cb(rclcpp::QOSDeadlineOfferedInfo &)
{
  RCLCPP_WARN(node_->get_logger(), "Missed a deadline!!!!");
}

}  // namespace dynamic_safety
