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

#include "grasp_execution/dynamic_safety/visualizer.hpp"

namespace grasp_execution
{

namespace dynamic_safety
{

Visualizer::Visualizer()
{
  // Dark Grey
  DARK_GREY.r = DARK_GREY.g = DARK_GREY.b = 0.628;
  DARK_GREY.a = 1;

  // Red
  RED.r = RED.a = 1;

  // Orange
  ORANGE.r = ORANGE.a = 1;
  ORANGE.g = 0.5;

  // Yellow
  YELLOW.r = YELLOW.g = YELLOW.a = 1;

  // Light Green
  LIGHT_GREEN.r = 0.5;
  LIGHT_GREEN.g = LIGHT_GREEN.a = 1;

  // Greem
  GREEN.g = GREEN.a = 1;
}


void Visualizer::configure(
  const robot_trajectory::RobotTrajectoryPtr & rt,
  const rclcpp::Node::SharedPtr & node,
  const Option & option,
  const SafetyZone::Option & zone_option)
{
  node_ = node;
  pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    option.topic, 2);
  rate_ = option.publish_frequency;
  step_ = option.step;
  update_trajectory(rt);
  safety_zone_.set(zone_option);
}

void Visualizer::update_trajectory(
  const robot_trajectory::RobotTrajectoryPtr & rt)
{
  int size = static_cast<int>(rt->getDuration() / step_);
  marker_msg_.reset(new visualization_msgs::msg::Marker());

  marker_msg_->header.frame_id = rt->getRobotModel()->getRootLinkName();
  marker_msg_->ns = "";
  marker_msg_->id = 0;
  marker_msg_->type = marker_msg_->SPHERE_LIST;
  marker_msg_->action = marker_msg_->ADD;
  marker_msg_->scale.x = 0.01;
  marker_msg_->scale.y = 0.01;
  marker_msg_->scale.z = 0.01;
  marker_msg_->lifetime = rclcpp::Duration(1e9);
  marker_msg_->points.resize(size);
  marker_msg_->colors.resize(size, DARK_GREY);

  for (int i = 0; i < size; i++) {
    moveit::core::RobotStatePtr rs =
      std::make_shared<moveit::core::RobotState>(rt->getRobotModel());
    rt->getStateAtDurationFromStart(step_ * i, rs);
    const auto & tf = rs->getGlobalLinkTransform(
      rt->getGroup()->getLinkModels().back());
    ASSERT_ISOMETRY(tf);  // unsanitized input, could contain a non-isometry
    marker_msg_->points[i].x = tf.translation().x();
    marker_msg_->points[i].y = tf.translation().y();
    marker_msg_->points[i].z = tf.translation().z();
  }
  current_time_point_ = 0;
  collision_time_point_ = -1;
}

void Visualizer::update(
  double current_time_point,
  double collision_time_point)
{
  current_time_point_ = current_time_point;
  collision_time_point_ = collision_time_point;
}

void Visualizer::start()
{
  timer_ = node_->create_wall_timer(
    std::chrono::nanoseconds(static_cast<int>(1e9 / rate_)),
    std::bind(&Visualizer::_timer_cb, this));
}

void Visualizer::reset()
{
  timer_.reset();
  pub_.reset();
}

void Visualizer::_timer_cb()
{
  for (size_t i = 0; i < marker_msg_->colors.size(); i++) {
    double time_point = step_ * i;
    if (time_point < current_time_point_ ||
      time_point >= current_time_point_ + safety_zone_.get_zone_limit(safety_zone_.REPLAN))
    {
      marker_msg_->colors[i] = DARK_GREY;
    } else {
      switch (safety_zone_.get_zone(time_point - current_time_point_)) {
        case SafetyZone::BLIND:
          marker_msg_->colors[i] = RED;
          break;
        case SafetyZone::EMERGENCY:
          marker_msg_->colors[i] = ORANGE;
          break;
        case SafetyZone::SLOWDOWN:
          marker_msg_->colors[i] = YELLOW;
          break;
        case SafetyZone::REPLAN:
          marker_msg_->colors[i] = LIGHT_GREEN;
          break;
        case SafetyZone::SAFE:
          marker_msg_->colors[i] = GREEN;
          break;
      }
    }
  }

  if (collision_time_point_ > 0) {
    int collision_idx = static_cast<int>(collision_time_point_ / step_);
    marker_msg_->colors[collision_idx] = RED;
  }
  marker_msg_->header.stamp = node_->now();

  // Check if timer_ has been reset before publishing the viz markers
  if (pub_) {
    pub_->publish(*marker_msg_);
  }
}

}  // namespace dynamic_safety

}  // namespace grasp_execution
