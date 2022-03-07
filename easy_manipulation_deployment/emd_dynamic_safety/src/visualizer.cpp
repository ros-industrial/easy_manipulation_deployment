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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "emd/dynamic_safety/visualizer.hpp"
#include "emd/interpolate.hpp"

#include "urdf/model.h"
#include "srdfdom/model.h"

namespace dynamic_safety
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dynamic_safety.visualizer");

Visualizer::Visualizer()
{
  // Dark Grey
  DARK_GREY.r = DARK_GREY.g = DARK_GREY.b = static_cast<float>(0.628);
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
  const rclcpp::Node::SharedPtr & node,
  const Option & option,
  const SafetyZone::Option & zone_option,
  const std::string & robot_urdf,
  const std::string & robot_srdf)
{
  urdf::ModelSharedPtr umodel = std::make_shared<urdf::Model>();
  srdf::ModelSharedPtr smodel = std::make_shared<srdf::Model>();
  if (umodel->initString(robot_urdf)) {
    if (!smodel->initString(*umodel, robot_srdf)) {
      RCLCPP_ERROR(LOGGER, "Unable to parse SRDF");
      // TODO(anyone): exception handling
    }
  } else {
    RCLCPP_ERROR(LOGGER, "Unable to parse URDF");
    // TODO(anyone): exception handling
  }
  // Construct planning scene
  scene_ = std::make_shared<planning_scene::PlanningScene>(umodel, smodel);
  node_ = node;
  start_ = false;
  pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    option.topic, 2);
  rate_ = option.publish_frequency;
  step_ = option.step;
  tcp_link_ = option.tcp_link;
  safety_zone_.set(zone_option);
  visualizer_callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions env_state_sub_option;
  env_state_sub_option.callback_group = visualizer_callback_group_;

  // Marker message update needs to be atomix
  marker_msg_ = std::make_shared<visualization_msgs::msg::Marker>();
}

void Visualizer::add_trajectory(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt)
{
  if (rt->points.empty()) {
    // TODO(anyone): exception handling
    return;
  }

  // Re-time trajectory
  double full_duration = rclcpp::Duration(rt->points.back().time_from_start).seconds();
  int state_size = static_cast<int>(full_duration / step_);

  marker_msg_->ns = "";
  marker_msg_->id = 0;
  marker_msg_->type = marker_msg_->SPHERE_LIST;
  marker_msg_->action = marker_msg_->ADD;
  marker_msg_->scale.x = 0.01;
  marker_msg_->scale.y = 0.01;
  marker_msg_->scale.z = 0.01;
  marker_msg_->lifetime = rclcpp::Duration::from_seconds(1);
  marker_msg_->points.clear();
  marker_msg_->colors.clear();
  auto & marker_points = marker_msg_->points;

  // record the starting offset
  double time_from_start = full_duration - static_cast<double>(state_size) * step_;

  size_t num_points = rt->points.size();
  size_t before, after;
  size_t i = 0;
  moveit::core::RobotStatePtr rs =
    std::make_shared<moveit::core::RobotState>(scene_->getRobotModel());
  marker_msg_->header.frame_id = rs->getRobotModel()->getRootLinkName();
  for (int idx = 0; idx <= state_size; idx++) {
    for (; i < rt->points.size(); i++) {
      if (rclcpp::Duration(rt->points[i].time_from_start).seconds() >= time_from_start) {
        break;
      }
    }
    before = std::max<size_t>((i == 0) ? 0 : (i - 1), 0);  // Avoid unsigned int 0 minus 1
    after = std::min<size_t>(i, num_points - 1);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
    emd::core::interpolate_between_points(
      rt->points[before].time_from_start, rt->points[before],
      rt->points[after].time_from_start, rt->points[after],
      point.time_from_start, point);
    for (size_t j = 0; j < rt->joint_names.size(); j++) {
      rs->setJointPositions(rt->joint_names[j], &point.positions[j]);
    }
    const auto & tf = rs->getGlobalLinkTransform(tcp_link_);
    // ASSERT_ISOMETRY(tf);  // unsanitized input, could contain a non-isometry
    geometry_msgs::msg::Point marker_point;
    marker_point.x = tf.translation().x();
    marker_point.y = tf.translation().y();
    marker_point.z = tf.translation().z();
    marker_points.push_back(std::move(marker_point));
    marker_msg_->colors.push_back(DARK_GREY);
    time_from_start += step_;
  }

  current_time_point_ = 0;
  collision_time_point_ = -1;

  // Better handling of this.
  timer_ = node_->create_wall_timer(
    std::chrono::nanoseconds(static_cast<int>(1e9 / rate_)),
    std::bind(&Visualizer::_timer_cb, this),
    visualizer_callback_group_);
}

void Visualizer::update(
  double current_time_point,
  double collision_time_point)
{
  current_time_point_ = current_time_point;
  collision_time_point_ = collision_time_point;
}

void Visualizer::update(
  const SafetyZone::Option & zone_option)
{
  safety_zone_.set(zone_option);
}

void Visualizer::start()
{
  start_ = true;
}

void Visualizer::stop()
{
  timer_.reset();
  start_ = false;
}

void Visualizer::reset()
{
  pub_.reset();
}

void Visualizer::_timer_cb()
{
  if (!start_) {
    return;
  }

  // Make the callback function atomic
  for (size_t i = 0; i < marker_msg_->colors.size(); i++) {
    double time_point = step_ * static_cast<int>(i);
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
    size_t collision_idx = static_cast<size_t>(collision_time_point_ / step_);
    marker_msg_->colors[collision_idx] = RED;
  }
  marker_msg_->header.stamp = node_->now();

  // Check if timer_ has been reset before publishing the viz markers
  if (pub_) {
    pub_->publish(*marker_msg_);
  }
}

}  // namespace dynamic_safety
