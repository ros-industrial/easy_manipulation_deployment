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
#include <utility>

#include "emd/dynamic_safety/dynamic_safety.hpp"


namespace dynamic_safety
{

static const rclcpp::Logger & LOGGER = rclcpp::get_logger("dynamic_safety");

const Option & Option::load(const rclcpp::Node::SharedPtr & node)
{
  // Load dyanmic safety parameters
  emd::declare_or_get_param<double>(
    rate,
    "rate",
    node, LOGGER);

  // Load dyanmic safety parameters
  emd::declare_or_get_param<bool>(
    allow_replan,
    "allow_replan",
    node, LOGGER, false);

  // Load dyanmic safety parameters
  emd::declare_or_get_param<bool>(
    visualize,
    "visualize",
    node, LOGGER, false);

  // Load safety zone paramters
  emd::declare_or_get_param<bool>(
    safety_zone_options.manual,
    "safety_zone.manual",
    node, LOGGER);
  emd::declare_or_get_param<std::string>(
    safety_zone_options.unit_type,
    "safety_zone.unit_type",
    node, LOGGER, "second");  // default: "second"

  // Only second is enabled
  if (safety_zone_options.unit_type != "second") {
    RCLCPP_WARN(
      LOGGER, "Wrong safety zone unit type: [%s], default to [second]",
      safety_zone_options.unit_type.c_str());
    safety_zone_options.unit_type = "second";
  }
  if (safety_zone_options.manual) {
    emd::declare_or_get_param<double>(
      safety_zone_options.collision_checking_deadline,
      "safety_zone.collision_checking_deadline",
      node, LOGGER);

    emd::declare_or_get_param<double>(
      safety_zone_options.slow_down_time,
      "safety_zone.slow_down_time",
      node, LOGGER);

    emd::declare_or_get_param<double>(
      safety_zone_options.replan_deadline,
      "safety_zone.replan_deadline",
      node, LOGGER);

    emd::declare_or_get_param<double>(
      safety_zone_options.look_ahead_time,
      "safety_zone.look_ahead_time",
      node, LOGGER);
  }

  // Load collision checker parameters
  emd::declare_or_get_param<bool>(
    collision_checker_options.distance,
    "collision_checker.distance",
    node, LOGGER, false);  // default: false

  emd::declare_or_get_param<bool>(
    collision_checker_options.continuous,
    "collision_checker.continuous",
    node, LOGGER, false);  // default: false

  emd::declare_or_get_param<bool>(
    collision_checker_options.realtime,
    "collision_checker.realtime",
    node, LOGGER, false);  // default: false

  if (collision_checker_options.continuous) {
    // TODO(anyone):
  } else {
    emd::declare_or_get_param<double>(
      collision_checker_options.step,
      "collision_checker.step",
      node, LOGGER);

    emd::declare_or_get_param<int>(
      collision_checker_options.thread_count,
      "collision_checker.thread_count",
      node, LOGGER, 1);  // default: 1
  }

  // Next point planner parameters
  emd::declare_or_get_param<std::string>(
    next_point_publisher_options.command_out_type,
    "next_point_publisher.command_out_type",
    node, LOGGER);

  emd::declare_or_get_param<std::string>(
    next_point_publisher_options.command_out_topic,
    "next_point_publisher.command_out_topic",
    node, LOGGER);

  emd::declare_or_get_param<bool>(
    next_point_publisher_options.publish_joint_position,
    "next_point_publisher.publish_joint_position",
    node, LOGGER, true);

  emd::declare_or_get_param<bool>(
    next_point_publisher_options.publish_joint_velocity,
    "next_point_publisher.publish_joint_velocity",
    node, LOGGER, false);

  emd::declare_or_get_param<bool>(
    next_point_publisher_options.publish_joint_effort,
    "next_point_publisher.publish_joint_effort",
    node, LOGGER, false);

  // Replanner parameters
  if (allow_replan) {
    emd::declare_or_get_param<std::string>(
      replanner_options.planner_name,
      "replanner.planner_name",
      node, LOGGER);
  }

  if (visualize) {
    emd::declare_or_get_param<double>(
      visualizer_options.publish_frequency,
      "visualizer.publish_frequency",
      node, LOGGER, 10);

    emd::declare_or_get_param<double>(
      visualizer_options.step,
      "visualizer.step",
      node, LOGGER, 0.1);

    emd::declare_or_get_param<std::string>(
      visualizer_options.topic,
      "visualizer.topic",
      node, LOGGER);
  }

  // Return idiom
  return *this;
}

void DynamicSafety::configure(
  const planning_scene::PlanningScenePtr & scene,
  const robot_trajectory::RobotTrajectoryPtr & rt,
  const rclcpp::Node::SharedPtr & node)
{
  node_ = node;

  collision_checker_.configure(
    option_.collision_checker_options, scene, rt);

  if (!safety_zone_.set(option_.safety_zone_options)) {
    throw std::runtime_error("Wrong safety zone parameters");
  }

  RCLCPP_INFO(LOGGER, "Configuring next point publisher");
  next_point_publisher_.configure(
    rt, option_.next_point_publisher_options, node, option_.rate);

  current_state_ = std::make_shared<moveit::core::RobotState>(rt->getRobotModel());

  if (option_.allow_replan) {
    replanner_.configure(scene, rt, node, option_.replanner_options);
  }

  if (option_.visualize) {
    visualizer_.configure(
      rt, node, option_.visualizer_options, option_.safety_zone_options);
  }

  benchmark_stats.clear();

  pf_ = new emd::TimeProfiler<>(5000);

  // double period = 1 / option_.rate;

  // Use mutually exclusive callback group
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto qos = rclcpp::QoS(2);  // .deadline(rclcpp::Duration::from_seconds(period));
  rclcpp::SubscriptionOptions state_sub_option;
  state_sub_option.callback_group = callback_group_;
  // state_sub_option.event_callbacks.deadline_callback =
  //   std::bind(&DynamicSafety::_deadline_cb, this, std::placeholders::_1);
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    qos,
    [ = ](sensor_msgs::msg::JointState::UniquePtr joint_state_msg) -> void
    {
      if (started) {
        _main_loop(std::move(joint_state_msg));
      }
    },
    state_sub_option);
}

void DynamicSafety::start()
{
  next_point_publisher_.start();
  RCLCPP_INFO(LOGGER, "Next point publisher started!");
  collision_time_point_ = -1;

  visualizer_.start();

  started = true;

  sig_ = std::promise<void>();
  future_ = sig_.get_future();
  RCLCPP_INFO(LOGGER, "All started");
}

void DynamicSafety::wait()
{
  RCLCPP_INFO(LOGGER, "Waiting...");
  future_.wait();
  RCLCPP_INFO(LOGGER, "Successfully exit.");
}

void DynamicSafety::stop()
{
  collision_checker_.reset();
  visualizer_.reset();
  RCLCPP_INFO(
    LOGGER, "Next Point Publisher ended with %s status",
    (next_point_publisher_.get_status() == NextPointPublisher::SUCCEEDED) ?
    "SUCCEEDED" : "FAILED");
  next_point_publisher_.reset();
  started = false;
  state_sub_.reset();
  callback_group_.reset();
  node_.reset();

  // Print out result
  std::ostringstream oss;
  pf_->print(oss);
  RCLCPP_INFO_STREAM(
    LOGGER,
    "Time stats:\n" << oss.str());
  delete pf_;
  sig_.set_value();
}

void DynamicSafety::_deadline_cb(rclcpp::QOSDeadlineRequestedInfo &)
{
}

void DynamicSafety::_main_loop(const sensor_msgs::msg::JointState::SharedPtr & joint_state_msg)
{
  pf_->reset();
  // Update everything
  for (size_t i = 0; i < joint_state_msg->name.size(); i++) {
    // TODO(anyone): multi-axis joint
    current_state_->setJointPositions(joint_state_msg->name[i], {joint_state_msg->position[i]});

    const auto & jm = current_state_->getJointModel(joint_state_msg->name[i]);
    if (!joint_state_msg->velocity.empty()) {
      current_state_->setJointVelocities(jm, &(joint_state_msg->velocity[i]));
    }
    if (!joint_state_msg->effort.empty()) {
      current_state_->setJointEfforts(jm, &(joint_state_msg->effort[i]));
    }
  }
  collision_checker_.update(*current_state_);

  // get scaled time point
  const auto & current_time_point = next_point_publisher_.current_point();
  // RCLCPP_INFO(node_->get_logger(), "Current time: %f", current_time_point);
  collision_time_point_ = -1;

  // Check collision once
  collision_checker_.run_once(
    current_time_point,
    option_.safety_zone_options.look_ahead_time,
    collision_time_point_
  );

  if (collision_time_point_ > 0) {
    RCLCPP_WARN(
      LOGGER, "collision_time: %f, current_time_point: %f",
      collision_time_point_, current_time_point);
    zone = safety_zone_.get_zone(collision_time_point_ - current_time_point);
  } else {
    zone = SafetyZone::SAFE;
  }

  visualizer_.update(current_time_point, collision_time_point_);

  switch (zone) {
    case SafetyZone::BLIND:
      next_point_publisher_.halt();
      RCLCPP_ERROR(LOGGER, "Obstacle too close, halt and abort.");
      return;
    case SafetyZone::EMERGENCY:
      next_point_publisher_.scale(1e-3, 0);
      RCLCPP_ERROR(LOGGER, "Emergency stop.");
      break;
    case SafetyZone::SLOWDOWN:
      if (!option_.allow_replan) {
        // Simply slow down to wait for restart
        next_point_publisher_.scale(1e-3, option_.safety_zone_options.slow_down_time);
        RCLCPP_WARN(LOGGER, "Slowing down.");
      } else {
        // Check if replan successfully
        if (replanner_.started()) {
          if (replanner_.get_result()) {
            if (current_time_point >= replan_time_point_) {
              const auto & new_traj = replanner_.get_trajectory();
              next_point_publisher_.update_traj(new_traj);
              collision_checker_.update_traj(new_traj, option_.collision_checker_options);
              visualizer_.update_trajectory(new_traj);
              replanner_.reset();
            } else {
              RCLCPP_WARN(LOGGER, "Haven't reach replan time point: %f", replan_time_point_);
            }
          } else {
            RCLCPP_WARN(LOGGER, "Why?");
          }
        } else {
          double replan_buffer = collision_time_point_ - current_time_point -
            option_.safety_zone_options.collision_checking_deadline -
            option_.safety_zone_options.slow_down_time -
            1.0 / option_.rate;
          if (replan_buffer > 0) {
            double scale =
              replan_buffer / option_.safety_zone_options.replan_deadline;

            next_point_publisher_.scale(
              scale, option_.safety_zone_options.slow_down_time);
            replan_time_point_ = current_time_point +
              scale * option_.safety_zone_options.replan_deadline;
          } else {
            next_point_publisher_.scale(
              1e-3, replan_buffer / 2);
            replan_time_point_ = current_time_point;
          }

          // Start replan
          replanner_.update(joint_state_msg);
          replanner_.start(
            replan_time_point_,
            option_.safety_zone_options.replan_deadline);
          RCLCPP_WARN(LOGGER, "Start slow down and replannig.");
        }
      }
      break;
    case SafetyZone::REPLAN:
      if (option_.allow_replan) {
        // Check if replan successfully
        if (replanner_.started()) {
          if (replanner_.get_result()) {
            if (current_time_point >= replan_time_point_) {
              const auto & new_traj = replanner_.get_trajectory();
              RCLCPP_INFO(LOGGER, "Update next point");
              next_point_publisher_.update_traj(new_traj);
              RCLCPP_INFO(LOGGER, "Update collision checker");
              collision_checker_.update_traj(new_traj, option_.collision_checker_options);
              visualizer_.update_trajectory(new_traj);
              replanner_.reset();
            } else {
              RCLCPP_WARN(LOGGER, "Haven't reach replan time point: %f", replan_time_point_);
            }
          }
        } else {
          // Start replan
          replanner_.update(joint_state_msg);
          replan_time_point_ =
            current_time_point + option_.safety_zone_options.replan_deadline - 1.0 / option_.rate;
          replanner_.start(
            replan_time_point_,
            option_.safety_zone_options.replan_deadline);
          RCLCPP_WARN(LOGGER, "Start replanning.");
        }
      } else {
        // Simply slow down to wait for restart
        next_point_publisher_.scale(1e-3, option_.safety_zone_options.slow_down_time);
        RCLCPP_WARN(LOGGER, "Replan Slowing down.");
      }
      break;
    case SafetyZone::SAFE:
      // back to original speed if slowed
      RCLCPP_WARN_ONCE(LOGGER, "SAFE");
      if (next_point_publisher_.get_scale() < 1.0) {
        RCLCPP_WARN(LOGGER, "Back to original speed");
        next_point_publisher_.scale(1.0, option_.safety_zone_options.slow_down_time);
      }
      break;
  }

  double lapse_time = pf_->lapse_and_record();
  if (lapse_time > 1.0 / option_.rate) {
    RCLCPP_ERROR(
      LOGGER, "Lapse time %fs exceeds maximum rate %fHz",
      lapse_time, option_.rate);
    // Simply slow down to wait for restart
    next_point_publisher_.scale(1e-3, option_.safety_zone_options.slow_down_time);
    RCLCPP_WARN_ONCE(LOGGER, "Slowing down.");

    // Simply slow down to wait for restart
    next_point_publisher_.scale(1e-3, option_.safety_zone_options.slow_down_time);
    RCLCPP_WARN_ONCE(LOGGER, "Slowing down.");
  } else {
    rclcpp::sleep_for(
      rclcpp::Duration::from_seconds(
        1.0 / option_.rate - lapse_time).to_chrono<std::chrono::nanoseconds>());
  }

  next_point_publisher_.run_once();

  auto status = next_point_publisher_.get_status();

  if (status != NextPointPublisher::RUNNING) {
    stop();
  }
}


}  // namespace dynamic_safety
