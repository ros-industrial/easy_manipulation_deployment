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

#include <unistd.h>
#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "emd/dynamic_safety/dynamic_safety.hpp"

namespace dynamic_safety
{

static const rclcpp::Logger & LOGGER = rclcpp::get_logger("dynamic_safety");

const Option & Option::load(const rclcpp::Node::SharedPtr & node)
{
  // Load dyanmic safety parameters
  emd::declare_or_get_param<bool>(
    dynamic_parameterization,
    "dynamic_safety.dynamic_parameterization",
    node, LOGGER, true);

  emd::declare_or_get_param<bool>(
    use_description_server,
    "dynamic_safety.use_description_server",
    node, LOGGER, true);

  if (use_description_server) {
    emd::declare_or_get_param<std::string>(
      description_server,
      "dynamic_safety.description_server",
      node, LOGGER);

    using namespace std::chrono_literals;
    auto description_loader_node = std::make_shared<rclcpp::Node>(
      std::string(
        node->get_name()) + "_description_loader");
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      description_loader_node, description_server);
    while (!parameters_client->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          LOGGER, "Interrupted while waiting for %s service. Exiting.",
          description_server.c_str());
        // TODO(anyone): exception handling.
        break;
      }
      RCLCPP_ERROR(
        LOGGER, "%s service not available, waiting again...",
        description_server.c_str());
    }
    RCLCPP_INFO(LOGGER, "Connected to description server %s!!", description_server.c_str());
    // search and wait for robot_description on param server
    while (robot_description.empty()) {
      try {
        RCLCPP_INFO(LOGGER, "Get parameters");
        auto f = parameters_client->get_parameters(
          {"robot_description",
            "robot_description_semantic"});
        RCLCPP_INFO(LOGGER, "Wait for parameters");
        rclcpp::spin_until_future_complete(description_loader_node, f);
        RCLCPP_INFO(LOGGER, "parameter received");
        std::vector<rclcpp::Parameter> values = f.get();
        RCLCPP_INFO(LOGGER, "parameter gotten");
        robot_description = values[0].as_string();
        robot_description_semantic = values[1].as_string();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(LOGGER, "%s", e.what());
      }

      if (!robot_description.empty()) {
        break;
      } else {
        RCLCPP_ERROR(
          LOGGER, "dynamic_safety is waiting for model"
          " URDF in parameter [robot_description] on the ROS param server.");
      }
      usleep(100000);
    }
    RCLCPP_INFO(
      LOGGER, "Recieved urdf & srdf from param server, parsing...");
  } else {
    emd::declare_or_get_param<std::string>(
      robot_description,
      "dynamic_safety.robot_description",
      node, LOGGER);  // default: "second"
    emd::declare_or_get_param<std::string>(
      robot_description_semantic,
      "dynamic_safety.robot_description_semantic",
      node, LOGGER);  // default: "second"
  }

  // Load dyanmic safety parameters
  emd::declare_or_get_param<bool>(
    allow_replan,
    "dynamic_safety.allow_replan",
    node, LOGGER, false);

  // Load dyanmic safety parameters
  emd::declare_or_get_param<bool>(
    visualize,
    "dynamic_safety.visualize",
    node, LOGGER, false);

  // -------------- Detailed parameters that needs to set manually --------------

  emd::declare_or_get_param<double>(
    safety_zone_options.look_ahead_time,
    "dynamic_safety.look_ahead_time",
    node, LOGGER);

  emd::declare_or_get_param<std::string>(
    environment_joint_states_topic,
    "dynamic_safety.environment_joint_states_topic",
    node, LOGGER);

  // Load collision checker parameters
  emd::declare_or_get_param<bool>(
    collision_checker_options.distance,
    "dynamic_safety.collision_checker.distance",
    node, LOGGER, false);  // default: false

  emd::declare_or_get_param<bool>(
    collision_checker_options.continuous,
    "dynamic_safety.collision_checker.continuous",
    node, LOGGER, false);  // default: false

  emd::declare_or_get_param<double>(
    collision_checker_options.step,
    "dynamic_safety.collision_checker.step",
    node, LOGGER);

  // TODO(anyone): padding

  // -------------- Load overwritable parameters -------------------
  // If the following parameters are defined and greater than zero,
  // dynamic parameterization will not change these parameters
  emd::declare_or_get_param<double>(
    rate,
    "dynamic_safety.rate",
    node, LOGGER, 0);

  emd::declare_or_get_param<double>(
    safety_zone_options.slow_down_time,
    "dynamic_safety.slow_down_time",
    node, LOGGER, 0);

  // Unable to overwrite currently, it is detected by default
  // emd::declare_or_get_param<bool>(
  //   collision_checker_options.realtime,
  //   "dynamic_safety.collision_checker.realtime",
  //   node, LOGGER, true);  // default: true

  emd::declare_or_get_param<int>(
    collision_checker_options.thread_count,
    "dynamic_safety.collision_checker.thread_count",
    node, LOGGER);

  // -------------- Static parameters -------------------
  if (!dynamic_parameterization) {
    // emd::declare_or_get_param<std::string>(
    //   safety_zone_options.unit_type,
    //   "dynamic_safety.safety_zone.unit_type",
    //   node, LOGGER, "second");  // default: "second"

    // // Only second is enabled
    // if (safety_zone_options.unit_type != "second") {
    //   RCLCPP_WARN(
    //     LOGGER, "Wrong safety zone unit type: [%s], default to [second]",
    //     safety_zone_options.unit_type.c_str());
    //   safety_zone_options.unit_type = "second";
    // }
    // TODO(anyone): distance based collision checking
    // I don't think distance makes sense here.

  } else {
    // Dynamically detect realtime OS
    std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
    collision_checker_options.realtime = false;
    if (realtime_file.is_open()) {
      realtime_file >> collision_checker_options.realtime;
    }

    // Set thread count to max capability
    if (collision_checker_options.thread_count == 0) {
      collision_checker_options.thread_count =
        static_cast<int>(std::thread::hardware_concurrency()) / 2;
    }
  }

  // // Next point publisher parameters
  // This part is disabled currently due to we are using ros2-controllers directly
  // to control the next point
  //
  // emd::declare_or_get_param<std::string>(
  //   next_point_publisher_options.command_out_type,
  //   "next_point_publisher.command_out_type",
  //   node, LOGGER);

  // emd::declare_or_get_param<std::string>(
  //   next_point_publisher_options.command_out_topic,
  //   "next_point_publisher.command_out_topic",
  //   node, LOGGER);

  // emd::declare_or_get_param<bool>(
  //   next_point_publisher_options.publish_joint_position,
  //   "next_point_publisher.publish_joint_position",
  //   node, LOGGER, true);

  // emd::declare_or_get_param<bool>(
  //   next_point_publisher_options.publish_joint_velocity,
  //   "next_point_publisher.publish_joint_velocity",
  //   node, LOGGER, false);

  // emd::declare_or_get_param<bool>(
  //   next_point_publisher_options.publish_joint_effort,
  //   "next_point_publisher.publish_joint_effort",
  //   node, LOGGER, false);

  // // Replanner parameters
  // if (allow_replan) {
  //   emd::declare_or_get_param<std::string>(
  //     replanner_options.planner_name,
  //     "replanner.planner_name",
  //     node, LOGGER);
  // }

  if (visualize) {
    // TODO(Briancbn): scene synchronization
    // emd::declare_or_get_param<bool>(
    //   visualizer_options.publish_scene,
    //   "dynamic_safety.visualize.publish_scene",
    //   node, LOGGER, false);

    emd::declare_or_get_param<double>(
      visualizer_options.publish_frequency,
      "dynamic_safety.visualizer.publish_frequency",
      node, LOGGER, 10);

    emd::declare_or_get_param<double>(
      visualizer_options.step,
      "dynamic_safety.visualizer.step",
      node, LOGGER, 0.1);

    emd::declare_or_get_param<std::string>(
      visualizer_options.topic,
      "dynamic_safety.visualizer.topic",
      node, LOGGER);

    emd::declare_or_get_param<std::string>(
      visualizer_options.tcp_link,
      "dynamic_safety.visualizer.tcp_link",
      node, LOGGER);

    // TODO(Briancbn): scene synchronization
    // emd::declare_or_get_param<std::string>(
    //   visualizer_options.scene_topic,
    //   "dynamic_safety.visualize.scene_topic",
    //   node, LOGGER);
  }

  // Return idiom
  return *this;
}

void DynamicSafety::configure(
  const rclcpp::Node::SharedPtr & node)
{
  node_ = node;

  // Initialize flags
  started = false;
  activated_ = false;

  collision_checker_.configure(
    option_.collision_checker_options,
    option_.robot_description,
    option_.robot_description_semantic);

  if (option_.dynamic_parameterization && option_.rate == 0) {
    // Use the new polling function to estimate
    // * Running rate
    option_.rate = (2.0 * collision_checker_.polling(option_.safety_zone_options.look_ahead_time));
  }
  RCLCPP_INFO(
    LOGGER, "Dynamic safety will run at %dHz.",
    static_cast<int>(option_.rate));

  // Blind zone is the iteration period
  option_.safety_zone_options.collision_checking_deadline = 1.0 / option_.rate;

  if (!safety_zone_.set(option_.safety_zone_options)) {
    throw std::runtime_error("Wrong safety zone parameters");
  }

  // RCLCPP_INFO(LOGGER, "Configuring next point publisher");
  // next_point_publisher_.configure(
  //   rt, option_.next_point_publisher_options, node, option_.rate);

  // current_state_ = std::make_shared<moveit::core::RobotState>(rt->getRobotModel());

  // if (option_.allow_replan) {
  //   replanner_.configure(scene, rt, node, option_.replanner_options);
  // }

  if (option_.visualize) {
    visualizer_.configure(
      node, option_.visualizer_options,
      option_.safety_zone_options,
      option_.robot_description,
      option_.robot_description_semantic);
  }

  benchmark_stats.clear();

  // Reset Cache
  env_state_cache_.initRT(sensor_msgs::msg::JointState());
  current_time_cache_.initRT(0);
  scale_cache_.initRT(1);

  // double period = 1 / option_.rate;
  env_state_callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions env_state_sub_option;
  env_state_sub_option.callback_group = env_state_callback_group_;
  auto qos = rclcpp::QoS(2);

  env_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    option_.environment_joint_states_topic,
    qos,
    [ = ](sensor_msgs::msg::JointState::UniquePtr joint_state_msg) -> void
    {
      if (started) {
        update_state(std::move(joint_state_msg));
      }
    },
    env_state_sub_option
  );

  main_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  main_timer_ = node_->create_wall_timer(
    rclcpp::Duration::from_seconds(1.0 / option_.rate).to_chrono<std::chrono::nanoseconds>(),
    [ = ]() -> void {
      if (started) {
        if (pf_) {
          pf_->reset();
        }
        _main_loop();
        if (pf_) {
          pf_->lapse_and_record();
        }
      }
    },
    main_callback_group_);
}

void DynamicSafety::add_trajectory(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt)
{
  collision_checker_.add_trajectory(rt);
  if (option_.visualize) {
    visualizer_.add_trajectory(rt);
  }
  activated_ = true;
}


void DynamicSafety::update_time(double current_time)
{
  current_time_cache_.writeFromNonRT(current_time);
}

void DynamicSafety::update_state(const sensor_msgs::msg::JointState::SharedPtr & state)
{
  env_state_cache_.writeFromNonRT(*state);
}

void DynamicSafety::update_state(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_state)
{
  current_state_cache_.writeFromNonRT(CurrentState(joint_names, current_state));
}

double DynamicSafety::get_scale()
{
  return *scale_cache_.readFromRT();
}

void DynamicSafety::start()
{
  // next_point_publisher_.start();
  // RCLCPP_INFO(LOGGER, "Next point publisher started!");
  // collision_time_point_ = -1;

  if (activated_) {
    if (!started) {
      pf_ = new emd::TimeProfiler<>(5000);

      started = true;

      // visualizer_.start();
      sig_ = std::promise<void>();
      future_ = sig_.get_future();
      RCLCPP_INFO(LOGGER, "All started");
    } else {
      RCLCPP_WARN(LOGGER, "Already started");
    }
  } else {
    RCLCPP_ERROR(LOGGER, "Not configured!! Please call the configure() first");
  }
}

void DynamicSafety::wait()
{
  RCLCPP_INFO(LOGGER, "Waiting...");
  future_.wait();
  RCLCPP_INFO(LOGGER, "Successfully exit.");
}

void DynamicSafety::stop()
{
  // visualizer_.reset();
  // RCLCPP_INFO(
  //   LOGGER, "Next Point Publisher ended with %s status",
  //   (next_point_publisher_.get_status() == NextPointPublisher::SUCCEEDED) ?
  //   "SUCCEEDED" : "FAILED");
  // next_point_publisher_.reset();
  if (option_.visualize) {
    visualizer_.stop();
  }
  started = false;
  activated_ = false;
  // node_.reset();

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

void DynamicSafety::_main_loop()
{
  // Update environment
  collision_checker_.update(*env_state_cache_.readFromRT());

  // get scaled time point
  double current_time_point = *current_time_cache_.readFromRT();
  // RCLCPP_INFO(node_->get_logger(), "Current time: %f", current_time_point);
  collision_time_point_ = -1;

  // Check collision once
  collision_checker_.run_once(
    current_time_point,
    option_.safety_zone_options.look_ahead_time,
    collision_time_point_
  );

  double scale = *scale_cache_.readFromRT();
  // Collision Happens in the future
  if (collision_time_point_ >= current_time_point) {
    double scale_step = 0;
    if (option_.safety_zone_options.slow_down_time <= 0 && option_.dynamic_parameterization) {
      // Dynamically adjust slow down time
      SafetyZone::Option dynamic_option = option_.safety_zone_options;
      dynamic_option.slow_down_time =
        _cal_scale_time(*current_state_cache_.readFromRT(), 3, scale, 0.0001);

      // Dynamically set scale step
      if (dynamic_option.slow_down_time > 0) {
        safety_zone_.set(dynamic_option);

        // Update visualizer as well
        if (option_.visualize) {
          visualizer_.update(dynamic_option);
        }
        scale_step = (scale - 0.0001) * (1.0 / option_.rate) / dynamic_option.slow_down_time;
      } else if (scale != 0.0001) {
        RCLCPP_ERROR(
          LOGGER,
          "There is no velocity state feedback from the robot. "
          "Please check /joint_states for velocity values!!"
          "Hard set the slow down time to 0.5s instead");
        option_.safety_zone_options.slow_down_time = 0.5;
        safety_zone_.set(option_.safety_zone_options);
        // Update visualizer as well
        if (option_.visualize) {
          visualizer_.update(option_.safety_zone_options);
        }
        scale_step = 2 * option_.rate;
      }
    } else {
      // Static Scale step
      scale_step = 1.0 * (1.0 / option_.rate) / option_.safety_zone_options.slow_down_time;
    }
    uint8_t zone = safety_zone_.get_zone(collision_time_point_ - current_time_point);

    if (!option_.allow_replan) {
      // No replanning
      if (zone <= SafetyZone::EMERGENCY) {
        // Emergency stop
        RCLCPP_ERROR(LOGGER, "Emergency stop!!");
        scale = 0.0001;
      } else {
        // Slow down
        RCLCPP_WARN(LOGGER, "Slowing down!!");
        scale -= scale_step;
        scale = std::max<double>(scale, 0.0001);
        RCLCPP_WARN(LOGGER, "Scale: %.2f", scale);
      }
    } else {
      // Replanning
    }
  } else {
    // No Collision
    if (scale < 1.0) {
      // Gradually rescale back to 1
      double scale_time = _cal_scale_time(*current_state_cache_.readFromRT(), 3, scale, 1.0);
      scale += (1.0 - scale) * (1.0 / option_.rate) / scale_time;
      scale = std::min<double>(scale, 1);
      RCLCPP_WARN(LOGGER, "Scale: %.2f", scale);
    }
  }
  scale_cache_.writeFromNonRT(scale);

  if (option_.visualize) {
    visualizer_.update(current_time_point, collision_time_point_);
  }

  // switch (zone) {
  //   case SafetyZone::BLIND:
  //     next_point_publisher_.halt();
  //     RCLCPP_ERROR(LOGGER, "Obstacle too close, halt and abort.");
  //     return;
  //   case SafetyZone::EMERGENCY:
  //     next_point_publisher_.scale(1e-3, 0);
  //     RCLCPP_ERROR(LOGGER, "Emergency stop.");
  //     break;
  //   case SafetyZone::SLOWDOWN:
  //     if (!option_.allow_replan) {
  //       // Simply slow down to wait for restart
  //       next_point_publisher_.scale(1e-3, option_.safety_zone_options.slow_down_time);
  //       RCLCPP_WARN(LOGGER, "Slowing down.");
  //     } else {
  //       // Check if replan successfully
  //       if (replanner_.started()) {
  //         if (replanner_.get_result()) {
  //           if (current_time_point >= replan_time_point_) {
  //             const auto & new_traj = replanner_.get_trajectory();
  //             next_point_publisher_.update_traj(new_traj);
  //             collision_checker_.update_traj(new_traj, option_.collision_checker_options);
  //             visualizer_.update_trajectory(new_traj);
  //             replanner_.reset();
  //           } else {
  //             RCLCPP_WARN(LOGGER, "Haven't reach replan time point: %f", replan_time_point_);
  //           }
  //         } else {
  //           RCLCPP_WARN(LOGGER, "Why?");
  //         }
  //       } else {
  //         double replan_buffer = collision_time_point_ - current_time_point -
  //           option_.safety_zone_options.collision_checking_deadline -
  //           option_.safety_zone_options.slow_down_time -
  //           1.0 / option_.rate;
  //         if (replan_buffer > 0) {
  //           double scale =
  //             replan_buffer / option_.safety_zone_options.replan_deadline;

  //           next_point_publisher_.scale(
  //             scale, option_.safety_zone_options.slow_down_time);
  //           replan_time_point_ = current_time_point +
  //             scale * option_.safety_zone_options.replan_deadline;
  //         } else {
  //           next_point_publisher_.scale(
  //             1e-3, replan_buffer / 2);
  //           replan_time_point_ = current_time_point;
  //         }

  //         // Start replan
  //         replanner_.update(joint_state_msg);
  //         replanner_.start(
  //           replan_time_point_,
  //           option_.safety_zone_options.replan_deadline);
  //         RCLCPP_WARN(LOGGER, "Start slow down and replannig.");
  //       }
  //     }
  //     break;
  //   case SafetyZone::REPLAN:
  //     if (option_.allow_replan) {
  //       // Check if replan successfully
  //       if (replanner_.started()) {
  //         if (replanner_.get_result()) {
  //           if (current_time_point >= replan_time_point_) {
  //             const auto & new_traj = replanner_.get_trajectory();
  //             RCLCPP_INFO(LOGGER, "Update next point");
  //             next_point_publisher_.update_traj(new_traj);
  //             RCLCPP_INFO(LOGGER, "Update collision checker");
  //             collision_checker_.update_traj(new_traj, option_.collision_checker_options);
  //             visualizer_.update_trajectory(new_traj);
  //             replanner_.reset();
  //           } else {
  //             RCLCPP_WARN(LOGGER, "Haven't reach replan time point: %f", replan_time_point_);
  //           }
  //         }
  //       } else {
  //         // Start replan
  //         replanner_.update(joint_state_msg);
  //         replan_time_point_ =
  //         current_time_point + option_.safety_zone_options.replan_deadline - 1.0 / option_.rate;
  //         replanner_.start(
  //           replan_time_point_,
  //           option_.safety_zone_options.replan_deadline);
  //         RCLCPP_WARN(LOGGER, "Start replanning.");
  //       }
  //     } else {
  //       // Simply slow down to wait for restart
  //       next_point_publisher_.scale(1e-3, option_.safety_zone_options.slow_down_time);
  //       RCLCPP_WARN(LOGGER, "Replan Slowing down.");
  //     }
  //     break;
  //   case SafetyZone::SAFE:
  //     // back to original speed if slowed
  //     RCLCPP_WARN_ONCE(LOGGER, "SAFE");
  //     if (next_point_publisher_.get_scale() < 1.0) {
  //       RCLCPP_WARN(LOGGER, "Back to original speed");
  //       next_point_publisher_.scale(1.0, option_.safety_zone_options.slow_down_time);
  //     }
  //     break;
  // }

  // double lapse_time = pf_->lapse_and_record();
  // if (lapse_time > 1.0 / option_.rate) {
  //   RCLCPP_ERROR(
  //     LOGGER, "Lapse time %fs exceeds maximum rate %fHz",
  //     lapse_time, option_.rate);
  //   // Simply slow down to wait for restart
  //   next_point_publisher_.scale(1e-3, option_.safety_zone_options.slow_down_time);
  //   RCLCPP_WARN_ONCE(LOGGER, "Slowing down.");

  //   // Simply slow down to wait for restart
  //   next_point_publisher_.scale(1e-3, option_.safety_zone_options.slow_down_time);
  //   RCLCPP_WARN_ONCE(LOGGER, "Slowing down.");
  // } else {
  //   rclcpp::sleep_for(
  //     rclcpp::Duration::from_seconds(
  //       1.0 / option_.rate - lapse_time).to_chrono<std::chrono::nanoseconds>());
  // }

  // next_point_publisher_.run_once();

  // auto status = next_point_publisher_.get_status();

  // if (status != NextPointPublisher::RUNNING) {
  //   stop();
  // }
}


}  // namespace dynamic_safety
