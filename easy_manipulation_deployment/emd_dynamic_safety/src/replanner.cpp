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
#include <unordered_map>
#include <utility>
#include <vector>

#include "emd/dynamic_safety/replanner.hpp"
#include "emd/interpolate.hpp"
#ifdef EMD_DYNAMIC_SAFETY_MOVEIT
#include "emd/dynamic_safety/replanner_moveit.hpp"
#endif
#ifdef EMD_DYNAMIC_SAFETY_TESSERACT
#include "emd/dynamic_safety/replanner_tesseract.hpp"
#endif

namespace dynamic_safety
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dynamic_safety.replanner");

class Replanner::Impl
{
public:
  Impl() = default;
  virtual ~Impl() = default;

  void configure(
    const ReplannerOption & option,
    const rclcpp::Node::SharedPtr & node,
    const std::string & robot_urdf,
    const std::string & robot_srdf)
  {
    if (option.framework == "moveit") {
#ifdef EMD_DYNAMIC_SAFETY_MOVEIT
      context_ = std::make_unique<dynamic_safety_moveit::MoveitReplannerContext>(
        robot_urdf, robot_srdf, option, node);
#endif
    } else if (option.framework == "tesseract") {
#ifdef EMD_DYNAMIC_SAFETY_TESSERACT
      context_ = std::make_unique<dynamic_safety_tesseract::TesseractReplannerContext>(
        robot_urdf, robot_srdf, option, node);
#endif
    }
    deadline_ = option.deadline;
  }

  void run_async(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint & start_point,
    const trajectory_msgs::msg::JointTrajectoryPoint & end_point)
  {
    start_time_ = std::chrono::steady_clock::now();
    plan_future_ = std::async(
      std::launch::async, &Impl::_run, this,
      joint_names, start_point, end_point);
  }

  void terminate_async()
  {
    if (!terminate_future_.valid()) {
      _start_async_termination_thread();
    } else {
      auto status = terminate_future_.wait_for(std::chrono::nanoseconds(0));
      if (status == std::future_status::ready) {
        terminate_future_.get();
        terminate_future_ = std::future<void>();
      }
    }
  }

  void add_trajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt)
  {
    // Deep copy;
    reference_trajectory_ = *rt;
  }

  void run_async(
    double start_state_time, double end_state_time)
  {
    trajectory_msgs::msg::JointTrajectoryPoint start_state, end_state;
    if (end_state_time < 0) {
      end_state = reference_trajectory_.points.back();
      end_state_time_ = rclcpp::Duration(end_state.time_from_start).seconds();
    } else {
      if (start_state_time >= end_state_time) {
        return;
      }
      start_state_time_ = start_state_time;
      end_state_time_ = end_state_time;
    }

    // Acquire start_state
    size_t num_points = reference_trajectory_.points.size();
    size_t before, after;
    size_t i = 0;
    for (; i < num_points; i++) {
      if (rclcpp::Duration(reference_trajectory_.points[i].time_from_start).seconds() >=
        start_state_time_)
      {
        before = std::max<size_t>((i == 0) ? 0 : (i - 1), 0);  // Avoid unsigned int 0 minus 1
        after = std::min<size_t>(i, num_points - 1);
        emd::core::interpolate_between_points(
          reference_trajectory_.points[before].time_from_start,
          reference_trajectory_.points[before],
          reference_trajectory_.points[after].time_from_start,
          reference_trajectory_.points[after],
          rclcpp::Duration::from_seconds(start_state_time_), start_state);
        RCLCPP_ERROR(LOGGER, "start_state:");
        for (auto & position : start_state.positions) {
          RCLCPP_ERROR(LOGGER, "start_state: %f", position);
        }
        break;
      }
    }
    // Acquire end_state
    if (end_state_time_ > 0) {
      for (; i < num_points; i++) {
        if (rclcpp::Duration(reference_trajectory_.points[i].time_from_start).seconds() >=
          end_state_time_)
        {
          before = std::max<size_t>((i == 0) ? 0 : (i - 1), 0);  // Avoid unsigned int 0 minus 1
          after = std::min<size_t>(i, num_points - 1);
          emd::core::interpolate_between_points(
            reference_trajectory_.points[before].time_from_start,
            reference_trajectory_.points[before],
            reference_trajectory_.points[after].time_from_start,
            reference_trajectory_.points[after],
            rclcpp::Duration::from_seconds(end_state_time_), end_state);
          break;
        }
      }
      if (i == num_points) {
        end_state = reference_trajectory_.points.back();
      }
    }
    run_async(reference_trajectory_.joint_names, start_state, end_state);
  }

  trajectory_msgs::msg::JointTrajectory::SharedPtr flatten_result(
    double current_time,
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint & /*current_state*/)
  {
    // Sort current state to plan joint_names
    // gather current name ordering of plan joint name
    // O(n^2) vs O(n) -> current algo
    auto reorder_joint = [](
      const std::vector<std::string> & reference_joint_order,
      std::vector<std::string> & current_joint_order,
      trajectory_msgs::msg::JointTrajectoryPoint & state) {
        std::unordered_map<std::string, size_t> ref_joint_idx_map;
        std::vector<size_t> joint_permutation;
        for (size_t i = 0; i < reference_joint_order.size(); i++) {
          ref_joint_idx_map[reference_joint_order[i]] = i;
        }
        // check the order that joint names is in
        for (auto & joint_name : current_joint_order) {
          joint_permutation.push_back(ref_joint_idx_map[joint_name]);
        }
        // Apply permutation order
        for (size_t i = 0; i < current_joint_order.size(); i++) {
          while (joint_permutation[i] != i) {
            std::swap(current_joint_order[joint_permutation[i]], current_joint_order[i]);
            if (!state.positions.empty()) {
              std::swap(
                state.positions[joint_permutation[i]],
                state.positions[i]);
            }
            if (!state.velocities.empty()) {
              std::swap(
                state.velocities[joint_permutation[i]],
                state.velocities[i]);
            }
            if (!state.accelerations.empty()) {
              std::swap(
                state.accelerations[joint_permutation[i]],
                state.accelerations[i]);
            }
            std::swap(joint_permutation[joint_permutation[i]], joint_permutation[i]);
          }
        }
      };
    size_t i = 0;
    size_t num_points = reference_trajectory_.points.size();
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> start_segment;
    std::vector<std::string> sorted_joint_names = joint_names;

    // TODO(Briancbn): add current state
    /*
    trajectory_msgs::msg::JointTrajectoryPoint sorted_current_states = current_state;
    if (plan_.joint_names != joint_names) {
      reorder_joint(plan_.joint_names, sorted_joint_names, sorted_current_states);
    }
    */
    if (current_time < start_state_time_) {
      for (; i < num_points; i++) {
        double time_from_start =
          rclcpp::Duration(reference_trajectory_.points[i].time_from_start).seconds();
        if (current_time <= time_from_start && start_state_time_ > time_from_start) {
          if (reference_trajectory_.joint_names != plan_.joint_names) {
            // Re-order joint if necessary
            std::vector<std::string> sorted_ref_joint_names = reference_trajectory_.joint_names;
            trajectory_msgs::msg::JointTrajectoryPoint sorted_point =
              reference_trajectory_.points[i];
            reorder_joint(plan_.joint_names, sorted_ref_joint_names, sorted_point);
            start_segment.push_back(sorted_point);
          } else {
            start_segment.push_back(reference_trajectory_.points[i]);
          }
          // Clear time
          start_segment.back().time_from_start = rclcpp::Duration::from_seconds(0.0);
        } else if (time_from_start >= start_state_time_) {
          break;
        }
      }
      plan_.points.insert(plan_.points.begin(), start_segment.begin(), start_segment.end());
    }

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> end_segment;
    for (; i < num_points; i++) {
      double time_from_start =
        rclcpp::Duration(reference_trajectory_.points[i].time_from_start).seconds();
      if (time_from_start > end_state_time_) {
        if (reference_trajectory_.joint_names != plan_.joint_names) {
          // Re-order joint if necessary
          std::vector<std::string> sorted_ref_joint_names = reference_trajectory_.joint_names;
          trajectory_msgs::msg::JointTrajectoryPoint sorted_point = reference_trajectory_.points[i];
          reorder_joint(plan_.joint_names, sorted_ref_joint_names, sorted_point);
          end_segment.push_back(sorted_point);
        } else {
          end_segment.push_back(reference_trajectory_.points[i]);
        }
        // Clear time
        end_segment.back().time_from_start = rclcpp::Duration::from_seconds(0.0);
      }
    }
    plan_.points.insert(plan_.points.end(), end_segment.begin(), end_segment.end());

    // TODO(anyone): Update start state speed
    if (context_->time_parameterize(plan_)) {
      return std::make_shared<trajectory_msgs::msg::JointTrajectory>(plan_);
    } else {
      return std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    }
  }

  uint8_t get_status() const
  {
    if (plan_future_.valid()) {
      auto status = plan_future_.wait_for(std::chrono::nanoseconds(0));
      double time_passed = std::chrono::duration<double, std::ratio<1>>(
        std::chrono::steady_clock::now() - start_time_).count();
      switch (status) {
        case std::future_status::deferred:
          return ReplannerStatus::IDLE;
        case std::future_status::ready:
          return ReplannerStatus::SUCCEED;
        case std::future_status::timeout:
          return (time_passed > deadline_) ? ReplannerStatus::TIMEOUT : ReplannerStatus::ONGOING;
        default:
          return ReplannerStatus::IDLE;
      }
    } else {
      return ReplannerStatus::IDLE;
    }
  }

  trajectory_msgs::msg::JointTrajectory::SharedPtr get_result()
  {
    if (!plan_future_.valid()) {
      return std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    }
    plan_ = plan_future_.get();
    // Reset shared future
    plan_future_ = std::shared_future<trajectory_msgs::msg::JointTrajectory>();
    RCLCPP_INFO(
      LOGGER,
      "Total time take: %.5fs",
      std::chrono::duration<double, std::ratio<1>>(
        std::chrono::steady_clock::now() - start_time_).count());
    return std::make_shared<trajectory_msgs::msg::JointTrajectory>(plan_);
  }

  void update(
    const sensor_msgs::msg::JointState & joint_states)
  {
    context_->update(joint_states);
  }


  void update(
    const moveit_msgs::msg::PlanningScene & scene_msg)
  {
    context_->update(scene_msg);
  }

private:
  trajectory_msgs::msg::JointTrajectory _run(
    const std::vector<std::string> joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint start_point,
    const trajectory_msgs::msg::JointTrajectoryPoint end_point)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    context_->run(joint_names, start_point, end_point, trajectory);
    return trajectory;
  }

  // Calling blocking get in a new async until it ended
  // This will result the future to invalid
  void _start_async_termination_thread()
  {
    terminate_future_ = std::async(
      [this]() -> void {
        plan_future_.get();
        RCLCPP_ERROR(
          LOGGER,
          "Unfortunately, Total time take: %.5fs",
          std::chrono::duration<double, std::ratio<1>>(
            std::chrono::steady_clock::now() - start_time_).count());
        // Make it invalid;
        plan_future_ = std::shared_future<trajectory_msgs::msg::JointTrajectory>();
      });
  }
  std::unique_ptr<ReplannerContext> context_;
  std::shared_future<trajectory_msgs::msg::JointTrajectory> plan_future_;
  std::future<void> terminate_future_;

  // Specialized planning feature
  trajectory_msgs::msg::JointTrajectory reference_trajectory_;
  double start_state_time_, end_state_time_;
  trajectory_msgs::msg::JointTrajectory plan_;

  double deadline_;
  std::chrono::steady_clock::time_point start_time_;
};

Replanner::Replanner()
: impl_ptr_(std::make_unique<Impl>())
{
}

Replanner::~Replanner()
{
}

void Replanner::configure(
  const ReplannerOption & option,
  const rclcpp::Node::SharedPtr & node,
  const std::string & robot_urdf,
  const std::string & robot_srdf)
{
  impl_ptr_->configure(option, node, robot_urdf, robot_srdf);
}

void Replanner::run_async(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & start_point,
  const trajectory_msgs::msg::JointTrajectoryPoint & end_point)
{
  impl_ptr_->run_async(joint_names, start_point, end_point);
}

void Replanner::add_trajectory(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt)
{
  impl_ptr_->add_trajectory(rt);
}

void Replanner::run_async(
  double start_state_time,
  double end_state_time)
{
  impl_ptr_->run_async(start_state_time, end_state_time);
}

void Replanner::terminate_async()
{
  impl_ptr_->terminate_async();
}

trajectory_msgs::msg::JointTrajectory::SharedPtr Replanner::flatten_result(
  double current_time,
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_state)
{
  return impl_ptr_->flatten_result(current_time, joint_names, current_state);
}

void Replanner::update(
  const sensor_msgs::msg::JointState & msg)
{
  impl_ptr_->update(msg);
}

void Replanner::update(
  const moveit_msgs::msg::PlanningScene & msg)
{
  impl_ptr_->update(msg);
}

uint8_t Replanner::get_status() const
{
  return impl_ptr_->get_status();
}

trajectory_msgs::msg::JointTrajectory::SharedPtr Replanner::get_result()
{
  return impl_ptr_->get_result();
}
}  // namespace dynamic_safety
