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
#include <vector>

#include "emd/dynamic_safety/dynamic_safety_trajectory_controller.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"

namespace dynamic_safety
{
controller_interface::InterfaceConfiguration
DynamicSafetyTrajectoryController::state_interface_configuration() const
{
  return JointTrajectoryController::state_interface_configuration();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DynamicSafetyTrajectoryController::on_configure(const rclcpp_lifecycle::State & state)
{
  auto result = JointTrajectoryController::on_configure(state);
  if (result ==
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    // Load safety officer configuration
    safety_officer_ = std::make_unique<DynamicSafety>(node_);

    // Remove existing subscriber and action monitor setup
    joint_command_subscriber_.reset();
    action_server_.reset();

    // Create new subscriber callback
    auto sub_callback =
      [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg) -> void {
        if (!validate_trajectory_msg(*msg)) {
          return;
        }

        // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
        // always replace old msg with new one for now
        if (subscriber_is_active_) {
          this->add_new_trajectory_msg(msg);
        }
      };

    joint_command_subscriber_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), sub_callback);

    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
      node_->get_node_base_interface(), node_->get_node_clock_interface(),
      node_->get_node_logging_interface(), node_->get_node_waitables_interface(),
      std::string(node_->get_name()) + "/follow_joint_trajectory",
      std::bind(&DynamicSafetyTrajectoryController::goal_callback, this, _1, _2),
      std::bind(&DynamicSafetyTrajectoryController::cancel_callback, this, _1),
      std::bind(&DynamicSafetyTrajectoryController::feedback_setup_callback, this, _1));

    safety_officer_->configure(node_);
    safety_officer_->set_new_trajectory_callback(
      std::bind(
        &DynamicSafetyTrajectoryController::add_new_trajectory_msg, this,
        std::placeholders::_1));
  }
  return result;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DynamicSafetyTrajectoryController::on_activate(const rclcpp_lifecycle::State & state)
{
  TimeData time_data;
  time_data.time = node_->now();
  time_data.period = rclcpp::Duration(0, 0);
  time_data.uptime = node_->now();
  time_data_.initRT(time_data);
  scaling_factor_.initRT(1.0);
  return JointTrajectoryController::on_activate(state);
}

controller_interface::return_type DynamicSafetyTrajectoryController::update()
{
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  auto resize_joint_trajectory_point =
    [&](trajectory_msgs::msg::JointTrajectoryPoint & point, size_t size) {
      point.positions.resize(size);
      if (has_velocity_state_interface_) {
        point.velocities.resize(size);
      }
      if (has_acceleration_state_interface_) {
        point.accelerations.resize(size);
      }
    };

  auto compute_error_for_joint =
    [&](JointTrajectoryPoint & error, int index, const JointTrajectoryPoint & current,
      const JointTrajectoryPoint & desired) {
      // error defined as the difference between current and desired
      size_t uindex = static_cast<size_t>(index);
      error.positions[uindex] = angles::shortest_angular_distance(
        current.positions[uindex],
        desired.positions[uindex]);
      if (has_velocity_state_interface_ && has_velocity_command_interface_) {
        error.velocities[uindex] = desired.velocities[uindex] - current.velocities[uindex];
      }
      if (has_acceleration_state_interface_ && has_acceleration_command_interface_) {
        error.accelerations[uindex] = desired.accelerations[uindex] - current.accelerations[uindex];
      }
    };

  // Main Speed scaling difference...
  // Adjust time with scaling factor
  TimeData time_data;
  time_data.time = node_->now();
  double scale = safety_officer_->get_scale();
  scaling_factor_.writeFromNonRT(scale);
  rcl_duration_value_t period = (time_data.time - time_data_.readFromRT()->time).nanoseconds();
  time_data.period = rclcpp::Duration(period) * (*scaling_factor_.readFromRT());
  time_data.uptime = time_data_.readFromRT()->uptime + time_data.period;
  rclcpp::Time traj_time = time_data_.readFromRT()->uptime + time_data.period;
  time_data_.writeFromNonRT(time_data);

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  if (current_external_msg != *new_external_msg) {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    (*new_external_msg)->header.stamp = time_data.uptime;
    traj_external_point_ptr_->update(*new_external_msg);
  }

  // TODO(Briancbn): Trajectory replacement.
  JointTrajectoryPoint state_current, state_desired, state_error;
  const auto joint_num = joint_names_.size();
  resize_joint_trajectory_point(state_current, joint_num);

  // Current state update
  auto assign_point_from_interface =
    [&, joint_num](std::vector<double> & trajectory_point_interface,
      const std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> &
      joint_interface) {
      for (size_t index = 0; index < joint_num; ++index) {
        trajectory_point_interface[index] = joint_interface[index].get().get_value();
      }
    };

  auto assign_interface_from_point = [&,
      joint_num](std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> &
      joint_interface, const std::vector<double> & trajectory_point_interface) {
      for (size_t index = 0; index < joint_num; ++index) {
        joint_interface[index].get().set_value(trajectory_point_interface[index]);
      }
    };

  state_current.time_from_start.set__sec(0);

  // Assign values from the hardware
  // Position states always exist
  assign_point_from_interface(state_current.positions, joint_state_interface_[0]);
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_) {
    assign_point_from_interface(state_current.velocities, joint_state_interface_[1]);
    // Acceleration is used only in combination with velocity
    if (has_acceleration_state_interface_) {
      assign_point_from_interface(state_current.accelerations, joint_state_interface_[2]);
    } else {
      // Make empty so the property is ignored during interpolation
      state_current.accelerations.clear();
    }
  } else {
    // Make empty so the property is ignored during interpolation
    state_current.velocities.clear();
    state_current.accelerations.clear();
  }

  // currently carrying out a trajectory
  if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg()) {
    // if sampling the first time, set the point before you sample
    if (!(*traj_point_active_ptr_)->is_sampled_already()) {
      (*traj_point_active_ptr_)->set_point_before_trajectory_msg(traj_time, state_current);
      safety_officer_->start();
    }
    double current_time =
      (traj_time - (*traj_point_active_ptr_)->get_trajectory_start_time()).seconds();
    safety_officer_->update_time(current_time);
    safety_officer_->update_state(joint_names_, state_current);
    resize_joint_trajectory_point(state_error, joint_num);

    // find segment for current timestamp
    joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    const bool valid_point =
      (*traj_point_active_ptr_)->sample(
      traj_time, state_desired, start_segment_itr,
      end_segment_itr);

    if (valid_point) {
      bool abort = false;
      bool outside_goal_tolerance = false;
      const bool before_last_point = end_segment_itr != (*traj_point_active_ptr_)->end();

      // set values for next hardware write()
      if (has_position_command_interface_) {
        assign_interface_from_point(joint_command_interface_[0], state_desired.positions);
      }
      if (has_velocity_command_interface_) {
        assign_interface_from_point(joint_command_interface_[1], state_desired.velocities);
      }
      if (has_acceleration_command_interface_) {
        assign_interface_from_point(joint_command_interface_[2], state_desired.accelerations);
      }

      for (size_t index = 0; index < joint_num; ++index) {
        // set values for next hardware write()
        compute_error_for_joint(state_error, static_cast<int>(index), state_current, state_desired);

        if (before_last_point &&
          !check_state_tolerance_per_joint(
            state_error, static_cast<int>(index),
            default_tolerances_.state_tolerance[index], true))
        {
          abort = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (!before_last_point && !check_state_tolerance_per_joint(
            state_error, static_cast<int>(index), default_tolerances_.goal_state_tolerance[index],
            true))
        {
          outside_goal_tolerance = true;
        }
      }

      const auto active_goal = *rt_active_goal_.readFromRT();
      if (active_goal) {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = node_->now();
        feedback->joint_names = joint_names_;

        feedback->actual = state_current;
        feedback->desired = state_desired;
        feedback->error = state_error;
        active_goal->setFeedback(feedback);

        // check abort
        if (abort || outside_goal_tolerance) {
          auto result = std::make_shared<FollowJTrajAction::Result>();

          if (abort) {
            RCLCPP_WARN(node_->get_logger(), "Aborted due to state tolerance violation");
            result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          } else if (outside_goal_tolerance) {
            RCLCPP_WARN(node_->get_logger(), "Aborted due to goal tolerance violation");
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
          }
          safety_officer_->stop();
          active_goal->setAborted(result);

          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
        }

        // check goal tolerance
        if (!before_last_point) {
          if (!outside_goal_tolerance) {
            auto res = std::make_shared<FollowJTrajAction::Result>();
            res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            safety_officer_->stop();
            active_goal->setSucceeded(res);
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

            RCLCPP_INFO(node_->get_logger(), "Goal reached, success!");
          } else if (default_tolerances_.goal_time_tolerance != 0.0) {
            // if we exceed goal_time_toleralance set it to aborted
            const rclcpp::Time traj_start = (*traj_point_active_ptr_)->get_trajectory_start_time();
            const rclcpp::Time traj_end = traj_start + start_segment_itr->time_from_start;

            // TODO(anyone): This will break in speed scaling we have to discuss
            // how to handle the goal time when the robot scales itself down.
            const double difference = node_->now().seconds() - traj_end.seconds();
            if (difference > default_tolerances_.goal_time_tolerance) {
              auto result = std::make_shared<FollowJTrajAction::Result>();
              result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
              safety_officer_->stop();
              active_goal->setAborted(result);
              rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
              RCLCPP_WARN(
                node_->get_logger(), "Aborted due goal_time_tolerance exceeding by %f seconds",
                difference);
            }
          }
        }
      }
    }
  }
  // else {
  //   TimeData time_data;
  //   time_data.time = node_->now();
  //   time_data.uptime = node_->now();
  //   time_data_.writeFromNonRT(time_data);
  // }

  publish_state(state_desired, state_current, state_error);
  return controller_interface::return_type::OK;
}

void DynamicSafetyTrajectoryController::feedback_setup_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  // Update new trajectory
  {
    preempt_active_goal();
    auto traj_msg =
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

    this->add_new_trajectory_msg(traj_msg);  // This would be overriden
  }

  // Update the active goal
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->preallocated_feedback_->joint_names = joint_names_;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Setup goal status checking timer
  goal_handle_timer_ = node_->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::seconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

void DynamicSafetyTrajectoryController::add_new_trajectory_msg(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & traj_msg)
{
  safety_officer_->add_trajectory(traj_msg);
  traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
}


}  // namespace dynamic_safety

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dynamic_safety::DynamicSafetyTrajectoryController,
  controller_interface::ControllerInterface)
