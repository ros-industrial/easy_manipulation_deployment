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

#ifndef EMD__DYNAMIC_SAFETY__DYNAMIC_SAFETY_TRAJECTORY_CONTROLLER_HPP_
#define EMD__DYNAMIC_SAFETY__DYNAMIC_SAFETY_TRAJECTORY_CONTROLLER_HPP_

#include <memory>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#pragma GCC diagnostic pop
#include "joint_trajectory_controller/trajectory.hpp"
#include "emd/dynamic_safety/dynamic_safety.hpp"
#include "angles/angles.h"

namespace dynamic_safety
{
class DynamicSafetyTrajectoryController : public joint_trajectory_controller::
  JointTrajectoryController
{
public:
  DynamicSafetyTrajectoryController() = default;
  ~DynamicSafetyTrajectoryController() override = default;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  controller_interface::return_type update() override;

protected:
  struct TimeData
  {
    TimeData()
    : time(0.0), period(0.0), uptime(0.0)
    {
    }
    rclcpp::Time time;
    rclcpp::Duration period;
    rclcpp::Time uptime;
  };

  /// Override existing ones used in on_configure
  void add_new_trajectory_msg(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & traj_msg);

  // Reserve vtable for bind
  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal)
  {
    return JointTrajectoryController::goal_callback(uuid, goal);
  }

  // Reserve vtable for bind
  rclcpp_action::CancelResponse cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
  {
    return JointTrajectoryController::cancel_callback(goal_handle);
  }

  /// Override existing ones used in on_configure
  void feedback_setup_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

private:
  // Atomic vs realtime buffer?
  realtime_tools::RealtimeBuffer<double> scaling_factor_;
  realtime_tools::RealtimeBuffer<TimeData> time_data_;

  // Dynamic Safety
  DynamicSafety::UniquePtr safety_officer_;
};

}  // namespace dynamic_safety

#endif  // EMD__DYNAMIC_SAFETY__DYNAMIC_SAFETY_TRAJECTORY_CONTROLLER_HPP_
