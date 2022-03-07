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

#ifndef EMD__DYNAMIC_SAFETY__NEXT_POINT_PUBLISHER_HPP_
#define EMD__DYNAMIC_SAFETY__NEXT_POINT_PUBLISHER_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit/robot_trajectory/robot_trajectory.h"

#include "std_msgs/msg/float64_multi_array.hpp"


namespace dynamic_safety
{

/// Next point publisher for dynamic safety.
class NextPointPublisher
{
public:
  /// Next Point Publisher options.
  struct Option
  {
    /// Command out type, currently support
    /// `trajectory_msgs/JointTrajectory` and
    /// `std_msgs/Float64MultiArray`.
    std::string command_out_type;

    /// Command out topic type.
    std::string command_out_topic;

    // Joint output type.
    /// Whether to publish joint position.
    bool publish_joint_position;

    // TODO(anyone): Implement velocity and effort.
    /// Whether to publish joint velocity (not implemented).
    bool publish_joint_velocity;
    /// Whether to publish joint effort (not implemented).
    bool publish_joint_effort;
  };

  /// Command type.
  enum class Command : uint8_t
  {
    TRAJECTORY,
    ARRAY
  };

  /// Status Macro.
  static const int8_t
    IDLE = 0,
    SUCCEEDED = 1,
    FAILED = 4,
    RUNNING = 5;

  /// Constructor.
  NextPointPublisher();

  /// Destructor.
  ~NextPointPublisher();

  /// Configure the next point publisher to start controlling the robot.
  /**
   * rate will be how far ahead the next point will be relative to the current time stamp.
   *
   * \param[in] traj Trajectory to follow.
   * \param[in] option Next point publisher options.
   * \param[in] node ROS Node to use for the publisher.
   * \param[in] rate How fast the outer loop will run.
   */
  void configure(
    const robot_trajectory::RobotTrajectoryPtr & traj,
    const Option & option,
    const rclcpp::Node::SharedPtr & node,
    double rate);

  /// Start the next point publisher.
  /**
   * \param[in] scale The speed scale at which the next point publisher will start with.
   */
  void start(double scale = 1);

  /// Scale the next point publisher speed.
  /**
   * Configure the next point publisher to change the speed scale to the target scale
   * within given time. If given time is shorter than given rate, the scale will be
   * set immediately, otherwise it will decrease linearly.
   *
   * \param[in] scale The target scale to reach.
   * \param[in] time_to_scale Desired time to reach that scale.
   */
  void scale(double scale, double time_to_scale);

  /// Stop the next point publisher.
  /**
   * This is equivalent as calling `scale(1e-3, 0)`.
   */
  void stop();

  /// Halt and reset the next point publisher.
  void halt();

  /// Reset / stop the next point publisher.
  void reset();

  /// Update the trajectory after next point publisher has started.
  /**
   * This cannot run parallel with run_once().
   *
   * \param[in] traj Trajectory to replace the current one.
   */
  void update_traj(
    const robot_trajectory::RobotTrajectoryPtr & traj);

  /// Get execution status.
  /**
   * This is thread safe.
   * Available status
   * ```
   * IDLE
   * SUCCEEDED
   * RUNNING
   * FAILED
   * ```
   *
   * \return Current execution status.
   */
  int8_t get_status() const
  {
    return static_cast<int8_t>(status_);
  }

  /// Get scale.
  /**
   * This is NOT thread safe.
   * Target scale if it is in the process of scaling,
   * otherwise it will return the current scale.
   *
   * \return Execution scale.
   */
  double get_scale() const
  {
    // return target scale
    return scale_ - scale_step_ * remaining_steps_to_scale_;
  }

  /// Get the current time point.
  /**
   * This time point is where the robot is on a trajectory, NOT absolute time.
   * This time point will take into scale into account.
   *
   * \return Time point reference.
   */
  const double & get_time_point() const
  {
    return time_point_;
  }

  /// Run the next point publisher once.
  /**
   * This should always run 1 / rate after receiving the joint state update.
   * Command will be published to the controller or driver.
   */
  void run_once();

  /// Update and get the current time point.
  /**
   * This time point is where the robot is on a trajectory, NOT absolute time.
   * This time point will take into scale into account.
   *
   * \return Time point.
   */
  const double & current_point();

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_out_array_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_out_traj_pub_;

private:
  // Get next point based on current time point.
  void _next_point();

  // Send command to the controller or driver.
  void _send_command();

  // Update start time and internal scale with the new scale.
  void _scale_impl(double scale);

  // Utility function to convert command type from string to enum.
  Command _from_string(const std::string & command_type);

  // Deadline callback (not implemented)
  void _deadline_cb(rclcpp::QOSDeadlineOfferedInfo &);

  // Time points for get the scaled and recalculated current time point.
  typedef std::chrono::steady_clock clock;
  typedef std::chrono::time_point<clock> TimePoint;
  TimePoint start_time_;
  TimePoint scale_time_;
  TimePoint end_time_;

  // 1 / rate.
  double period_;

  // Current trajectory.
  robot_trajectory::RobotTrajectoryPtr traj_;

  // command.
  Command command_out_type_;
  std::string command_out_topic_;
  double time_point_;
  moveit::core::RobotStatePtr point_;
  std::vector<double> command_out_;
  trajectory_msgs::msg::JointTrajectory command_out_traj_;
  std_msgs::msg::Float64MultiArray command_out_array_;

  // status thread safe.
  std::atomic_int8_t status_;

  // Attributes to assist scaling capability.
  double scale_;
  int remaining_steps_to_scale_;
  double scale_step_;

  // Joint output type.
  bool publish_joint_position_;

  // TODO(anyone): Implement velocity and effort.
  bool publish_joint_velocity_;
  bool publish_joint_effort_;
};

}  // namespace dynamic_safety


#endif  // EMD__DYNAMIC_SAFETY__NEXT_POINT_PUBLISHER_HPP_
