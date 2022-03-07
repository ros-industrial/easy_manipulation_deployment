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

#ifndef EMD__DYNAMIC_SAFETY__REPLANNER_HPP_
#define EMD__DYNAMIC_SAFETY__REPLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "emd/dynamic_safety/replanner_common.hpp"

namespace dynamic_safety
{

/// Collision checking context to-be-inherited.
class Replanner
{
public:
  /// Constructor
  Replanner();

  /// Destructor
  virtual ~Replanner();

  /// Replanning implementation pointer.
  class Impl;

  /// Configure the replanner to prepare for planning.
  /**
   * \param[in] option Collison checker options.
   * \param[in] node to inherite parameters from (MoveIt)
   * \param[in] robot_urdf Robot URDF Model.
   * \param[in] robot_srdf Robot SRDFConfiguration
   */
  void configure(
    const ReplannerOption & option,
    const rclcpp::Node::SharedPtr & node,
    const std::string & robot_urdf,
    const std::string & robot_srdf);

  /// Run asynchronous job to start planning
  /**
   * \param[in] joint_names Joint names order for the start point.
   * \param[in] start_point Robot start state.
   * \param[in] end_point Robot end state.
   */
  void run_async(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint & start_point,
    const trajectory_msgs::msg::JointTrajectoryPoint & end_point);

  /// Start a new thread to terminate asynchronously
  /**
   * Upon finish this would set the status to IDLE
   */
  void terminate_async();

  /// Specialized replanning feature, adding reference trajectory
  /**
   * This function needs to be used before `run_async(start_state_time)`.
   *
   * \param[in] rt trajectory used as reference for replanning.
   */
  void add_trajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt);

  /// Specialized replanning feature, replan with a starting time on a referenced trajectory.
  /**
   * `add_trajectory()` needs to be called before using this function.
   *
   * \param[in] start_state_time start time of the reference
   * \param[in] end_state_time start time of the reference
   */
  void run_async(
    double start_state_time,
    double end_state_time = -1.0);

  /// Configure the replanner to prepare for planning.
  /**
   * \param[in] option Collison checker options.
   * \param[in] node to inherite parameters from (MoveIt)
   * \param[in] robot_urdf Robot URDF Model.
   * \param[in] robot_srdf Robot SRDFConfiguration
   */
  trajectory_msgs::msg::JointTrajectory::SharedPtr flatten_result(
    double current_time,
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_state);

  /// Get Replanner status
  /**
   * Status value defined as follows
   * ```cpp
   * struct Status
   * {
   *   static const uint8_t
   *     IDLE = 0,
   *     ONGOING = 1,
   *     SUCCEED = 2,
   *     TIMEOUT = 3;
   * };
   * ```
   * \return replanner status
   */
  uint8_t get_status() const;

  /// Get Replanner status
  /**
   * \return New trajectory
   */
  trajectory_msgs::msg::JointTrajectory::SharedPtr get_result();

  /// Update joint states
  /**
   * \param[in] joint_state Robot joint state to reference and update from.
   */
  void update(
    const sensor_msgs::msg::JointState & joint_states);

  /// Sync with MoveIt Scene
  /**
   * \param[in] scene_msg MoveIt scene to sync to.
   */
  void update(
    const moveit_msgs::msg::PlanningScene & scene_msg);

private:
  // Implementation Pointer
  std::unique_ptr<Impl> impl_ptr_;
};

}  // namespace dynamic_safety

#endif  // EMD__DYNAMIC_SAFETY__REPLANNER_HPP_
