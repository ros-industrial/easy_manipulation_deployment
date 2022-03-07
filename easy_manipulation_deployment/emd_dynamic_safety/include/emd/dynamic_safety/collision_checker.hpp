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

#ifndef EMD__DYNAMIC_SAFETY__COLLISION_CHECKER_HPP_
#define EMD__DYNAMIC_SAFETY__COLLISION_CHECKER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "emd/dynamic_safety/collision_checker_common.hpp"

namespace dynamic_safety
{

/// Collision checker for dynamic safety.
class CollisionChecker
{
public:
  /// Constructor.
  CollisionChecker();

  /// Destructor.
  ~CollisionChecker();

  /// Collision checking implementation pointer.
  class Impl;

  /// Configure the collision checker to prepare for collision checking.
  /**
   * \param[in] option Collison checker options.
   * \param[in] robot_urdf Robot URDF Model.
   * \param[in] robot_srdf Robot SRDFConfiguration
   */
  void configure(
    const CollisionCheckerOption & option,
    const std::string & robot_urdf,
    const std::string & robot_srdf);

  /// Update the rest of the robot state beyond the detector group.
  /**
   * \param[in] joint_state Robot joint state to reference and update from.
   */
  void update(
    const sensor_msgs::msg::JointState & joint_state);

  /// Update other information such as collision objects from the scene.
  /**
   * \param[in] scene_msg MoveIt scene message to sync to.
   */
  void update(
    const moveit_msgs::msg::PlanningScene & scene_msg);

  /// Run collision checking once.
  /**
   * \param[in] current_time Current time_from_start in the trajectory.
   * \param[in] look_ahead_time How far to check beyond the current time point.
   * \param[out] collision_time Collision time_from_start in the trajectory, -1 if no collision found.
   */
  void run_once(
    double current_time,
    double look_ahead_time,
    double & collision_time);

  /// Estimate collision checking duration for specific look_ahead_time
  /**
   * \param[in] look_ahead_time how far ahead collision checking should do.
   * \param[in] sample_size the number of sample to average
   * \return average collision checking duration
   */
  double polling(
    double look_ahead_time,
    int sample_size = 10);

  /// Reset / stop collision checker.
  /**
   * After this is called, configure() has to be called again before running run_once().
   *
   * \sa configure()
   * \sa run_once()
   */
  void stop();

  /// Reset / stop collision checker.
  /**
   * After this is called, configure() has to be called again before running run_once().
   *
   * \sa configure()
   * \sa run_once()
   */
  void reset();

  /// Update the trajectory after collision checker has started.
  /**
   * This cannot run parallel with run_once().
   * \param[in] rt Robot trajectory to replace the current one.
   */
  void add_trajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt);

private:
  // Implementation Pointer
  std::unique_ptr<Impl> impl_ptr_;
};

}  // namespace dynamic_safety

#endif  // EMD__DYNAMIC_SAFETY__COLLISION_CHECKER_HPP_
