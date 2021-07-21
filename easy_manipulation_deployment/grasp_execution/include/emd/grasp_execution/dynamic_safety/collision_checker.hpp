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

#ifndef EMD__GRASP_EXECUTION__DYNAMIC_SAFETY__COLLISION_CHECKER_HPP_
#define EMD__GRASP_EXECUTION__DYNAMIC_SAFETY__COLLISION_CHECKER_HPP_

#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit/robot_state/robot_state.h"
#include "moveit/planning_scene/planning_scene.h"

namespace grasp_execution
{

namespace dynamic_safety
{

/// Collision checker for dynamic safety.
class CollisionChecker
{
public:
  /// Collision checker options.
  struct Option
  {
    /// Whether to use distance based collison checking (not fully implemented).
    bool distance;

    /// Continuous collison checker (not implemented).
    bool continuous;

    /// Enable realtime configuration (not fully implemented).
    bool realtime;

    /// Steps (in the unit of time) between states taken from the trajectory (discrete only).
    double step;
    /// Thread count used for collision checking (discrete only).
    int thread_count;
  };

  /// Collision checking context, each for one thread.
  class Context;

  /// Constructor.
  CollisionChecker();

  /// Destructor.
  ~CollisionChecker();

  /// Configure the collision checker to prepare for collision checking.
  /**
   * \param[in] option Collison checker options.
   * \param[in] scene Moveit scene to use / clone for collision checking.
   * \param[in] rt Robot trajectory to check collision for.
   */
  void configure(
    const Option & option,
    const planning_scene::PlanningScenePtr & scene,
    const robot_trajectory::RobotTrajectoryPtr & rt);

  /// Update the rest of the robot state beyond the detector group.
  /**
   * \param[in] state Robot state to reference and update from.
   */
  void update(
    const moveit::core::RobotState & state);

  /// Run collision checking once.
  /**
   * \param[in] point Current time point.
   * \param[in] look_ahead_time How far to check beyond the current time point.
   * \param[out] collision_point Collision time point, -1 if no collision found.
   */
  void run_once(
    double point,
    double look_ahead_time,
    double & collision_point);

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
   * \param[in] traj Trajectory to replace the current one.
   * \param[in] option Collision checking option to update.
   */
  void update_traj(
    const robot_trajectory::RobotTrajectoryPtr & traj,
    const Option & option);

private:
  // Main function for running discrete collision checking
  void _discrete_runner_fn(int runner_id);

  // robot states to verify
  std::vector<moveit::core::RobotState> states_;
  std::vector<double> duration_from_start_;
  double step_;

  // Run result
  // cannot use bool https://stackoverflow.com/a/25194424
  // also bool is not thread safe
  std::vector<uint8_t> results_;
  std::vector<double> distances_;

  // Context
  std::vector<std::unique_ptr<Context>> contexts_;

  // thread handling / Synchronization variables
  // Similar to https://stackoverflow.com/a/53274193/13517633
  std::vector<std::shared_ptr<std::thread>> runners_;
  std::condition_variable init_cv_;
  std::mutex init_m_;
  int n_active_workers_;
  int current_iteration_;
  int thread_count_;

  // Atomic variables to monitor collision checking progress
  std::atomic_int itr_;
  std::atomic_int itr_end_;

  // Atomic variables to control thread starting and ending
  std::atomic_bool started_;
};

}  // namespace dynamic_safety

}  // namespace grasp_execution
#endif  // EMD__GRASP_EXECUTION__DYNAMIC_SAFETY__COLLISION_CHECKER_HPP_
