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

#ifndef EMD__DYNAMIC_SAFETY__VISUALIZER_HPP_
#define EMD__DYNAMIC_SAFETY__VISUALIZER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "emd/dynamic_safety/safety_zone.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "moveit/planning_scene/planning_scene.h"

namespace dynamic_safety
{

/// Visualizer for dynamic safety.
class Visualizer
{
public:
  /// Option for visualizer.
  struct Option
  {
    /// Visualizer publish frequency.
    double publish_frequency;

    /// Visualizer trajectory display step / resolution.
    double step;

    /// Visualizer topic.
    std::string topic;

    std::string tcp_link;
  };

  /// Constructor.
  Visualizer();

  /// Configure and prepare the Visualizer
  /**
   * An internal safety zone is created.
   *
   * \param[in] rt Robot trajectory to display and track.
   * \param[in] node This is needed to create publisher and timer.
   * \param[in] option Visualizer option.
   * \param[in] zone_option This can be used to create safety zone.
   */
  void configure(
    const rclcpp::Node::SharedPtr & node,
    const Option & option,
    const SafetyZone::Option & zone_option,
    const std::string & robot_urdf,
    const std::string & robot_srdf);

  /// Update existing trajectory.
  /**
   * Update the trajectory being displayed.
   *
   * \param[in] rt Trajectory to update.
   */
  void add_trajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt);

  /// Update the current state
  /**
   * If there is no collision, collision time point will be -1.
   * \param[in] current_time_point Relative time point to start of the trajectory.
   * \param[in] collision_time_point Relative collision time point.
   */
  void update(
    double current_time_point,
    double collision_time_point = -1);

  /// Update
  /**
   * If there is no collision, collision time point will be -1.
   * \param[in] zone_option Update safety zone options if needed.
   */
  void update(
    const SafetyZone::Option & zone_option);

  /// Start the Visualizer
  void start();

  /// stop the Visualizer
  /**
   * Visualizer needs to be configured to start again.
   */
  void stop();

  /// Reset the Visualizer
  /**
   * Visualizer needs to be configured to start again.
   */
  void reset();

private:
  // timer callback.
  void _timer_cb();

  // Planning scene
  planning_scene::PlanningScenePtr scene_;

  // The rate the visualizer run.
  double rate_;

  // The trajectory display resolution.
  double step_;

  // The link to display the trajectory
  std::string tcp_link_;

  // ROS attributes
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;

  visualization_msgs::msg::Marker::SharedPtr marker_msg_;

  /// Common color to use.
  std_msgs::msg::ColorRGBA DARK_GREY;
  std_msgs::msg::ColorRGBA RED;
  std_msgs::msg::ColorRGBA ORANGE;
  std_msgs::msg::ColorRGBA YELLOW;
  std_msgs::msg::ColorRGBA LIGHT_GREEN;
  std_msgs::msg::ColorRGBA GREEN;

  // Current time point and collision time point.
  // TODO(anyone): Check if needed to be atomic.
  double current_time_point_;
  double collision_time_point_;

  std::atomic_bool start_;
  rclcpp::CallbackGroup::SharedPtr visualizer_callback_group_;

  // Safety zone
  SafetyZone safety_zone_;
};

}  // namespace dynamic_safety


#endif  // EMD__DYNAMIC_SAFETY__VISUALIZER_HPP_
