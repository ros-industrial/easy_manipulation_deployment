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

#ifndef EMD__DYNAMIC_SAFETY__DYNAMIC_SAFETY_HPP_
#define EMD__DYNAMIC_SAFETY__DYNAMIC_SAFETY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "emd/utils.hpp"
#include "emd/dynamic_safety/safety_zone.hpp"
#include "emd/dynamic_safety/collision_checker.hpp"
// #include "emd/dynamic_safety/next_point_publisher.hpp"
#include "emd/dynamic_safety/replanner.hpp"
#include "emd/dynamic_safety/visualizer.hpp"
#include "emd/profiler.hpp"
#include "realtime_tools/realtime_buffer.h"

namespace dynamic_safety
{

struct Option
{
  double rate;

  bool dynamic_parameterization;

  bool use_description_server;

  std::string description_server;

  std::string joint_limits_parameter_server;
  std::string joint_limits_parameter_namespace;

  std::unordered_map<std::string, std::pair<double, double>> joint_limits;

  std::string robot_description;
  std::string robot_description_semantic;

  std::string environment_joint_states_topic;

  bool allow_replan;

  bool visualize;

  SafetyZone::Option safety_zone_options;

  CollisionCheckerOption collision_checker_options;

  // NextPointPublisher::Option next_point_publisher_options;

  ReplannerOption replanner_options;

  Visualizer::Option visualizer_options;

  const Option & load(const rclcpp::Node::SharedPtr & node);
};

class DynamicSafety
{
public:
  // cppcheck-suppress unknownMacro
  RCLCPP_SMART_PTR_DEFINITIONS(DynamicSafety)

  explicit DynamicSafety(
    rclcpp::Node::SharedPtr node)
  : DynamicSafety(Option().load(node))
  {
  }

  explicit DynamicSafety(const Option & option)
  : activated_(false)
  {
    option_ = option;

    // Reset Cache
    env_state_cache_.initRT(sensor_msgs::msg::JointState());
    current_state_cache_.initRT(CurrentState());
    current_time_cache_.initRT(0);
    scale_cache_.initRT(1);
  }

  ~DynamicSafety()
  {
    stop();
    delete pf_;
  }

  void configure(
    const rclcpp::Node::SharedPtr & node);

  void set_new_trajectory_callback(
    std::function<void(const trajectory_msgs::msg::JointTrajectory::SharedPtr &)> cb)
  {
    NewTrajectoryCB = cb;
  }

  void add_trajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt);

  void update_time(double current_time);
  void update_state(const sensor_msgs::msg::JointState::SharedPtr & state);

  void update_state(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_state);

  struct CurrentState
  {
    CurrentState() = default;

    CurrentState(
      const std::vector<std::string> & _joint_names,
      const trajectory_msgs::msg::JointTrajectoryPoint & _state)
    {
      joint_names = _joint_names;
      state = _state;
    }
    std::vector<std::string> joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint state;
  };

  void update_state(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr & state);

  double get_scale();

  void start();

  void wait();

  void stop();

private:
  void _deadline_cb(rclcpp::QOSDeadlineRequestedInfo &);

  void _main_loop();

  double _cal_scale_time(
    const CurrentState & current_state,
    double current_scale,
    double target_scale);

  void _handle_replanner(double start_state_time);

  // Temporary functions to be moved into collision checker
  double _back_track_last_collision();
  double full_duration_;
  std::function<void(const trajectory_msgs::msg::JointTrajectory::SharedPtr &)> NewTrajectoryCB;

  Option option_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr env_state_sub_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::CallbackGroup::SharedPtr env_state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr main_callback_group_;

  CollisionChecker collision_checker_;
  SafetyZone safety_zone_;
  // NextPointPublisher next_point_publisher_;
  Replanner replanner_;
  Visualizer visualizer_;

  double collision_time_point_;
  // double replan_time_point_;

  // uint8_t zone;

  std::atomic_bool activated_;
  std::atomic_bool started;

  std::vector<double> benchmark_stats;

  std::promise<void> sig_;
  std::future<void> future_;

  emd::TimeProfiler<> * pf_;

  // realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectoryPoint> state_cache_;
  realtime_tools::RealtimeBuffer<sensor_msgs::msg::JointState> env_state_cache_;
  realtime_tools::RealtimeBuffer<CurrentState> current_state_cache_;
  realtime_tools::RealtimeBuffer<double> current_time_cache_;
  realtime_tools::RealtimeBuffer<double> scale_cache_;
  // realtime_tools::RealtimeBuffer<octomap::OcTree> env_state_cache_;
};

}  // namespace dynamic_safety


#endif  // EMD__DYNAMIC_SAFETY__DYNAMIC_SAFETY_HPP_
