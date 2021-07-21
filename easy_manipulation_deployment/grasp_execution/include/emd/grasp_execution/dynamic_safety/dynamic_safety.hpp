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

#ifndef EMD__GRASP_EXECUTION__DYNAMIC_SAFETY__DYNAMIC_SAFETY_HPP_
#define EMD__GRASP_EXECUTION__DYNAMIC_SAFETY__DYNAMIC_SAFETY_HPP_

#include <memory>
#include <vector>

#include "emd/grasp_execution/utils.hpp"
#include "emd/grasp_execution/dynamic_safety/safety_zone.hpp"
#include "emd/grasp_execution/dynamic_safety/collision_checker.hpp"
#include "emd/grasp_execution/dynamic_safety/next_point_publisher.hpp"
#include "emd/grasp_execution/dynamic_safety/replanner.hpp"
#include "emd/grasp_execution/dynamic_safety/visualizer.hpp"
#include "emd/grasp_execution/core/profiler.hpp"

namespace grasp_execution
{

namespace dynamic_safety
{

struct Option
{
  double rate;

  bool allow_replan;

  bool visualize;

  SafetyZone::Option safety_zone_options;

  CollisionChecker::Option collision_checker_options;

  NextPointPublisher::Option next_point_publisher_options;

  Replanner::Option replanner_options;

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
  {
    option_ = option;
  }

  ~DynamicSafety()
  {
    delete pf_;
  }

  void configure(
    const planning_scene::PlanningScenePtr & scene,
    const robot_trajectory::RobotTrajectoryPtr & rt,
    const rclcpp::Node::SharedPtr & node);

  void start();

  void wait();

  void stop();

private:
  void _deadline_cb(rclcpp::QOSDeadlineRequestedInfo &);

  void _main_loop(const sensor_msgs::msg::JointState::SharedPtr & joint_state_msg);

  Option option_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  moveit::core::RobotStatePtr current_state_;

  CollisionChecker collision_checker_;
  SafetyZone safety_zone_;
  NextPointPublisher next_point_publisher_;
  Replanner replanner_;
  Visualizer visualizer_;

  double collision_time_point_;
  double replan_time_point_;

  uint8_t zone;

  std::atomic_bool started;

  std::vector<double> benchmark_stats;

  std::promise<void> sig_;
  std::future<void> future_;

  core::TimeProfiler<> * pf_;
};

}  // namespace dynamic_safety

}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__DYNAMIC_SAFETY__DYNAMIC_SAFETY_HPP_
