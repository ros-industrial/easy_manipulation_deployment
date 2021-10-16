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
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "emd/dynamic_safety/collision_checker_tesseract.hpp"
#include "tesseract_environment/kdl/kdl_state_solver.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"
#include "tesseract_rosutils/utils.h"

namespace dynamic_safety_tesseract
{

static const rclcpp::Logger LOGGER =
  rclcpp::get_logger("dynamic_safety_tesseract.collision_checker");
static const rcutils_duration_value_t LOG_RATE = 5e8;

TesseractCollisionCheckerContext::TesseractCollisionCheckerContext(
  const std::string & robot_urdf,
  const std::string & robot_srdf,
  const std::string & collision_checking_plugin)
: dynamic_safety::CollisionCheckerContext(robot_urdf, robot_srdf, collision_checking_plugin)
{
  // Load default collisiong manager:
  // * Bullet Discrete BVH Manager - default
  // * FCL Discrete BVH Manager
  // * Bullet Cast BVH Manager - default
  if (!env) {
    env = std::make_shared<tesseract_environment::Environment>(true);
    tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    env->init<tesseract_environment::KDLStateSolver>(robot_urdf, robot_srdf, locator);
    // Load collision_checking_plugin
    // TODO(anyone): use the mapping file
    // auto loader = pluginlib::ClassLoader<collision_detection::CollisionPlugin>(
    //     "moveit_core", "collision_detection::CollisionPlugin");
    auto to_all_lower = [](std::string in) -> std::string {
        std::transform(in.begin(), in.end(), in.begin(), ::tolower);
        return in;
      };

    std::string plugin_name;
    // TODO(anyone): use plugin loader after release of fix
    //               https://github.com/ros-planning/moveit2/pull/658
    if (to_all_lower(collision_checking_plugin) == "fcl") {
      env->setActiveDiscreteContactManager(
        tesseract_collision::tesseract_collision_fcl::FCLDiscreteBVHManager::name());
    } else if (to_all_lower(collision_checking_plugin) == "bullet") {
      // Default is bullet
    }
  }
  instances++;

  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
}

void TesseractCollisionCheckerContext::configure(
  const dynamic_safety::CollisionCheckerOption & option)
{
  collision_check_config_.longest_valid_segment_length = 0.1;  // Configurable?
  collision_check_config_.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  collision_check_config_.contact_request.calculate_penetration = false;
  collision_check_config_.contact_request.calculate_distance = option.distance;
  collision_check_config_.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  tesseract_collision::CollisionMarginData margin_data(0.0);
  collision_check_config_.collision_margin_data = margin_data;
  if (!option.continuous) {
    // Clone shared object
    discrete_manager_ = env->getDiscreteContactManager();
  } else {
    continuous_manager_ = env->getContinuousContactManager();
  }
  state_solver_ = env->getStateSolver();
}

void TesseractCollisionCheckerContext::run_discrete(
  std::vector<std::string> joint_names,
  trajectory_msgs::msg::JointTrajectoryPoint point,
  uint8_t & result, double & distance)
{
  state_solver_->setState(joint_names, point.positions);
  discrete_manager_->setCollisionObjectsTransform(
    state_solver_->getCurrentState()->link_transforms);
  result = false;
  collision_result_.clear();

  auto active_links = discrete_manager_->getActiveCollisionObjects();
  discrete_manager_->contactTest(collision_result_, collision_check_config_.contact_request);
  result = static_cast<uint8_t>(!collision_result_.empty());
  double tmp_distance = std::numeric_limits<double>::max();
  for (auto & collision : collision_result_) {
    RCLCPP_WARN_THROTTLE(
      LOGGER, *clock_, LOG_RATE, "collision pair: <%s, %s>",
      collision.first.first.c_str(),
      collision.first.second.c_str());

    for (auto & result : collision.second) {
      RCLCPP_INFO_THROTTLE(
        LOGGER, *clock_, LOG_RATE, "distance: %s",
        result.distance);
      if (result.distance < tmp_distance) {
        tmp_distance = result.distance;
      }
    }
  }

  distance = tmp_distance;
}

void TesseractCollisionCheckerContext::run_continuous(
  std::vector<std::string> joint_names,
  trajectory_msgs::msg::JointTrajectoryPoint point1,
  trajectory_msgs::msg::JointTrajectoryPoint point2,
  uint8_t & result, double & distance)
{
  // TODO(anyone): This is not checking self collision
  state_solver_->setState(joint_names, point1.positions);
  tesseract_environment::EnvState::Ptr state1 =
    std::make_shared<tesseract_environment::EnvState>(
    *(state_solver_->getCurrentState()));

  state_solver_->setState(joint_names, point2.positions);
  tesseract_environment::EnvState::Ptr state2 =
    std::make_shared<tesseract_environment::EnvState>(
    *(state_solver_->getCurrentState()));

  auto & pose1 = state1->link_transforms;
  auto & pose2 = state2->link_transforms;
  assert(pose1.size() == pose2.size());
  auto it1 = pose1.begin();
  auto it2 = pose2.begin();
  std::string link_names = "";
  while (it1 != pose1.end()) {
    assert(pose1.find(it1->first) != pose2.end());
    continuous_manager_->setActiveCollisionObjects({it1->first});
    link_names += "\t" + it1->first + "\n";
    continuous_manager_->setCollisionObjectsTransform(
      it1->first,
      it1->second, it2->second);
    std::advance(it1, 1);
    std::advance(it2, 1);
  }
  printf("Active Links: \n%s", link_names.c_str());

  continuous_manager_->contactTest(collision_result_, collision_check_config_.contact_request);
  result = static_cast<uint8_t>(!collision_result_.empty());
  double tmp_distance = std::numeric_limits<double>::max();
  for (auto & collision : collision_result_) {
    RCLCPP_WARN(
      LOGGER, "collision pair: <%s, %s>",
      collision.first.first.c_str(),
      collision.first.second.c_str());

    for (auto & result : collision.second) {
      RCLCPP_INFO(
        LOGGER, "distance: %s",
        result.distance);
      if (result.distance < tmp_distance) {
        tmp_distance = result.distance;
      }
    }
  }

  distance = tmp_distance;
}

void TesseractCollisionCheckerContext::update(
  const sensor_msgs::msg::JointState & joint_states)
{
  state_solver_->setState(joint_states.name, joint_states.position);
  if (discrete_manager_) {
    discrete_manager_->setCollisionObjectsTransform(
      state_solver_->getCurrentState()->link_transforms);
  }
  if (continuous_manager_) {
    continuous_manager_->setCollisionObjectsTransform(
      state_solver_->getCurrentState()->link_transforms);
  }
}

}  // namespace dynamic_safety_tesseract
