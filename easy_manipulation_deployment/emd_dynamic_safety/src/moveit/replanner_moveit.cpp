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
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "emd/dynamic_safety/replanner_moveit.hpp"
#include "moveit/robot_state/conversions.h"
#include "pluginlib/class_loader.hpp"

namespace dynamic_safety_moveit
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dynamic_safety_moveit.replanner");

MoveitReplannerContext::MoveitReplannerContext(
  const std::string & robot_urdf,
  const std::string & robot_srdf,
  const dynamic_safety::ReplannerOption & option,
  const rclcpp::Node::SharedPtr & node)
{
  urdf::ModelSharedPtr umodel = std::make_shared<urdf::Model>();
  srdf::ModelSharedPtr smodel = std::make_shared<srdf::Model>();

  if (umodel->initString(robot_urdf)) {
    if (!smodel->initString(*umodel, robot_srdf)) {
      RCLCPP_ERROR(LOGGER, "Unable to parse SRDF");
      // TODO(anyone): exception handling
    }
  } else {
    RCLCPP_ERROR(LOGGER, "Unable to parse URDF");
    // TODO(anyone): exception handling
  }

  // Construct robot model
  auto rm = std::make_shared<moveit::core::RobotModel>(umodel, smodel);

  // Joint Limit loader
  // new node for parameter loading
  auto joint_limits_node = std::make_shared<rclcpp::Node>(
    std::string(
      node->get_name()) + "_joint_limits_loader");

  auto joint_limit_parameters_client =
    std::make_shared<rclcpp::AsyncParametersClient>(
    joint_limits_node, option.joint_limits_parameter_server);
  while (!joint_limit_parameters_client->wait_for_service()) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        LOGGER, "Interrupted while waiting for %s service. Exiting.",
        option.joint_limits_parameter_server.c_str());
      // TODO(anyone): exception handling.
      break;
    }
    RCLCPP_ERROR(
      LOGGER, "%s service not available, waiting again...",
      option.joint_limits_parameter_server.c_str());
  }
  RCLCPP_INFO(
    LOGGER, "Connected to description server %s!!",
    option.joint_limits_parameter_server.c_str());

  auto joint_group = rm->getJointModelGroup(option.group);

  for (auto & name : joint_group->getVariableNames()) {
    auto joint = rm->getJointModel(name);
    moveit::core::VariableBounds bound;
    auto urdf_joint = umodel->getJoint(joint->getName());
    bound = joint->getVariableBounds(joint->getName());
    std::vector<rclcpp::Parameter> hal_p;
    auto hal_f = joint_limit_parameters_client->get_parameters(
      {option.joint_limits_parameter_namespace + "." +
        joint->getName() + ".has_acceleration_limits"});
    rclcpp::spin_until_future_complete(joint_limits_node, hal_f);
    if (hal_f.get()[0].as_bool()) {
      RCLCPP_INFO(LOGGER, "Found acceleration limits");
      auto al_f = joint_limit_parameters_client->get_parameters(
        {option.joint_limits_parameter_namespace + "." +
          joint->getName() + ".max_acceleration"});
      rclcpp::spin_until_future_complete(joint_limits_node, al_f);
      bound.acceleration_bounded_ = true;
      bound.max_acceleration_ = fabs(al_f.get()[0].as_double());
      bound.min_acceleration_ = -bound.max_acceleration_;
      joint->setVariableBounds(joint->getName(), bound);
    }
  }

  scene_ = std::make_shared<planning_scene::PlanningScene>(rm);

  // Construct a planning instance
  //
  // load the planning plugin
  auto planner_plugin_loader =
    std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
    "moveit_core", "planning_interface::PlannerManager");
  auto to_all_lower = [](std::string in) -> std::string {
      std::transform(in.begin(), in.end(), in.begin(), ::tolower);
      return in;
    };

  std::string planner_name = to_all_lower(option.planner);
  std::string plugin_name("");
  if (planner_name == "ompl") {
    plugin_name = "ompl_interface/OMPLPlanner";
    planning_request_.planner_id = option.ompl_planner_id;
  }
  // TODO(anyone): Add in other planning methods.

  planning_manager_ = planner_plugin_loader->createUniqueInstance(plugin_name);

  // new node for planning config parameter loading
  auto planner_config_loader_node = std::make_shared<rclcpp::Node>(
    std::string(
      node->get_name()) + "_planner_config_loader",
    rclcpp::NodeOptions().allow_undeclared_parameters(true));

  // Load self parameters
  if (option.planner_parameter_server == node->get_name()) {
    auto planner_config =
      node->list_parameters({option.group, option.planner_parameter_namespace}, 5);
    if (!planner_config.names.empty()) {
      std::string result = "Parameters found:\n";
      for (auto & name : planner_config.names) {
        result += "\t" + name + "\n";
      }
      RCLCPP_WARN(LOGGER, "%s", result.c_str());
      auto planner_config_params = node->get_parameters(planner_config.names);
      planner_config_loader_node->set_parameters(planner_config_params);
    } else {
      RCLCPP_ERROR(
        LOGGER, "no planner configs defined");
    }
  } else {
    // Getting parameters from external node
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      planner_config_loader_node, option.planner_parameter_server);
    while (!parameters_client->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          LOGGER, "Interrupted while waiting for %s service. Exiting.",
          option.planner_parameter_server.c_str());
        // TODO(anyone): exception handling.
        break;
      }
      RCLCPP_ERROR(
        LOGGER, "%s service not available, waiting again...",
        option.planner_parameter_server.c_str());
    }
    RCLCPP_INFO(
      LOGGER, "Connected to description server %s!!",
      option.planner_parameter_server.c_str());
    rcl_interfaces::msg::ListParametersResult planner_config;
    while (planner_config.names.empty()) {
      try {
        RCLCPP_INFO(LOGGER, "Get parameters");
        auto planner_config_future = parameters_client->list_parameters(
          {option.group, option.planner_parameter_namespace},
          5);
        rclcpp::spin_until_future_complete(
          planner_config_loader_node, planner_config_future);
        planner_config = planner_config_future.get();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(LOGGER, "%s", e.what());
      }
      if (!planner_config.names.empty()) {
        std::string result = "Parameters found:\n";
        for (auto & name : planner_config.names) {
          result += "\t" + name + "\n";
        }
        RCLCPP_WARN(LOGGER, "%s", result.c_str());
        break;
      } else {
        RCLCPP_ERROR(
          LOGGER, "dynamic_safety is waiting for planner_configs");
      }
      usleep(100000);
    }

    auto f = parameters_client->get_parameters(planner_config.names);
    rclcpp::spin_until_future_complete(planner_config_loader_node, f);
    planner_config_loader_node->set_parameters(f.get());
  }

  if (!planning_manager_->initialize(
      scene_->getRobotModel(),
      planner_config_loader_node, option.planner_parameter_namespace))
  {
    throw std::runtime_error("Unable to initialize planning plugin");
  }

  planning_request_.group_name = option.group;
  planning_request_.allowed_planning_time = option.deadline;
  planning_request_.num_planning_attempts = 1;
  planning_request_.allowed_planning_time = 0.3;
  planning_request_.max_velocity_scaling_factor = 1;
  planning_request_.max_acceleration_scaling_factor = 1;

  time_parameterization_ = option.time_parameterization;
}

void MoveitReplannerContext::run(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & start_point,
  const trajectory_msgs::msg::JointTrajectoryPoint & end_point,
  trajectory_msgs::msg::JointTrajectory & plan)
{
  // TODO(anyone): This is still a bit unsafe, not thread safe
  // Nothing I can think of to solve this right now
  // Lock scene
  std::lock_guard<std::mutex> lk(scene_mtx_);
  auto & start_state = scene_->getCurrentStateNonConst();
  moveit::core::RobotState end_state = scene_->getCurrentState();

  // Update state
  for (size_t i = 0; i < joint_names.size(); i++) {
    // TODO(anyone): multi-axis joint
    start_state.setJointPositions(joint_names[i], {start_point.positions[i]});
    end_state.setJointPositions(joint_names[i], {end_point.positions[i]});
  }
  moveit::core::robotStateToRobotStateMsg(start_state, planning_request_.start_state);
  planning_request_.goal_constraints = {
    kinematic_constraints::constructGoalConstraints(
      end_state,
      scene_->getRobotModel()->getJointModelGroup(
        planning_request_.group_name))
  };

  planning_interface::MotionPlanResponse planning_response;

  auto context =
    planning_manager_->getPlanningContext(
    scene_, planning_request_, planning_response.error_code_);
  if (context) {
    context->solve(planning_response);
    // This will make it much less likely to timeout
    if (planning_response.error_code_.val == planning_response.error_code_.SUCCESS) {
      moveit_msgs::msg::RobotTrajectory moveit_traj_msg;
      planning_response.trajectory_->getRobotTrajectoryMsg(moveit_traj_msg);
      plan = std::move(moveit_traj_msg.joint_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
    }
  } else {
    RCLCPP_ERROR(LOGGER, "No planner found");
  }
}

void MoveitReplannerContext::update(
  const sensor_msgs::msg::JointState & joint_states)
{
  if (scene_mtx_.try_lock()) {
    auto & current_state = scene_->getCurrentStateNonConst();
    // Update state
    for (size_t i = 0; i < joint_states.name.size(); i++) {
      // TODO(anyone): multi-axis joint
      if (current_state.getJointModel(joint_states.name[i])) {
        current_state.setJointPositions(joint_states.name[i], {joint_states.position[i]});
      }
    }
    scene_mtx_.unlock();
  } else {
    RCLCPP_WARN(LOGGER, "Planning ongoing scene is not updated");
  }
}

void MoveitReplannerContext::update(
  const moveit_msgs::msg::PlanningScene & scene)
{
  if (scene_mtx_.try_lock()) {
    scene_->processPlanningSceneWorldMsg(scene.world);
    scene_mtx_.unlock();
  } else {
    RCLCPP_WARN(LOGGER, "Planning ongoing scene is not updated");
  }
}

bool MoveitReplannerContext::time_parameterize(
  trajectory_msgs::msg::JointTrajectory & plan,
  double scale)
{
  robot_trajectory::RobotTrajectory rt(scene_->getRobotModel(), planning_request_.group_name);
  moveit_msgs::msg::RobotTrajectory rt_msg;
  rt_msg.joint_trajectory = plan;
  auto state = scene_->getCurrentState();
  rt.setRobotTrajectoryMsg(scene_->getCurrentState(), rt_msg);
  if (!_time_parameterization(rt, scale)) {
    return false;
  }
  rt.getRobotTrajectoryMsg(rt_msg);
  plan = rt_msg.joint_trajectory;
  return true;
}

bool MoveitReplannerContext::_time_parameterization(
  robot_trajectory::RobotTrajectory & trajectory, double scale)
{
  if (time_parameterization_ == "totg") {
    return totg_.computeTimeStamps(trajectory, scale);
  } else if (time_parameterization_ == "iptp") {
    return iptp_.computeTimeStamps(trajectory, scale);
  } else if (time_parameterization_ == "isp") {
    return isp_.computeTimeStamps(trajectory, scale);
  }
  return false;
}

}  // namespace dynamic_safety_moveit
