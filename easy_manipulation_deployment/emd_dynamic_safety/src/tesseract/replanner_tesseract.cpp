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
#include <unordered_map>
#include <vector>

#include "emd/dynamic_safety/replanner_tesseract.hpp"
#include "tesseract_kinematics/kdl/kdl_fwd_kin_chain.h"
#include "tesseract_kinematics/opw/opw_inv_kin.h"
#include "tesseract_environment/ofkt/ofkt_state_solver.h"
#include "tesseract_motion_planners/core/utils.h"
#include "tesseract_motion_planners/interface_utils.h"

// OMPL
#include "tesseract_motion_planners/ompl/ompl_motion_planner.h"
#include "tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h"
#include "tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h"

// Trajopt
#include "tesseract_motion_planners/trajopt/trajopt_motion_planner.h"
#include "tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h"
#include "tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h"
#include "tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h"
#include "tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h"

// TrajoptIFOPT
#include "tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_motion_planner.h"
#include "tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h"
#include "tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h"
#include "tesseract_motion_planners/trajopt_ifopt/problem_generators/default_problem_generator.h"

// Time parameterization
#include "tesseract_time_parameterization/iterative_spline_parameterization.h"
#include "tesseract_time_parameterization/time_optimal_trajectory_generation.h"
#include "tesseract_time_parameterization/instructions_trajectory.h"

#include "tesseract_rosutils/utils.h"

namespace dynamic_safety_tesseract
{

static const rclcpp::Logger LOGGER =
  rclcpp::get_logger("dynamic_safety_tesseract.tesseract_replanner");

TesseractReplannerContext::TesseractReplannerContext(
  const std::string & robot_urdf,
  const std::string & robot_srdf,
  const dynamic_safety::ReplannerOption & option,
  const rclcpp::Node::SharedPtr & node)
{
  env_ = std::make_shared<tesseract_environment::Environment>(true);
  tesseract_scene_graph::ResourceLocator::Ptr locator =
    std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  auto scene_graph = tesseract_urdf::parseURDFString(robot_urdf, locator);
  auto srdf = std::make_shared<tesseract_srdf::SRDFModel>();
  srdf->initString(*scene_graph, robot_srdf);
  env_->init<tesseract_environment::OFKTStateSolver>(*scene_graph, srdf);


  // Initialize kinematics manager
  // auto mm = env_->getManipulatorManager();
  // mm->init(scene_graph, srdf->kinematics_information);

  fwd_kin_ = env_->getManipulatorManager()->getFwdKinematicSolver(option.group);
  inv_kin_ = env_->getManipulatorManager()->getInvKinematicSolver(option.group);

  manip_.manipulator = option.group;
  // manip_.manipulator_ik_solver = "OPWInvKin";

  planner_ = option.planner;
  if (planner_.size() >= 4 &&
    planner_.compare(0, 4, "ompl") == 0)
  {
    auto profile =
      std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
    // We just need one solution
    profile->max_solutions = 1;
    profile->planning_time = option.deadline;

    // Hardcoded planner type
    // Only 1 thread is needed, retrying would be used.
    if (option.ompl_planner_id == "RRTConnect" ||
      option.ompl_planner_id.empty())
    {
      profile->planners = {
        std::make_shared<tesseract_planning::RRTConnectConfigurator>()
      };
    } else if (option.ompl_planner_id == "SBL") {
      profile->planners = {
        std::make_shared<tesseract_planning::SBLConfigurator>()
      };
    } else if (option.ompl_planner_id == "PRM") {
      profile->planners = {
        std::make_shared<tesseract_planning::PRMConfigurator>()
      };
    } else if (option.ompl_planner_id == "PRMstar") {
      profile->planners = {
        std::make_shared<tesseract_planning::PRMstarConfigurator>()
      };
    } else if (option.ompl_planner_id == "LazyPRM") {
      profile->planners = {
        std::make_shared<tesseract_planning::LazyPRMstarConfigurator>()
      };
    } else if (option.ompl_planner_id == "EST") {
      profile->planners = {
        std::make_shared<tesseract_planning::ESTConfigurator>()
      };
    } else if (option.ompl_planner_id == "BKPIECE1") {
      profile->planners = {
        std::make_shared<tesseract_planning::BKPIECE1Configurator>()
      };
    } else if (option.ompl_planner_id == "KPIECE1") {
      profile->planners = {
        std::make_shared<tesseract_planning::KPIECE1Configurator>()
      };
    }

    if (planner_ == "ompl") {
      profile->optimize = true;
    } else {
      // optimize using trajopt
      profile->optimize = false;
      profile->planning_time /= 4;
    }
    ompl_planner_.plan_profiles["DEFAULT"] = profile;
    ompl_planner_.problem_generator = &tesseract_planning::DefaultOMPLProblemGenerator;
  }

  if (planner_ == "trajopt" || planner_ == "ompl-trajopt") {
    auto plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
    auto composite_profile =
      std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();

    auto solver_profile =
      std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();

    // TODO(Briancbn): Parameterize the following
    plan_profile->joint_coeff = Eigen::VectorXd::Constant(fwd_kin_->numJoints(), 1, 10);

    // Use the fastest way to check collision
    composite_profile->contact_test_type = tesseract_collision::ContactTestType::FIRST;

    // Ensure continuity in vel and accel
    composite_profile->smooth_velocities = true;
    composite_profile->smooth_accelerations = true;
    composite_profile->smooth_jerks = true;
    composite_profile->avoid_singularity = true;

    composite_profile->collision_constraint_config.enabled = false;
    composite_profile->collision_cost_config.safety_margin = 0.005;
    composite_profile->collision_cost_config.coeff = 50;

    solver_profile->opt_info.max_time = option.deadline;
    if (planner_ != "trajopt") {
      solver_profile->opt_info.max_time *= 3.0 / 4.0;
    }

    trajopt_planner_.plan_profiles["DEFAULT"] = plan_profile;
    trajopt_planner_.composite_profiles["DEFAULT"] = composite_profile;
    trajopt_planner_.solver_profiles["DEFAULT"] = solver_profile;
    trajopt_planner_.problem_generator = &tesseract_planning::DefaultTrajoptProblemGenerator;
  }

  if (planner_ == "trajopt_ifopt" || planner_ == "ompl-trajopt_ifopt") {
    trajopt_ifopt_planner_.plan_profiles["DEFAULT"] =
      std::make_shared<tesseract_planning::TrajOptIfoptDefaultPlanProfile>();

    auto plan_profile =
      std::make_shared<tesseract_planning::TrajOptIfoptDefaultPlanProfile>();

    auto composite_profile =
      std::make_shared<tesseract_planning::TrajOptIfoptDefaultCompositeProfile>();

    // Add in trajopt collision evaluator, not added by default
    // TODO(Briancbn): parameterize margin here with moveit padding.
    double margin_coeff = 50;
    double margin = 0.005;
    auto collision_config =
      std::make_shared<trajopt_ifopt::TrajOptCollisionConfig>(margin, margin_coeff);
    collision_config->contact_request.type = tesseract_collision::ContactTestType::ALL;
    collision_config->type = tesseract_collision::CollisionEvaluatorType::DISCRETE;

    // additional margin buffer for the collision check
    collision_config->collision_margin_buffer = 0.001;
    composite_profile->longest_valid_segment_fraction = 0.001;
    composite_profile->longest_valid_segment_length = 0.001;
    composite_profile->collision_constraint_config = collision_config;
    // composite_profile->collision_cost_config = collision_config;

    // Ensure continuity in vel and accel
    composite_profile->smooth_velocities = true;

    // Not used atm
    // composite_profile->smooth_accelerations = true;
    // composite_profile->smooth_jerks = true;

    plan_profile->joint_coeff = Eigen::VectorXd::Constant(fwd_kin_->numJoints(), 1, 10);

    trajopt_ifopt_planner_.composite_profiles["DEFAULT"] =
      composite_profile;
    trajopt_ifopt_planner_.plan_profiles["DEFAULT"] = plan_profile;
    trajopt_ifopt_planner_.problem_generator =
      &tesseract_planning::DefaultTrajOptIfoptProblemGenerator;
  }

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

  for (auto & name : fwd_kin_->getJointNames()) {
    std::vector<rclcpp::Parameter> hvl_p;
    auto hvl_f = joint_limit_parameters_client->get_parameters(
      {option.joint_limits_parameter_namespace + "." +
        name + ".has_velocity_limits"});
    rclcpp::spin_until_future_complete(joint_limits_node, hvl_f);
    if (hvl_f.get()[0].as_bool()) {
      RCLCPP_INFO(LOGGER, "Found velocity limits");
      auto vl_f = joint_limit_parameters_client->get_parameters(
        {option.joint_limits_parameter_namespace + "." +
          name + ".max_velocity"});
      rclcpp::spin_until_future_complete(joint_limits_node, vl_f);
      joint_vel_limits_.push_back(vl_f.get()[0].as_double());
    } else {
      // TODO(Briancbn): Error Handling
    }
    std::vector<rclcpp::Parameter> hal_p;
    auto hal_f = joint_limit_parameters_client->get_parameters(
      {option.joint_limits_parameter_namespace + "." +
        name + ".has_acceleration_limits"});
    rclcpp::spin_until_future_complete(joint_limits_node, hal_f);
    if (hal_f.get()[0].as_bool()) {
      RCLCPP_INFO(LOGGER, "Found acceleration limits");
      auto al_f = joint_limit_parameters_client->get_parameters(
        {option.joint_limits_parameter_namespace + "." +
          name + ".max_acceleration"});
      rclcpp::spin_until_future_complete(joint_limits_node, al_f);
      joint_accel_limits_.push_back(al_f.get()[0].as_double());
    } else {
      // TODO(Briancbn): Error Handling
    }
  }
  time_parameterization_ = option.time_parameterization;
}

void TesseractReplannerContext::run(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & start_point,
  const trajectory_msgs::msg::JointTrajectoryPoint & end_point,
  trajectory_msgs::msg::JointTrajectory & plan)
{
  env_->setState(joint_names, start_point.positions);

  planning_request_.env = env_;
  planning_request_.env_state = env_->getCurrentState();

  // Specify start location
  Eigen::VectorXd start_point_eigen =
    Eigen::Map<const Eigen::VectorXd>(
    start_point.positions.data(),
    static_cast<int64_t>(start_point.positions.size()));
  RCLCPP_WARN_STREAM(LOGGER, "start_state: " << start_point_eigen);

  Eigen::VectorXd end_point_eigen =
    Eigen::Map<const Eigen::VectorXd>(
    end_point.positions.data(),
    static_cast<int64_t>(end_point.positions.size()));
  RCLCPP_WARN_STREAM(LOGGER, "end_state: " << end_point_eigen);
  tesseract_planning::StateWaypoint wp0(joint_names, start_point_eigen);

  tesseract_planning::StateWaypoint wp1(joint_names, end_point_eigen);

  // Define Plan Instructions
  tesseract_planning::PlanInstruction start_instruction(
    wp0, tesseract_planning::PlanInstructionType::START);

  // Using default profile
  tesseract_planning::PlanInstruction plan_f1(wp1,
    tesseract_planning::PlanInstructionType::FREESPACE, "DEFAULT");

  // Create program
  tesseract_planning::CompositeInstruction program;
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(manip_);
  program.push_back(plan_f1);

  // Create a seed
  planning_request_.seed =
    tesseract_planning::generateSeed(program, planning_request_.env_state, env_);

  planning_request_.instructions = program;

  tesseract_planning::PlannerResponse response;

  if (planner_.size() >= 4 &&
    planner_.compare(0, 4, "ompl") == 0)
  {
    auto ompl_status = ompl_planner_.solve(planning_request_, response);
    if (!ompl_status) {
      RCLCPP_ERROR(LOGGER, "OMPL planning failed:<");
      return;
    }
    RCLCPP_INFO(LOGGER, "OMPL planning successful!!");

    if (planner_ == "ompl") {
      plan = to_joint_trajectory_msg(response.results);
    } else if (planner_ == "ompl-trajopt") {
      RCLCPP_INFO(LOGGER, "Clean up using trajopt");
      // Using trajopt-ifopt to clean things up
      planning_request_.seed = response.results;
      auto trajopt_status =
        trajopt_planner_.solve(planning_request_, response);
      if (!trajopt_status) {
        RCLCPP_WARN(
          LOGGER, "Trajopt clean up failed. "
          "Using original trajectory");
        plan = to_joint_trajectory_msg(planning_request_.seed);
      } else {
        plan = to_joint_trajectory_msg(response.results);
      }
    } else if (planner_ == "ompl-trajopt_ifopt") {
      RCLCPP_INFO(LOGGER, "Clean up using trajopt ifopt");
      // Using trajopt-ifopt to clean things up
      planning_request_.seed = response.results;
      auto trajopt_ifopt_status =
        trajopt_ifopt_planner_.solve(planning_request_, response);
      if (!trajopt_ifopt_status) {
        RCLCPP_WARN(
          LOGGER, "Trajopt IFOPT clean up failed. "
          "Using original trajectory");
        plan = to_joint_trajectory_msg(planning_request_.seed);
      } else {
        plan = to_joint_trajectory_msg(response.results);
      }
    }
  } else if (planner_ == "trajopt") {
    auto trajopt_status = trajopt_planner_.solve(planning_request_, response);
    if (!trajopt_status) {
      RCLCPP_ERROR(LOGGER, "Trajopt planning failed:<");
    } else {
      RCLCPP_INFO(LOGGER, "Trajopt planning successful");
      plan = to_joint_trajectory_msg(response.results);
    }
  } else if (planner_ == "trajopt_ifopt") {
    auto trajopt_ifopt_status =
      trajopt_ifopt_planner_.solve(planning_request_, response);
    if (!trajopt_ifopt_status) {
      RCLCPP_ERROR(LOGGER, "Trajopt IFOPT planning failed:<");
    } else {
      RCLCPP_INFO(LOGGER, "Trajopt IFOPT planning successful");
      plan = to_joint_trajectory_msg(response.results);
    }
  }
}

void TesseractReplannerContext::update(
  const sensor_msgs::msg::JointState & joint_states)
{
  env_->setState(joint_states.name, joint_states.position);
}

bool TesseractReplannerContext::time_parameterize(
  trajectory_msgs::msg::JointTrajectory & plan,
  double scale)
{
  auto reorder_limits = [](
    const std::vector<std::string> & reference_joint_order,
    std::vector<std::string> & current_joint_order,
    std::vector<double> & jvl,
    std::vector<double> & jal) {
      std::unordered_map<std::string, size_t> ref_joint_idx_map;
      std::vector<size_t> joint_permutation;
      for (size_t i = 0; i < reference_joint_order.size(); i++) {
        ref_joint_idx_map[reference_joint_order[i]] = i;
      }
      // check the order that joint names is in
      for (auto & joint_name : current_joint_order) {
        joint_permutation.push_back(ref_joint_idx_map[joint_name]);
      }
      // Apply permutation order
      for (size_t i = 0; i < current_joint_order.size(); i++) {
        while (joint_permutation[i] != i) {
          std::swap(current_joint_order[joint_permutation[i]], current_joint_order[i]);
          std::swap(
            jvl[joint_permutation[i]],
            jvl[i]);
          std::swap(
            jal[joint_permutation[i]],
            jal[i]);
          std::swap(joint_permutation[joint_permutation[i]], joint_permutation[i]);
        }
      }
    };
  if (plan.points.empty()) {
    return false;
  }
  tesseract_planning::CompositeInstruction program;
  to_composite_instructions(plan, program);
  tesseract_planning::TrajectoryContainer::Ptr trajectory =
    std::make_shared<tesseract_planning::InstructionsTrajectory>(program);

  // re-order joints if necessary
  auto reordered_joint_vel_limits = joint_vel_limits_;
  auto reordered_joint_accel_limits = joint_vel_limits_;
  auto reordered_joint_names = fwd_kin_->getJointNames();
  if (reordered_joint_names != plan.joint_names) {
    reorder_limits(
      plan.joint_names, reordered_joint_names,
      joint_vel_limits_, joint_accel_limits_);
  }

  bool result = false;
  if (time_parameterization_ == "isp") {
    result = isp_.compute(*trajectory, joint_vel_limits_, joint_accel_limits_, scale);
  } else if (time_parameterization_ == "totg") {
    // TODO(Briancbn): scale is not working with totg
    result = totg_.computeTimeStamps(
      *trajectory,
      Eigen::Map<const Eigen::VectorXd>(
        joint_vel_limits_.data(),
        static_cast<int64_t>(joint_vel_limits_.size())),
      Eigen::Map<const Eigen::VectorXd>(
        joint_accel_limits_.data(),
        static_cast<int64_t>(joint_accel_limits_.size())),
      scale);
  }
  if (!result) {
    return false;
  }
  plan = to_joint_trajectory_msg(program);

  return true;
}

trajectory_msgs::msg::JointTrajectory TesseractReplannerContext::to_joint_trajectory_msg(
  const tesseract_planning::CompositeInstruction & result)
{
  auto reorder_joint = [](
    const std::vector<std::string> & reference_joint_order,
    std::vector<std::string> & current_joint_order,
    trajectory_msgs::msg::JointTrajectoryPoint & state) {
      std::unordered_map<std::string, size_t> ref_joint_idx_map;
      std::vector<size_t> joint_permutation;
      for (size_t i = 0; i < reference_joint_order.size(); i++) {
        ref_joint_idx_map[reference_joint_order[i]] = i;
      }
      // check the order that joint names is in
      for (auto & joint_name : current_joint_order) {
        joint_permutation.push_back(ref_joint_idx_map[joint_name]);
      }
      // Apply permutation order
      for (size_t i = 0; i < current_joint_order.size(); i++) {
        if (joint_permutation[i] != i) {
          std::swap(current_joint_order[joint_permutation[i]], current_joint_order[i]);
          if (!state.positions.empty()) {
            std::swap(
              state.positions[joint_permutation[i]],
              state.positions[i]);
          }
          if (!state.velocities.empty()) {
            std::swap(
              state.velocities[joint_permutation[i]],
              state.velocities[i]);
          }
          if (!state.accelerations.empty()) {
            std::swap(
              state.accelerations[joint_permutation[i]],
              state.accelerations[i]);
          }
          std::swap(joint_permutation[joint_permutation[i]], joint_permutation[i]);
        }
      }
    };

  auto to_vector = [](const Eigen::VectorXd & in, std::vector<double> & out) {
      if (in.size()) {
        out = std::vector<double>(&in[0], in.data() + in.cols() * in.rows());
      }
    };
  auto traj = tesseract_planning::toJointTrajectory(result);
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = traj.front().joint_names;
  for (size_t i = 0; i < traj.size(); i++) {
    // // Skip first
    // if (i == 0) {
    //   continue;
    // }
    auto joint_state = traj[i];
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(joint_state.time);
    to_vector(joint_state.position, point.positions);
    to_vector(joint_state.velocity, point.velocities);
    to_vector(joint_state.acceleration, point.accelerations);
    if (joint_state.joint_names != traj_msg.joint_names) {
      reorder_joint(traj_msg.joint_names, joint_state.joint_names, point);
    }
    traj_msg.points.push_back(point);
  }
  return traj_msg;
}

void TesseractReplannerContext::to_composite_instructions(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  tesseract_planning::CompositeInstruction & program)
{
  for (size_t i = 0; i < trajectory.points.size(); i++) {
    Eigen::VectorXd state =
      Eigen::Map<const Eigen::VectorXd>(
      trajectory.points[i].positions.data(),
      static_cast<int64_t>(trajectory.points[i].positions.size()));
    tesseract_planning::StateWaypoint swp(trajectory.joint_names, state);
    if (i == 0) {
      program.setStartInstruction(
        tesseract_planning::MoveInstruction(
          swp,
          tesseract_planning::MoveInstructionType::START));
    } else {
      program.push_back(
        tesseract_planning::MoveInstruction(
          swp,
          tesseract_planning::MoveInstructionType::FREESPACE));
    }
  }
}

}  // namespace dynamic_safety_tesseract
