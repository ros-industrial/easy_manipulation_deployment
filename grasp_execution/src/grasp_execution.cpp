// Copyright 2020 ROS Industrial Consortium Asia Pacific
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

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "grasp_execution/grasp_execution.hpp"
#include "grasp_execution/utils.hpp"

#include "rclcpp/rclcpp.hpp"

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"
#include "moveit/robot_state/cartesian_interpolator.h"
#include "moveit/macros/console_colors.h"
#include "moveit/utils/message_checks.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"

#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/impl/utils.h"
#include "tf2_eigen/tf2_eigen.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "grasp_planning/msg/grasp_pose.hpp"

namespace grasp_execution
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("grasp_execution");

static const char PLANNING_GROUP[] = "manipulator";

static const char CAMERA_FRAME[] = "camera_frame";

static const char SEPARATOR[] =
  "------------------------------------------------"
  "------------------------------------------------";

static const float CLEARANCE = 0.1;

GraspExecution::GraspExecution(const rclcpp::Node::SharedPtr & node)
: node_(node),
  moveit_cpp_(std::make_shared<moveit::planning_interface::MoveItCpp>(node_)),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
  tf_listener_(*tf_buffer_),
  ignore_links_{
    "gripper_finger1_finger_link",
    "gripper_finger1_finger_tip_link",
    "gripper_finger1_inner_knuckle_link",
    "gripper_finger1_knuckle_link",
    "gripper_finger2_finger_link",
    "gripper_finger2_finger_tip_link",
    "gripper_finger2_inner_knuckle_link",
    "gripper_finger2_knuckle_link",
    "gripper_base_link",
    "ee_palm"
  }
{
  // let RViz display query PlanningScene
  moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();
  moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

  grasp_planning_sub_ = node_->create_subscription<grasp_planning::msg::GraspPose>(
    "grasp_poses", 10,
    [ = ](grasp_planning::msg::GraspPose::UniquePtr msg) {
      // Check if there are on going tasks
      if (execution_thread_ptr_) {
        // Check if the ongoing tasks is finished
        auto status = execution_thread_future_.wait_for(std::chrono::milliseconds(0));
        if (status == std::future_status::ready) {
          execution_thread_ptr_->join();
          execution_thread_ptr_.reset();
        } else {
          RCLCPP_INFO(node_->get_logger(), "Task is still ongoing");
          return;
        }
      }

      RCLCPP_INFO(node_->get_logger(), "Start new execution thread");
      auto sig = std::promise<bool>();
      execution_thread_future_ = sig.get_future();
      execution_thread_ptr_ = std::make_shared<std::thread>(
        &GraspExecution::workflow, this, std::move(msg), std::move(sig));
    }
  );
}

GraspExecution::~GraspExecution()
{
  if (execution_thread_ptr_) {
    execution_thread_ptr_->join();
  }

  // Exit everything in order
  moveit_cpp_.reset();
  for (auto & arm : arms_) {
    arm.second.reset();
  }
}

bool GraspExecution::init(const std::string & planning_group, const std::string & _ee_link)
{
  // Check if planner is already registered
  if (arms_.find(planning_group) == arms_.end()) {
    RCLCPP_INFO(
      LOGGER,
      MOVEIT_CONSOLE_COLOR_CYAN
      "Initializing planning group: [%s]."
      MOVEIT_CONSOLE_COLOR_RESET,
      planning_group.c_str());

    // Print out basic planning group info for debugging
    moveit::core::RobotStatePtr state;
    moveit_cpp_->getCurrentState(state, 0);
    auto robot_model = state->getRobotModel();
    auto joint_model_group = robot_model->getJointModelGroup(planning_group);
    size_t dof = joint_model_group->getVariableCount();
    auto link_names = joint_model_group->getLinkModelNames();
    auto root_link = robot_model->getRootLinkName();

    if (dof < 1) {
      RCLCPP_ERROR(LOGGER, "Planning Group [%s] is empty", planning_group.c_str());
      return false;
    } else {
      RCLCPP_INFO(
        LOGGER,
        "Basic Info:\n"
        "Base frame: %s\n"
        "Joint Model Group: %s\n"
        "\tDoF: %u\n"
        "\tfirst_link: %s\n"
        "\tend_link:  %s\n"
        "\tChain: %s\n"
        "\tEnd Effector: %s\n",
        root_link.c_str(), planning_group.c_str(), dof,
        link_names.front().c_str(), link_names.back().c_str(),
        (joint_model_group->isChain() ? "Yes" : "No"),
        (joint_model_group->isEndEffector() ? "Yes" : "No"));

      // Initializing planner
      arms_[planning_group] = std::make_shared<PlannerT>(planning_group, moveit_cpp_);

      // Set robot base frame and end effector frame
      robot_frame_ = root_link;

      ee_link_ = (_ee_link.empty() ? link_names.back() : _ee_link);
      if (_ee_link != link_names.back()) {
        RCLCPP_WARN(
          LOGGER,
          MOVEIT_CONSOLE_COLOR_YELLOW
          "Assuming [%s] is rigidly attached to end_link [%s]"
          MOVEIT_CONSOLE_COLOR_RESET, ee_link_.c_str(), link_names.back().c_str());
      }

      // Set end effector frame to the child link of ee_link
      auto ee_joint_models = robot_model->getLinkModel(ee_link_)->getChildJointModels();
      if (ee_joint_models.empty()) {
        RCLCPP_WARN(
          LOGGER,
          MOVEIT_CONSOLE_COLOR_YELLOW
          "No valid child frame for [%s], use itself as attach frame for object"
          MOVEIT_CONSOLE_COLOR_RESET, ee_link_.c_str());
        ee_frame_ = ee_link_;
      } else {
        RCLCPP_INFO(LOGGER, "End effectors attached to [%s]:", ee_link_.c_str());
        for (auto & ee_joint_model : ee_joint_models) {
          RCLCPP_INFO(
            LOGGER, "- %s",
            ee_joint_model->getChildLinkModel()->getName().c_str());
        }
        // TODO(Briancbn): multiple end effector
        ee_frame_ = ee_joint_models.front()->getChildLinkModel()->getName();
      }
      return true;
    }

  } else {
    return false;
  }
}

void GraspExecution::workflow(
  grasp_planning::msg::GraspPose::UniquePtr msg,
  std::promise<bool> && _sig)
{
  // Get home state
  moveit::core::RobotStatePtr home_state;
  moveit_cpp_->getCurrentState(home_state, 0);

  // To-be-removed: get home pose
  auto home_pose = get_curr_pose(ee_link_);

  // ------------------- Prepare object for grasping --------------------------
  RCLCPP_INFO(
    LOGGER,
    MOVEIT_CONSOLE_COLOR_CYAN
    "Adding objects to scene"
    MOVEIT_CONSOLE_COLOR_RESET);

  std::vector<moveit_msgs::msg::CollisionObject> grasp_objects;

  for (size_t i = 0; i < msg->num_objects; i++) {
    moveit_msgs::msg::CollisionObject temp_collision_object;
    temp_collision_object.header.frame_id =
      (msg->object_poses[i].header.frame_id.empty() ? CAMERA_FRAME :
      msg->object_poses[i].header.frame_id);
    // Use random UUID unless specified object name
    // TODO(Briancbn): Use primitive shapes as part of id header
    temp_collision_object.id = "#object-" + gen_uuid();
    temp_collision_object.primitives.push_back(msg->object_shapes[i]);
    temp_collision_object.primitive_poses.push_back(msg->object_poses[i].pose);

    // Print out all object poses as debug information
    std::ostringstream oss;
    print_pose(msg->object_poses[i], oss);
    RCLCPP_INFO(LOGGER, oss.str());

    auto tmp_pose = msg->object_poses[i];
    to_frame(msg->object_poses[i], tmp_pose, robot_frame_, *tf_buffer_);
    std::ostringstream oss2;
    print_pose(tmp_pose, oss2);
    RCLCPP_INFO(LOGGER, oss2.str());


    temp_collision_object.operation = temp_collision_object.ADD;
    grasp_objects.push_back(temp_collision_object);

    // Add object to planning scene
    {    // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(temp_collision_object);
    }    // Unlock PlanningScene
  }

  // -------------------------------------------------------------

  // ------------------- Executing grasp poses one by one --------------------------
  for (size_t i = 0; i < msg->grasp_poses.size(); i++) {
    auto & grasp_pose = msg->grasp_poses[i];

    // Frame ID default will be the Camera ID if not set
    if (grasp_pose.header.frame_id.empty()) {
      grasp_pose.header.frame_id = CAMERA_FRAME;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    to_frame(grasp_pose, target_pose, robot_frame_, *tf_buffer_);

    auto temp_target_pose = target_pose;

    // ------------------- Move to pre grasp location --------------------------
    RCLCPP_INFO(
      LOGGER,
      MOVEIT_CONSOLE_COLOR_CYAN
      "Moving to pre-grasp location for [%s]."
      MOVEIT_CONSOLE_COLOR_RESET,
      grasp_objects[i].id.c_str());

    // Initial approach doesn't move down yet
    temp_target_pose.pose.position.z = target_pose.pose.position.z + CLEARANCE;

    // Print out target pose for debug purpose
    std::ostringstream oss;
    print_pose(temp_target_pose, oss);
    RCLCPP_INFO(LOGGER, oss.str());

    // Robot is above the object
    move_to(temp_target_pose, ee_link_);

    // ------------------- Approaching grasp location --------------------------
    RCLCPP_INFO(
      LOGGER,
      MOVEIT_CONSOLE_COLOR_CYAN
      "Approaching grasp location for [%s]. "
      MOVEIT_CONSOLE_COLOR_RESET,
      grasp_objects[i].id.c_str());
    // Move down to pick
    move_until_before_collide(temp_target_pose, ee_link_, -0.002, 40, 'z');

    // TODO(Briancbn): Call to gripper driver

    // ------------------- Attach grasp object to robot --------------------------
    RCLCPP_INFO(
      LOGGER,
      MOVEIT_CONSOLE_COLOR_CYAN
      "Attaching [%s] to robot ee frame: [%s]."
      MOVEIT_CONSOLE_COLOR_RESET,
      grasp_objects[i].id.c_str(), ee_frame_.c_str());

    attach_object_to_ee(grasp_objects[i]);

    // TODO(Briancbn): Update pre-pick location
    // ------------------- Cartesian move to pose grasp location --------------------------
    RCLCPP_INFO(
      LOGGER,
      MOVEIT_CONSOLE_COLOR_CYAN
      "Cartesian to post grasp location for [%s]."
      MOVEIT_CONSOLE_COLOR_RESET,
      grasp_objects[i].id.c_str());

    temp_target_pose.pose.position.z = target_pose.pose.position.z + CLEARANCE;
    cartesian_to(
      std::vector<geometry_msgs::msg::Pose>{temp_target_pose.pose},
      ee_link_, 0.01);

    // ------------------- Cartesian move to release location --------------------------
    // TODO(Briancbn): Configurable release pose
    geometry_msgs::msg::PoseStamped release_pose = home_pose;
    release_pose.pose.orientation = target_pose.pose.orientation;

    RCLCPP_INFO(
      LOGGER,
      MOVEIT_CONSOLE_COLOR_CYAN
      "Cartesian to pre-release location for [%s]."
      MOVEIT_CONSOLE_COLOR_RESET,
      grasp_objects[i].id.c_str());

    release_pose.pose.position.x -= 0.3;
    release_pose.pose.position.z = target_pose.pose.position.z + CLEARANCE;
    cartesian_to(
      std::vector<geometry_msgs::msg::Pose>{release_pose.pose},
      ee_link_, 0.01);

    // ------------------- Approaching release location --------------------------
    RCLCPP_INFO(
      LOGGER,
      MOVEIT_CONSOLE_COLOR_CYAN
      "Approaching release location for [%s]."
      MOVEIT_CONSOLE_COLOR_RESET,
      grasp_objects[i].id.c_str());
    // move to release
    move_until_before_collide(release_pose, ee_link_, -0.01, 20, 'z');

    // TODO(Briancbn): Call to gripper driver

    // ------------------- detach grasp object from robot --------------------------
    RCLCPP_INFO(
      LOGGER,
      MOVEIT_CONSOLE_COLOR_CYAN
      "Detaching [%s] from robot ee frame: [%s]."
      MOVEIT_CONSOLE_COLOR_RESET,
      grasp_objects[i].id.c_str(), ee_frame_.c_str());

    detach_object_to_ee(grasp_objects[i]);

    // ------------------- Cartesian move to pose grasp location --------------------------
    RCLCPP_INFO(
      LOGGER,
      MOVEIT_CONSOLE_COLOR_CYAN
      "Cartesian to post release location for [%s]."
      MOVEIT_CONSOLE_COLOR_RESET,
      grasp_objects[i].id.c_str());

    temp_target_pose.pose.position.z = target_pose.pose.position.z + CLEARANCE;
    cartesian_to(
      std::vector<geometry_msgs::msg::Pose>{release_pose.pose},
      ee_link_, 0.01);

    // ------------------- Cartesian move back to Home --------------------------
    move_to(*home_state);  // Robot is above the object
  }
  // -------------------------------------------------------------

  _sig.set_value(true);
}

geometry_msgs::msg::Pose GraspExecution::get_object_pose(const std::string & object_id) const
{
  geometry_msgs::msg::Pose object_pose;
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    collision_detection::World::ObjectConstPtr object = scene->getWorld()->getObject(object_id);
    auto poses = (*object).shape_poses_[0];
    Eigen::Quaterniond q(poses.linear());
    object_pose.position.x = poses.translation().x();
    object_pose.position.y = poses.translation().y();
    object_pose.position.z = poses.translation().z();
    object_pose.orientation.x = q.x();
    object_pose.orientation.y = q.y();
    object_pose.orientation.z = q.z();
    object_pose.orientation.w = q.w();
  }    // Unlock PlanningScene

  return object_pose;
}

geometry_msgs::msg::PoseStamped GraspExecution::get_curr_pose(const std::string & link_name) const
{
  moveit::core::RobotStatePtr state;
  moveit_cpp_->getCurrentState(state, 0);
  const auto & transform = state->getGlobalLinkTransform(link_name);
  ASSERT_ISOMETRY(transform);  // unsanitized input, could contain a non-isometry

  Eigen::Quaterniond q(transform.linear());
  geometry_msgs::msg::PoseStamped output_pose;
  output_pose.header.frame_id = state->getRobotModel()->getRootLinkName();
  output_pose.header.stamp = node_->now();
  output_pose.pose.position.x = transform.translation().x();
  output_pose.pose.position.y = transform.translation().y();
  output_pose.pose.position.z = transform.translation().z();
  output_pose.pose.orientation.x = q.x();
  output_pose.pose.orientation.y = q.y();
  output_pose.pose.orientation.z = q.z();
  output_pose.pose.orientation.w = q.w();
  return output_pose;
}


bool GraspExecution::move_to(const geometry_msgs::msg::PoseStamped & pose, const std::string & link)
{
  auto arm = arms_[PLANNING_GROUP];
  RCLCPP_INFO(LOGGER, SEPARATOR);
  arm->setGoal(pose, link);
  const auto plan_solution = arm->plan();  // PlanningComponent::PlanSolution
  if (plan_solution) {
    RCLCPP_INFO(LOGGER, "Sending the trajectory for execution");
    arm->execute(true);  // blocked execution
    RCLCPP_INFO(LOGGER, SEPARATOR);

    return true;
  } else {
    RCLCPP_INFO(LOGGER, SEPARATOR);
    return false;
  }
}

bool GraspExecution::move_to(const moveit::core::RobotState & state)
{
  auto arm = arms_[PLANNING_GROUP];
  RCLCPP_INFO(LOGGER, SEPARATOR);
  arm->setGoal(state);
  const auto plan_solution = arm->plan();  // PlanningComponent::PlanSolution
  if (plan_solution) {
    RCLCPP_INFO(LOGGER, "Sending the trajectory for execution");
    arm->execute(true);  // blocked execution
    RCLCPP_INFO(LOGGER, SEPARATOR);

    return true;
  } else {
    RCLCPP_INFO(LOGGER, SEPARATOR);
    return false;
  }
}

/// Referenced from Move Group capability MoveGroupCartesianPathService::computeService
bool GraspExecution::cartesian_to(
  const std::vector<geometry_msgs::msg::Pose> & _waypoints,
  const std::string & _link, double step, double jump_threshold)
{
  const std::string & planning_group = PLANNING_GROUP;
  double fraction = 0.0;
  robot_trajectory::RobotTrajectoryPtr rt;

  RCLCPP_INFO(LOGGER, SEPARATOR);

  RCLCPP_INFO(LOGGER, "Received request to compute Cartesian path");
  // Start Cartesian Planning
  {    // Lock PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->updateFrameTransforms();

    // Get start state
    moveit::core::RobotState start_state =
      planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp_->getPlanningSceneMonitor())->
      getCurrentState();
    // Check if planning group valid
    if (const moveit::core::JointModelGroup * jmg =
      start_state.getJointModelGroup(planning_group))
    {
      std::string link_name = _link;
      if (link_name.empty() && !jmg->getLinkModelNames().empty()) {
        link_name = jmg->getLinkModelNames().back();
      }

      EigenSTL::vector_Isometry3d waypoints(_waypoints.size());

      for (size_t i = 0; i < _waypoints.size(); i++) {
        tf2::fromMsg(_waypoints[i], waypoints[i]);
      }

      // TODO(Briancbn): Properly deal with this
      // Create path_constraint
      moveit_msgs::msg::Constraints path_constraints;
      bool avoid_collisions = true;

      if (step < std::numeric_limits<double>::epsilon()) {
        RCLCPP_ERROR(
          LOGGER, "Maximum step to take between consecutive configrations along Cartesian path"
          "was not specified (this value needs to be > 0)");
        RCLCPP_INFO(LOGGER, SEPARATOR);
        return false;
      } else {
        if (!waypoints.empty()) {
          moveit::core::GroupStateValidityCallbackFn constraint_fn;
          std::unique_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
          std::unique_ptr<kinematic_constraints::KinematicConstraintSet> kset;
          if (avoid_collisions || !moveit::core::isEmpty(path_constraints)) {
            ls.reset(
              new planning_scene_monitor::LockedPlanningSceneRO(
                moveit_cpp_->
                getPlanningSceneMonitor()));
            kset.reset(new kinematic_constraints::KinematicConstraintSet((*ls)->getRobotModel()));
            kset->add(path_constraints, (*ls)->getTransforms());
            constraint_fn = [
              planning_scene =
              (avoid_collisions ? static_cast<const planning_scene::PlanningSceneConstPtr &>(*ls)
              .get() : nullptr),
              constraint_set = (kset->empty() ? nullptr : kset.get())
              ](moveit::core::RobotState * state,
                const moveit::core::JointModelGroup * group, const double * ik_solution) {
                state->setJointGroupPositions(group, ik_solution);
                state->update();
                return (!planning_scene ||
                       !planning_scene->isStateColliding(*state, group->getName())) &&
                       (!constraint_set || constraint_set->decide(*state).satisfied);
              };
          }

          bool global_frame = true;

          std::vector<moveit::core::RobotStatePtr> traj;
          fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
            &start_state, jmg, traj, start_state.getLinkModel(link_name), waypoints, global_frame,
            moveit::core::MaxEEFStep(step), moveit::core::JumpThreshold(
              jump_threshold), constraint_fn);

          rt = std::make_shared<robot_trajectory::RobotTrajectory>(
            moveit_cpp_->getPlanningSceneMonitor()->getRobotModel(), planning_group);
          for (const moveit::core::RobotStatePtr & traj_state : traj) {
            rt->addSuffixWayPoint(traj_state, 0.0);
          }

          // time trajectory
          // \todo optionally compute timing to move the eef with constant speed
          trajectory_processing::IterativeParabolicTimeParameterization time_param;
          time_param.computeTimeStamps(*rt, 1.0);

          RCLCPP_INFO(
            LOGGER,
            "Computed Cartesian path with %u points (followed %lf%% of requested trajectory)",
            (unsigned int)traj.size(), fraction * 100.0);
        }
      }
    } else {
      RCLCPP_INFO(LOGGER, SEPARATOR);
      return false;
    }
  }    // Lock PlanningScene

  // Execute cartesian path
  if (fraction > 0) {
    moveit_cpp_->execute(planning_group, rt);
  }
  RCLCPP_INFO(LOGGER, SEPARATOR);
  return true;
}

bool GraspExecution::move_until_before_collide(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & link, double step_size, int max_attempts,
  char axis)
{
  auto next_waypoint = [&axis, &step_size](geometry_msgs::msg::Pose & _pose) {
      switch (axis) {
        case 'x':
          _pose.position.x += step_size;
          break;
        case 'y':
          _pose.position.y += step_size;
          break;
        case 'z':
          _pose.position.z += step_size;
          break;
        default:
          return false;
      }
      return true;
    };
  if (pose.header.frame_id != robot_frame_) {
    RCLCPP_ERROR(LOGGER, "Please use global frame for this function");
    return false;
  }

  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose temp_target_pose = pose.pose;
  if (!next_waypoint(temp_target_pose)) {
    return false;
  }
  for (int i = 0; i < max_attempts; i++) {
    waypoints.push_back(temp_target_pose);
    next_waypoint(temp_target_pose);
  }

  return cartesian_to(waypoints, link, std::abs(step_size / 3));
}

void GraspExecution::attach_object_to_ee(const moveit_msgs::msg::CollisionObject & object)
{
  /*
  moveit_msgs::AttachedCollisionObject
  -----------------------------------
  string link_name
  moveit_msgs/CollisionObject object
  string[] touch_links
  trajectory_msgs/JointTrajectory detach_posture
  float64 weight
  */
  geometry_msgs::msg::PoseStamped ee_pose;
  geometry_msgs::msg::PoseStamped object_pose;

  object_pose.header.frame_id = object.header.frame_id;
  object_pose.pose = object.pose;

  to_frame(object_pose, ee_pose, ee_frame_, *tf_buffer_);

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = ee_frame_;


  attached_object.touch_links = ignore_links_;
  attached_object.object = object;
  attached_object.object.pose = ee_pose.pose;
  attached_object.object.operation = attached_object.object.ADD;

  // Add object to planning scene
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processAttachedCollisionObjectMsg(attached_object);
  }    // Unlock PlanningScene
}

void GraspExecution::detach_object_to_ee(const moveit_msgs::msg::CollisionObject & object)
{
  /*
  moveit_msgs::AttachedCollisionObject
  -----------------------------------
  string link_name
  moveit_msgs/CollisionObject object
  string[] touch_links
  trajectory_msgs/JointTrajectory detach_posture
  float64 weight
  */

  moveit_msgs::msg::AttachedCollisionObject detach_object;

  detach_object.object.id = object.id;
  detach_object.touch_links = ignore_links_;
  detach_object.object.operation = object.REMOVE;

  // Add object to planning scene
  {    // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processAttachedCollisionObjectMsg(detach_object);
  }    // Unlock PlanningScene
}

}  // namespace grasp_execution
