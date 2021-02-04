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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "grasp_execution/moveit_cpp_if.hpp"
#include "grasp_execution/utils.hpp"

#include "moveit/macros/console_colors.h"

namespace grasp_execution
{

static const char PLANNING_GROUP[] = "manipulator";

static const char CAMERA_FRAME[] = "camera_frame";

static const char EE_LINK[] = "ee_palm";

static const float CLEARANCE = 0.1;

class Demo : public MoveitCppGraspExecution
{
public:
  explicit Demo(const rclcpp::Node::SharedPtr & node)
  : MoveitCppGraspExecution(node),
    node_(node)
  {}

  void order_schedule(
    const grasp_planning::msg::GraspPose::SharedPtr & msg) override
  {
    int worker_id = planning_scheduler.add_workflow(
      std::bind(&Demo::planning_workflow, this, msg, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "New Job Added!!");
    if (worker_id < 0) {
      RCLCPP_INFO(node_->get_logger(), "No available planning worker, job in queue.");
    }
  }

  void planning_workflow(
    const grasp_planning::msg::GraspPose::SharedPtr & msg,
    std::promise<bool> && _sig) override
  {
    std::string ee_link = EE_LINK;
    std::string planning_group = PLANNING_GROUP;
    auto release_pose = get_curr_pose(ee_link);
    double clearance = CLEARANCE;

    // Get home state
    moveit::core::RobotStatePtr home_state;
    moveit_cpp_->getCurrentState(home_state, 0);

    // ------------------- Prepare object for grasping --------------------------
    RCLCPP_INFO(
      node_->get_logger(),
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
      RCLCPP_INFO(node_->get_logger(), oss.str());

      auto tmp_pose = msg->object_poses[i];
      to_frame(msg->object_poses[i], tmp_pose, robot_frame_, *tf_buffer_);
      std::ostringstream oss2;
      print_pose(tmp_pose, oss2);
      RCLCPP_INFO(node_->get_logger(), oss2.str());


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
        node_->get_logger(),
        MOVEIT_CONSOLE_COLOR_CYAN
        "Moving to pre-grasp location for [%s]."
        MOVEIT_CONSOLE_COLOR_RESET,
        grasp_objects[i].id.c_str());

      // Initial approach doesn't move down yet
      temp_target_pose.pose.position.z = target_pose.pose.position.z + clearance;

      // Print out target pose for debug purpose
      std::ostringstream oss;
      print_pose(temp_target_pose, oss);
      RCLCPP_INFO(node_->get_logger(), oss.str());

      // Robot is above the object
      move_to(planning_group, temp_target_pose, ee_link);

      // ------------------- Approaching grasp location --------------------------
      RCLCPP_INFO(
        node_->get_logger(),
        MOVEIT_CONSOLE_COLOR_CYAN
        "Approaching grasp location for [%s]. "
        MOVEIT_CONSOLE_COLOR_RESET,
        grasp_objects[i].id.c_str());
      // Move down to pick
      move_until_before_collide(planning_group, temp_target_pose, ee_link, -0.002, 40, 'z');

      // TODO(Briancbn): Call to gripper driver

      // ------------------- Attach grasp object to robot --------------------------
      RCLCPP_INFO(
        node_->get_logger(),
        MOVEIT_CONSOLE_COLOR_CYAN
        "Attaching [%s] to robot ee frame: [%s]."
        MOVEIT_CONSOLE_COLOR_RESET,
        grasp_objects[i].id.c_str(), ee_link.c_str());

      attach_object_to_ee(grasp_objects[i], ee_link);

      // ------------------- Cartesian move to pose grasp location --------------------------
      RCLCPP_INFO(
        node_->get_logger(),
        MOVEIT_CONSOLE_COLOR_CYAN
        "Cartesian to post grasp location for [%s]."
        MOVEIT_CONSOLE_COLOR_RESET,
        grasp_objects[i].id.c_str());

      temp_target_pose.pose.position.z = target_pose.pose.position.z + clearance;
      cartesian_to(
        planning_group,
        std::vector<geometry_msgs::msg::Pose>{temp_target_pose.pose},
        ee_link, 0.01);

      // ------------------- Cartesian move to release location --------------------------
      RCLCPP_INFO(
        node_->get_logger(),
        MOVEIT_CONSOLE_COLOR_CYAN
        "Cartesian to pre-release location for [%s]."
        MOVEIT_CONSOLE_COLOR_RESET,
        grasp_objects[i].id.c_str());

      // TODO(Briancbn): Configurable release pose
      release_pose.pose.position.x -= 0.3;
      release_pose.pose.orientation = target_pose.pose.orientation;
      release_pose.pose.position.z = target_pose.pose.position.z + clearance;

      cartesian_to(
        planning_group,
        std::vector<geometry_msgs::msg::Pose>{release_pose.pose},
        ee_link, 0.01);

      // ------------------- Approaching release location --------------------------
      RCLCPP_INFO(
        node_->get_logger(),
        MOVEIT_CONSOLE_COLOR_CYAN
        "Approaching release location for [%s]."
        MOVEIT_CONSOLE_COLOR_RESET,
        grasp_objects[i].id.c_str());
      // move to release
      move_until_before_collide(planning_group, release_pose, ee_link, -0.01, 20, 'z');

      // TODO(Briancbn): Call to gripper driver

      // ------------------- detach grasp object from robot --------------------------
      RCLCPP_INFO(
        node_->get_logger(),
        MOVEIT_CONSOLE_COLOR_CYAN
        "Detaching [%s] from robot ee frame: [%s]."
        MOVEIT_CONSOLE_COLOR_RESET,
        grasp_objects[i].id.c_str(), ee_link.c_str());

      detach_object_from_ee(grasp_objects[i], ee_link);

      // ------------------- Cartesian move to pose grasp location --------------------------
      RCLCPP_INFO(
        node_->get_logger(),
        MOVEIT_CONSOLE_COLOR_CYAN
        "Cartesian to post release location for [%s]."
        MOVEIT_CONSOLE_COLOR_RESET,
        grasp_objects[i].id.c_str());

      temp_target_pose.pose.position.z = target_pose.pose.position.z + clearance;
      cartesian_to(
        planning_group,
        std::vector<geometry_msgs::msg::Pose>{release_pose.pose},
        ee_link, 0.01);

      // ------------------- Cartesian move back to Home --------------------------
      move_to(planning_group, *home_state);  // Robot is above the object
    }
    // -------------------------------------------------------------

    _sig.set_value(true);
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace grasp_execution

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node =
    rclcpp::Node::make_shared("grasp_execution_demo_node", "", node_options);

  grasp_execution::Demo demo(node);

  demo.init("manipulator", "ee_palm");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
