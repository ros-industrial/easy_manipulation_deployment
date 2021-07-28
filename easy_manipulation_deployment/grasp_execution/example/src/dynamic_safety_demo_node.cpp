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

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "grasp_execution/moveit2/moveit_cpp_if.hpp"

class DynamicSafetyDemo : public grasp_execution::moveit2::MoveitCppGraspExecution
{
public:
  explicit DynamicSafetyDemo(
    const rclcpp::Node::SharedPtr & node)
  : MoveitCppGraspExecution(node),
    node_(node)
  {}

  bool demo_test()
  {
    double clearance = 0.05;

    // Get home state
    moveit::core::RobotStatePtr home_state(get_curr_state());

    std::string ee_link = "ee_palm";
    std::string planning_group = "manipulator";
    auto end_pose = get_curr_pose(ee_link);
    end_pose.pose.position.x -= 0.6;
    end_pose.pose.position.z -= 0.2;

    bool result;

    result = this->default_plan_pre_grasp(
      planning_group, ee_link, end_pose, clearance);
    prompt_job_end(node_->get_logger(), result);
    if (!result) {
      return false;
    }

    // Create collision object to trigger dynamic safety
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "box";

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.resize(3);
    box.dimensions = {0.02, 0.5, 0.35};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.4;
    box_pose.position.y = 0;
    box_pose.position.z = 0.15;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add object to planning scene
    {  // Lock PlanningScene
      RCLCPP_INFO(node_->get_logger(), "Add object to planning scene");
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
    }  // Unlock PlanningScene

    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(node_->get_logger(), "Execute traj");
    result = this->squash_and_execute(planning_group, "ds_async");
    prompt_job_end(node_->get_logger(), result);
    if (!result) {
      return false;
    }
    return true;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node =
    rclcpp::Node::make_shared("dynamic_safety_demo_node", "", node_options);

  DynamicSafetyDemo demo(node);

  const std::string workcell_context_path =
    node->get_parameter("workcell_context").as_string();
  demo.init_from_yaml(workcell_context_path);

/*
  demo.init(
    "manipulator", "ee_palm",
    "ds_async", "grasp_execution/DynamicSafetyAsyncExecutor", "ur5_arm_controller");
*/

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread thr(
    [&executor]() {executor.spin();});

  rclcpp::sleep_for(std::chrono::seconds(20));
  demo.demo_test();

  thr.join();

  rclcpp::shutdown();
  return 0;
}
