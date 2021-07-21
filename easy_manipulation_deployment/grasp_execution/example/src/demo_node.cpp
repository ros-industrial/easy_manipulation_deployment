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

#include "emd/grasp_execution/moveit2/moveit_cpp_if.hpp"
#include "emd/grasp_execution/utils.hpp"

#include "emd_msgs/msg/grasp_task.hpp"
#include "emd_msgs/srv/grasp_request.hpp"

#include "moveit/macros/console_colors.h"

namespace grasp_execution
{

static const char GRASP_TASK_TOPIC[] = "grasp_tasks";

static const char GRASP_REQUEST_TOPIC[] = "grasp_requests";

class Demo : public moveit2::MoveitCppGraspExecution
{
public:
  explicit Demo(
    const rclcpp::Node::SharedPtr & node,
    const std::string & grasp_task_topic,
    const std::string & grasp_request_topic)
  : MoveitCppGraspExecution(node, 1, 1),
    node_(node)
  {
    grasp_task_sub_ = node_->create_subscription<emd_msgs::msg::GraspTask>(
      grasp_task_topic, 10,
      [ = ](emd_msgs::msg::GraspTask::UniquePtr msg) {
        order_schedule(std::move(msg));
      });

    grasp_req_service_ = node_->create_service<emd_msgs::srv::GraspRequest>(
      grasp_request_topic,
      [ = ](
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<emd_msgs::srv::GraspRequest::Request> req,
        const std::shared_ptr<emd_msgs::srv::GraspRequest::Response> res) -> void
      {
        (void)req_header;
        auto task = std::make_unique<emd_msgs::msg::GraspTask>();
        task->task_id = gen_uuid();
        task->grasp_targets = req->grasp_targets;
        order_schedule(std::move(task), true);
        res->success = true;
      });
  }

  void order_schedule(
    const emd_msgs::msg::GraspTask::SharedPtr & msg,
    bool blocking = false)
  {
    // target id will be "#<shape>-<task_id>-<target-index>"

    // ------------------- Prepare object for grasping --------------------------
    for (size_t i = 0; i < msg->grasp_targets.size(); i++) {
      register_target_object(
        msg->grasp_targets[i].target_shape,
        msg->grasp_targets[i].target_pose,
        i,
        msg->task_id);
    }
    // -------------------------------------------------------------

    for (size_t i = 0; i < msg->grasp_targets.size(); i++) {
      auto grasp_target = std::make_shared<emd_msgs::msg::GraspTarget>(msg->grasp_targets[i]);

      auto target_id =
        gen_target_object_id(grasp_target->target_shape, msg->task_id, i);

      // Start planning workflow using planning schedule
      auto status = planning_scheduler.add_workflow(
        target_id,
        std::bind(
          &Demo::planning_workflow, this,
          std::move(grasp_target),
          std::placeholders::_1));

      // Check if workflow started properly
      switch (status) {
        case core::Workflow::Status::ONGOING:
          RCLCPP_INFO(
            node_->get_logger(),
            MOVEIT_CONSOLE_COLOR_YELLOW
            "New job [%s] started!!"
            MOVEIT_CONSOLE_COLOR_RESET, target_id.c_str());
          break;
        case core::Workflow::Status::QUEUED:
          RCLCPP_INFO(
            node_->get_logger(),
            MOVEIT_CONSOLE_COLOR_YELLOW
            "New job [%s] started!!"
            "No available planning worker, new job in queue."
            MOVEIT_CONSOLE_COLOR_RESET, target_id.c_str());
          break;
        case core::Workflow::Status::INVALID:
          RCLCPP_INFO(
            node_->get_logger(),
            MOVEIT_CONSOLE_COLOR_RED
            "New job [%s] is invalid, it could be already completed, ongoing or queued."
            "You can check with get_status(<workflow-id>), or use another <workflow-id>"
            MOVEIT_CONSOLE_COLOR_RESET, target_id.c_str());
          break;
        default:
          break;
      }

      if (blocking) {
        // Wait for the job to finish
        bool result;
        planning_scheduler.wait_till_complete(target_id, result);
      }
    }
  }

  bool planning_workflow(
    const emd_msgs::msg::GraspTarget::SharedPtr & target,
    const std::string & target_id)
  {
    // Get home state
    moveit::core::RobotStatePtr home_state(get_curr_state());

    // TODO(Briancbn): select grasp method based on end effector availability
    double clearance;
    std::string ee_link = "";
    std::string planning_group = "";
    auto & grasp_method = target->grasp_methods[0];
    const std::string & ee_brand = grasp_method.ee_id;

    // Select the group based on ee brand name
    for (auto & group : get_workcell_context().groups) {
      for (auto & ee : group.second.end_effectors) {
        if (ee.second.brand == ee_brand) {
          ee_link = ee.second.link;
          clearance = ee.second.clearance;
          planning_group = group.first;
          break;
        }
      }
    }

    float cartesian_step_size = static_cast<float>(node_->get_parameter(
        "planning_strategy.cartesian_planning.move_step_length").as_double());

    int backtrack_steps = node_->get_parameter(
      "planning_strategy.cartesian_non_deterministic_hybrid.backtrack_steps").as_int();

    int hybrid_max_attempts = node_->get_parameter(
      "planning_strategy.cartesian_non_deterministic_hybrid.max_planning_tries").as_int();

    int non_deterministic_max_attempts = node_->get_parameter(
      "planning_strategy.non_deterministic.max_planning_tries").as_int();

    // Exit if brand name not found.
    if (ee_link.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "End effector brand: %s", ee_brand.c_str());
    }

    auto release_pose = get_curr_pose(ee_link);

    // TODO(Briancbn): iterate to find the valid grasp_pose within grasp_method
    const auto & grasp_pose = grasp_method.grasp_poses[0];

    bool result;

    // ------------------- Plan to grasp location --------------------------
    prompt_job_start(node_->get_logger(), target_id, "Plan to grasp location.");

    // Initial approach doesn't move down yet
    result = this->default_plan_pre_grasp(
      cartesian_step_size,
      backtrack_steps,
      hybrid_max_attempts,
      non_deterministic_max_attempts,
      planning_group, ee_link, grasp_pose, clearance);

    prompt_job_end(node_->get_logger(), result);

    if (!result) {
      return false;
    }

    // ------------------- Move to grasp location --------------------------
    prompt_job_start(node_->get_logger(), target_id, "Move to grasp location.");

    result = squash_and_execute(planning_group);

    arms_[planning_group].traj.clear();

    prompt_job_end(node_->get_logger(), result);

    if (!result) {
      return false;
    }

    // TODO(Briancbn): Call to gripper driver

    // ------------------- Attach grasp object to robot --------------------------
    prompt_job_start(
      node_->get_logger(), target_id,
      "Attaching to robot ee frame: [" + ee_link + "]");

    attach_object_to_ee(target_id, ee_link);

    result = true;

    prompt_job_end(node_->get_logger(), result);

    if (!result) {
      return false;
    }

    // ------------------- Plan to release location --------------------------
    prompt_job_start(
      node_->get_logger(), target_id,
      "Plan to release location");

    // TODO(Briancbn): Configurable release pose
    geometry_msgs::msg::PoseStamped base_grasp_pose;
    to_frame(grasp_pose, base_grasp_pose, this->robot_frame_);
    release_pose.pose.position.x -= 0.3;
    release_pose.pose.position.z = base_grasp_pose.pose.position.z;
    release_pose.pose.orientation = base_grasp_pose.pose.orientation;

    result = this->default_plan_transport(
      cartesian_step_size,
      backtrack_steps,
      hybrid_max_attempts,
      non_deterministic_max_attempts,
      planning_group, ee_link, release_pose, clearance);

    prompt_job_end(node_->get_logger(), result);
    if (!result) {
      return false;
    }

    // TODO(Briancbn): Call to gripper driver

    // ------------------- Move to release location --------------------------
    prompt_job_start(node_->get_logger(), target_id, "Move to release location.");

    result = squash_and_execute(planning_group);

    prompt_job_end(node_->get_logger(), result);

    arms_[planning_group].traj.clear();

    if (!result) {
      return false;
    }

    // ------------------- detach grasp object from robot --------------------------
    prompt_job_start(
      node_->get_logger(), target_id,
      "Detaching from robot ee frame: [" + ee_link + "]");

    detach_object_from_ee(target_id, ee_link);

    result = true;

    prompt_job_end(node_->get_logger(), result);

    if (!result) {
      return false;
    }

    // ------------------- Move back to Home --------------------------
    prompt_job_start(
      node_->get_logger(), target_id,
      "Move back to home");
    result = move_to(
      non_deterministic_max_attempts,
      planning_group, *home_state);

    prompt_job_end(node_->get_logger(), result);

    if (!result) {
      return false;
    }
    // -------------------------------------------------------------

    // ------------------ Remove Object from world -------------------

    prompt_job_start(
      node_->get_logger(), target_id,
      "Remove object from world");

    remove_object(target_id);

    result = true;

    prompt_job_end(node_->get_logger(), result);
    return result;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<emd_msgs::msg::GraspTask>::SharedPtr grasp_task_sub_;
  rclcpp::Service<emd_msgs::srv::GraspRequest>::SharedPtr grasp_req_service_;
};

}  // namespace grasp_execution

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node =
    rclcpp::Node::make_shared("grasp_execution_demo_node", "", node_options);

  grasp_execution::Demo demo(
    node, grasp_execution::GRASP_TASK_TOPIC, grasp_execution::GRASP_REQUEST_TOPIC);

  const std::string workcell_context_path =
    node->get_parameter("workcell_context").as_string();
  demo.init_from_yaml(workcell_context_path);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
