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

#include <memory>
#include <string>

#include "test_hardware.hpp"
#include "emd/dynamic_safety/replanner.hpp"
#include "gtest/gtest.h"

namespace test_dynamic_safety
{

void print_traj(const trajectory_msgs::msg::JointTrajectory::SharedPtr result)
{
  printf("joint_names:\n");
  for (auto & name : result->joint_names) {
    printf("- %s\n", name.c_str());
  }
  printf("points:\n");
  for (auto & point : result->points) {
    printf("- positions:\n");
    for (auto & position : point.positions) {
      printf("  - %.8f\n", position);
    }

    printf("  velocities:\n");
    for (auto & velocity : point.velocities) {
      printf("  - %.8f\n", velocity);
    }
    printf("  accelerations:\n");
    for (auto & acceleration : point.accelerations) {
      printf("  - %.8f\n", acceleration);
    }
    printf("  efforts:\n");
    for (auto & effort : point.effort) {
      printf("  - %.8f\n", effort);
    }
    printf("  time_from_start: %.8fsecs\n",
      rclcpp::Duration(point.time_from_start).seconds());
  }

}

class ReplannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Load robot model
    robot_ = TestHardware();
    robot_.load_urdf("moveit_resources_panda_description", "urdf/panda.urdf");
    robot_.load_srdf(std::string(TEST_DIRECTORY) + "/panda.srdf");

    // Initialize default option for collision checking
    option_.framework = "moveit";
    option_.planner = "ompl";
    option_.group = "panda_arm";
    option_.deadline = 1.0;

    joint_names_ = std::vector<std::string>{
      "panda_joint1",
      "panda_joint2",
      "panda_joint3",
      "panda_joint4",
      "panda_joint5",
      "panda_joint6",
      "panda_joint7",
    };

    option_.planner_parameter_server = "/test_planner_config_node";
    option_.planner_parameter_namespace = "ompl";
    option_.joint_limits_parameter_server = "/test_joint_limit_node";
    option_.joint_limits_parameter_namespace = "robot_description_planning.joint_limits";

    // OMPL Planner configuration
    planner_config_node_ = std::make_shared<rclcpp::Node>("test_planner_config_node");
    planner_config_node_->declare_parameter(
        "ompl.panda_arm.planner_configs",
        std::vector<std::string>{
          "RRTConnectkConfigDefault",
          "LBKPIECEkConfigDefault"
        });

    planner_config_node_->declare_parameter(
        "ompl.planner_configs.RRTConnectkConfigDefault.type",
        "geometric::RRTConnect");

    planner_config_node_->declare_parameter(
        "ompl.planner_configs.RRTConnectkConfigDefault.range",
        0.0);

    planner_config_node_->declare_parameter(
        "ompl.planner_configs.LBKPIECEkConfigDefault.type",
        "geometric::LBKPIECE");
    planner_config_node_->declare_parameter(
        "ompl.planner_configs.LBKPIECEkConfigDefault.range",
        0.0);
    planner_config_node_->declare_parameter(
        "ompl.planner_configs.LBKPIECEkConfigDefault.border_fraction",
        0.9);
    planner_config_node_->declare_parameter(
        "ompl.planner_configs.LBKPIECEkConfigDefault.min_valid_path_fraction",
        "0.5");
    // Projection evaluator is needed for LBKPIECE
    planner_config_node_->declare_parameter(
        "ompl.panda_arm.projection_evaluator",
        "joints(panda_joint1,panda_joint2)");

    // Joint limit configuration
    joint_limit_node_ = std::make_shared<rclcpp::Node>("test_joint_limit_node");
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint1.has_acceleration_limits",
      true);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint2.has_acceleration_limits",
      true);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint3.has_acceleration_limits",
      true);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint4.has_acceleration_limits",
      true);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint5.has_acceleration_limits",
      true);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint6.has_acceleration_limits",
      true);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint7.has_acceleration_limits",
      true);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint1.max_acceleration",
      3.75);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint2.max_acceleration",
      1.875);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint3.max_acceleration",
      2.5);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint4.max_acceleration",
      3.125);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint5.max_acceleration",
      3.75);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint6.max_acceleration",
      5.0);
    joint_limit_node_->declare_parameter(
      "robot_description_planning.joint_limits.panda_joint7.max_acceleration",
      5.0);

    ok_ = true;

    thr_ = std::thread([this](){
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(planner_config_node_);
        executor.add_node(joint_limit_node_);
        while (ok_) {
          executor.spin_some();
        }
      });
  }
  void TearDown() override
  {
    ok_ = false;
    thr_.join();
  }

  TestHardware robot_;
  dynamic_safety::ReplannerOption option_;
  trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory_;

  dynamic_safety::Replanner replanner_;
  rclcpp::Node::SharedPtr planner_config_node_;
  rclcpp::Node::SharedPtr joint_limit_node_;
  std::vector<std::string> joint_names_;

  std::thread thr_;
  std::atomic_bool ok_;
};

#ifdef EMD_DYNAMIC_SAFETY_MOVEIT
// cppcheck-suppress syntaxError
TEST_F(ReplannerTest, MoveItOMPLReplanner)
{
  // Dummy node
  auto replanner_node = std::make_shared<rclcpp::Node>("test_replanner");

  // Initialize option for replanning
  option_.framework = "moveit";
  option_.planner = "ompl";
  option_.ompl_planner_id = "RRTConnectkConfigDefault";
  option_.group = "panda_arm";
  option_.deadline = 1.0;

  replanner_.configure(option_, replanner_node, robot_.get_urdf(), robot_.get_srdf());
  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.positions = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.positions = {0, 0, 0, 0, 0, 1.571, 0.785};

  auto result = std::make_shared<trajectory_msgs::msg::JointTrajectory>();

  // Idle state before planning
  EXPECT_EQ(replanner_.get_status(), dynamic_safety::ReplannerStatus::IDLE);
  replanner_.run_async(joint_names_, start_point, end_point);
  EXPECT_EQ(replanner_.get_status(), dynamic_safety::ReplannerStatus::ONGOING);
  std::thread thr([&result, this](){
    result = replanner_.get_result();
  });
  rclcpp::sleep_for(std::chrono::seconds(1));
  EXPECT_EQ(replanner_.get_status(), dynamic_safety::ReplannerStatus::IDLE);
  thr.join();
  print_traj(result);

  option_.ompl_planner_id = "LBKPIECEkConfigDefault";
  replanner_.configure(option_, replanner_node, robot_.get_urdf(), robot_.get_srdf());
  replanner_.run_async(joint_names_, start_point, end_point);
  result = replanner_.get_result();
  print_traj(result);
}

TEST_F(ReplannerTest, MoveItOMPLReplannerTimeout)
{
  // Dummy node
  auto replanner_node = std::make_shared<rclcpp::Node>("test_replanner");

  // Initialize option for replanning
  option_.framework = "moveit";
  option_.planner = "ompl";
  option_.ompl_planner_id = "";
  option_.group = "panda_arm";
  option_.deadline = 0;

  replanner_.configure(option_, replanner_node, robot_.get_urdf(), robot_.get_srdf());
  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.positions = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.positions = {0, 0, 0, 0, 0, 1.571, 0.785};

  replanner_.run_async(joint_names_, start_point, end_point);
  EXPECT_EQ(replanner_.get_status(), dynamic_safety::ReplannerStatus::TIMEOUT);
}
#endif

}  // namespace test_dynamic_safety

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
