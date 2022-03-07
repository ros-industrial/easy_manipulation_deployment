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
#include <vector>

#include "test_replanner.hpp"
#include "emd/dynamic_safety/replanner.hpp"
#include "gtest/gtest.h"

namespace test_dynamic_safety
{

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
  std::thread thr([&result, this]() {
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

// cppcheck-suppress syntaxError
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
