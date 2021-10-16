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
#include "emd/dynamic_safety/collision_checker.hpp"
#include "gtest/gtest.h"

namespace test_dynamic_safety
{

class CollisionCheckingTest : public ::testing::Test
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
    option_.group = "panda_arm";
    option_.distance = false;
    option_.continuous = false;
    option_.realtime = false;
    option_.step = 0.05;
    option_.thread_count = static_cast<int>(std::thread::hardware_concurrency());

    trajectory_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();

    trajectory_->joint_names = {
      "panda_joint1",
      "panda_joint2",
      "panda_joint3",
      "panda_joint4",
      "panda_joint5",
      "panda_joint6",
      "panda_joint7"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;

    // First point
    point.positions = {0, -M_PI / 4, 0, -M_PI * 3 / 4, 0, M_PI / 2, M_PI / 4};
    point.time_from_start = rclcpp::Duration(0);
    trajectory_->points.push_back(point);

    // Second point
    point.positions = {0, -M_PI / 4, 0, -M_PI * 1 / 4, 0, M_PI / 4, M_PI / 4};
    point.time_from_start = rclcpp::Duration::from_seconds(1.004);
    trajectory_->points.push_back(point);

    // Third point
    point.positions = {0, 0, 0, 0, 0, 0, 0};
    point.time_from_start = rclcpp::Duration::from_seconds(2.01);
    trajectory_->points.push_back(point);
  }

  TestHardware robot_;
  dynamic_safety::CollisionCheckerOption option_;
  trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory_;

  dynamic_safety::CollisionChecker collision_checker_;
};

#ifdef EMD_DYNAMIC_SAFETY_MOVEIT
// cppcheck-suppress syntaxError
TEST_F(CollisionCheckingTest, MoveItDiscreteFCLPolling)
{
  option_.framework = "moveit";
  option_.group = "panda_arm";
  option_.distance = false;
  option_.continuous = false;
  option_.collision_checking_plugin = "fcl";
  collision_checker_.configure(
    option_,
    robot_.get_urdf(),
    robot_.get_srdf()
  );

  collision_checker_.add_trajectory(trajectory_);

  printf(
    "Average MoveIt FCL Discrete "
    "collision_checking time for 1.0s look ahead time: %.4fms\n",
    collision_checker_.polling(1.0, 20) * 1e3);
}

#ifndef EMD_DYNAMIC_SAFETY_TESSERACT
TEST_F(CollisionCheckingTest, MoveItDiscreteBulletPolling)
{
  option_.collision_checking_plugin = "bullet";
  collision_checker_.configure(
    option_,
    robot_.get_urdf(),
    robot_.get_srdf()
  );

  collision_checker_.add_trajectory(trajectory_);
  printf(
    "Average MoveIt Bullet Discrete "
    "collision_checking time for 1.0s look ahead time: %.4fms\n",
    collision_checker_.polling(1.0, 20) * 1e3);
}
#endif

TEST_F(CollisionCheckingTest, MoveitDiscreteFCL)
{
  option_.collision_checking_plugin = "fcl";
  option_.continuous = false;
  collision_checker_.configure(
    option_,
    robot_.get_urdf(),
    robot_.get_srdf()
  );
  collision_checker_.add_trajectory(trajectory_);
  double collision_time;

  // Check end state
  collision_checker_.run_once(2.0, 0.0, collision_time);
  EXPECT_NE(collision_time, -1.0);

  // Check the whole trajectory at once
  collision_checker_.run_once(0, 2, collision_time);
  EXPECT_NEAR(collision_time, 1.66, 0.0001);
}

#ifndef EMD_DYNAMIC_SAFETY_TESSERACT
TEST_F(CollisionCheckingTest, MoveItDiscreteBullet)
{
  option_.collision_checking_plugin = "bullet";

  collision_checker_.configure(
    option_,
    robot_.get_urdf(),
    robot_.get_srdf()
  );
  collision_checker_.add_trajectory(trajectory_);
  double collision_time;

  // Check end state
  collision_checker_.run_once(0, 2.0, collision_time);
  EXPECT_NE(collision_time, -1.0);
}

TEST_F(CollisionCheckingTest, MoveItContinuousBullet)
{
  option_.collision_checking_plugin = "bullet";
  option_.continuous = true;

  collision_checker_.configure(
    option_,
    robot_.get_urdf(),
    robot_.get_srdf()
  );
  collision_checker_.add_trajectory(trajectory_);
  double collision_time;

  // Check end state
  collision_checker_.run_once(0, 2.0, collision_time);
  EXPECT_NE(collision_time, -1.0);
}
#endif
#endif

#ifdef EMD_DYNAMIC_SAFETY_TESSERACT
// cppcheck-suppress syntaxError
TEST_F(CollisionCheckingTest, TesseractDiscreteFCLPolling)
{
  option_.framework = "tesseract";
  collision_checker_.configure(
    option_,
    robot_.get_urdf(),
    robot_.get_srdf()
  );

  collision_checker_.add_trajectory(trajectory_);

  printf(
    "Average Tesseract FCL Discrete "
    "collision_checking time for 1.0s look ahead time: %.4fms\n",
    collision_checker_.polling(1.0, 20) * 1e3);
}

TEST_F(CollisionCheckingTest, TesseractDiscreteBulletPolling)
{
  option_.framework = "tesseract";
  option_.collision_checking_plugin = "bullet";
  collision_checker_.configure(
    option_,
    robot_.get_urdf(),
    robot_.get_srdf()
  );

  collision_checker_.add_trajectory(trajectory_);
  printf(
    "Average MoveIt Bullet Discrete "
    "collision_checking time for 1.0s look ahead time: %.4fms\n",
    collision_checker_.polling(1.0, 20) * 1e3);
}

TEST_F(CollisionCheckingTest, TesseractDiscreteFCL)
{
  option_.framework = "tesseract";
  option_.collision_checking_plugin = "fcl";
  option_.continuous = false;
  collision_checker_.configure(
    option_,
    robot_.get_urdf(),
    robot_.get_srdf()
  );
  collision_checker_.add_trajectory(trajectory_);
  double collision_time;

  // Check end state
  collision_checker_.run_once(2.0, 0.0, collision_time);
  EXPECT_NE(collision_time, -1.0);

  // Check the whole trajectory at once
  collision_checker_.run_once(0, 2, collision_time);
  EXPECT_NEAR(collision_time, 1.66, 0.0001);
}

TEST_F(CollisionCheckingTest, TesseractDiscreteBullet)
{
  option_.framework = "tesseract";
  option_.collision_checking_plugin = "bullet";

  collision_checker_.configure(
    option_,
    robot_.get_urdf(),
    robot_.get_srdf()
  );
  collision_checker_.add_trajectory(trajectory_);
  double collision_time;

  // Check end state
  collision_checker_.run_once(0, 2.0, collision_time);
  EXPECT_NE(collision_time, -1.0);
}

#endif

}  // namespace test_dynamic_safety
