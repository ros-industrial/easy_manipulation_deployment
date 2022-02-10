// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
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

#include <gtest/gtest.h>
#include "grasp_scene_test.hpp"

GraspSceneTest::GraspSceneTest()
{
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  node = rclcpp::Node::make_shared("grasp_scene_test", "", node_options);
}

// // Uncomment once Visualizer is removed from the graspScene class
// TEST_F(GraspSceneTest, PrintPoseTest)
// {
//   geometry_msgs::msg::Pose test_pose;
//   test_pose.position.x = 0.1;
//   test_pose.position.y = 0.2;
//   test_pose.position.z = 0.3;
//   test_pose.orientation.x = 0.01;
//   test_pose.orientation.y = 0.02;
//   test_pose.orientation.z = 0.03;
//   test_pose.orientation.w = 0.04;


//   grasp_planner::GraspScene<epd_msgs::msg::EPDObjectTracking> test_tracking(node);

//   grasp_planner::GraspScene<epd_msgs::msg::EPDObjectLocalization> test_localization(node);

//   grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2> test_direct(node);

//   test_tracking.printPose(test_pose);
//   test_localization.printPose(test_pose);
//   test_direct.printPose(test_pose);
// }
