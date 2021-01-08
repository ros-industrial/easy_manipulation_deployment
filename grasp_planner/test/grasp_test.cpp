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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include "grasps.hpp"
#include "load_perception.hpp"
#include "planning_functions.hpp"

TEST(FingerGripperTest, EmptyFinger)
{
  TwoFinger empty_finger;

  EXPECT_EQ(0, empty_finger.corner1[0]);
  EXPECT_EQ(0, empty_finger.corner1[1]);
  EXPECT_EQ(0, empty_finger.corner3[0]);
  EXPECT_EQ(0, empty_finger.corner3[1]);
  EXPECT_EQ(0, empty_finger.centre[0]);
  EXPECT_EQ(0, empty_finger.centre[1]);
}
TEST(FingerGripperTest, FingerSmallerThanObject)
{
  goto_folder_filepath("test/perception_output/regular_object/horizontal", "grasp_planning");
  epd_msgs::msg::EPDObjectLocalization::SharedPtr msg = LoadPerception();
  cv::Mat depth_img = load_depth_img();
  std::vector<float> angles;
  for (int i = 0; i < static_cast<int>(msg->num_objects); i++) {
    double yaw, pitch, roll;
    tf2::Matrix3x3(
      tf2::Quaternion(
        msg->objects[i].pos.pose.orientation.x,
        msg->objects[i].pos.pose.orientation.y,
        msg->objects[i].pos.pose.orientation.z,
        msg->objects[i].pos.pose.orientation.w))
    .getEulerYPR(yaw, pitch, roll);

    angles.push_back(yaw);
  }
  Msg message(msg, angles);
  TwoFinger grasp = load_finger_attributes("short.yaml", "test/test_yaml", "grasp_planning");
  grasp.get_checkpoints(message);

  for (int i = 0; i < static_cast<int>(grasp.checkpoint_startx[0].size()); i++) {
    EXPECT_TRUE(grasp.checkpoint_startx[0][i] == grasp.checkpoint_endx[0][i]);
    EXPECT_TRUE(grasp.checkpoint_startx[0][i] >= message.tl_x[0]);
    EXPECT_TRUE(grasp.checkpoint_startx[0][i] < message.br_x[0] + 1);
    EXPECT_TRUE(grasp.checkpoint_endx[0][i] >= message.tl_x[0]);
    EXPECT_TRUE(grasp.checkpoint_endx[0][i] < message.br_x[0] + 1);
  }
}
TEST(FingerGripperTest, BoundingBoxAtEdgeLeft)
{
  goto_folder_filepath("test/perception_output/regular_object/horizontal", "grasp_planning");

  // REPLACE WHEN FINAL RELEASE
  epd_msgs::msg::EPDObjectLocalization::SharedPtr msg = LoadPerception();

  msg->roi_array[0].x_offset = 0;
  msg->roi_array[0].y_offset = 0;
  cv::Mat depth_img = load_depth_img();
  std::vector<float> angles;
  for (int i = 0; i < static_cast<int>(msg->num_objects); i++) {
    double yaw, pitch, roll;
    tf2::Matrix3x3(
      tf2::Quaternion(
        msg->objects[i].pos.pose.orientation.x,
        msg->objects[i].pos.pose.orientation.y,
        msg->objects[i].pos.pose.orientation.z,
        msg->objects[i].pos.pose.orientation.w))
    .getEulerYPR(yaw, pitch, roll);
    angles.push_back(yaw);
  }

  Msg message(msg, angles);
  TwoFinger grasp = load_finger_attributes(
    "attributes.yaml",
    "test/test_yaml",
    "grasp_planning");
  grasp.get_checkpoints(message);
  EXPECT_EQ(0, static_cast<int>(grasp.checkpoint_startx[0][0]));
  EXPECT_EQ(0, static_cast<int>(grasp.checkpoint_starty[0][0]));
}

TEST(FingerGripperTest, BoundingBoxAtEdgeRight)
{
  goto_folder_filepath("test/perception_output/regular_object/horizontal", "grasp_planning");

  epd_msgs::msg::EPDObjectLocalization::SharedPtr msg = LoadPerception();

  msg->roi_array[0].x_offset = 640 - 214;
  msg->roi_array[0].y_offset = 480 - 140;
  cv::Mat depth_img = load_depth_img();
  std::vector<float> angles;
  for (int i = 0; i < static_cast<int>(msg->num_objects); i++) {
    double yaw, pitch, roll;
    tf2::Matrix3x3(
      tf2::Quaternion(
        msg->objects[i].pos.pose.orientation.x,
        msg->objects[i].pos.pose.orientation.y,
        msg->objects[i].pos.pose.orientation.z,
        msg->objects[i].pos.pose.orientation.w))
    .getEulerYPR(yaw, pitch, roll);

    angles.push_back(yaw);
  }

  Msg message(msg, angles);
  TwoFinger grasp = load_finger_attributes(
    "attributes.yaml",
    "test/test_yaml",
    "grasp_planning");
  grasp.get_checkpoints(message);
  EXPECT_EQ(640, static_cast<int>(grasp.checkpoint_endx[0].back()));
  EXPECT_EQ(480, static_cast<int>(grasp.checkpoint_endy[0].back()));
}
TEST(FingerGripperTest, HorizontalObject)
{
  goto_folder_filepath("test/perception_output/regular_object/horizontal", "grasp_planning");
  // REPLACE WHEN FINAL RELEASE
  epd_msgs::msg::EPDObjectLocalization::SharedPtr msg = LoadPerception();

  cv::Mat depth_img = load_depth_img();
  std::vector<float> angles;
  for (int i = 0; i < static_cast<int>(msg->num_objects); i++) {
    double yaw, pitch, roll;
    tf2::Matrix3x3(
      tf2::Quaternion(
        msg->objects[i].pos.pose.orientation.x,
        msg->objects[i].pos.pose.orientation.y,
        msg->objects[i].pos.pose.orientation.z,
        msg->objects[i].pos.pose.orientation.w))
    .getEulerYPR(yaw, pitch, roll);
    angles.push_back(yaw);
  }
  Msg message(msg, angles);
  TwoFinger grasp = load_finger_attributes(
    "attributes.yaml",
    "test/test_yaml",
    "grasp_planning");
  grasp.get_checkpoints(message);

  EXPECT_EQ(1, static_cast<int>(grasp.gradient_vector.size()));
  EXPECT_EQ(1, static_cast<int>(grasp.c1_vector.size()));
  EXPECT_EQ(1, static_cast<int>(grasp.c2_vector.size()));
  EXPECT_EQ(0, static_cast<int>(grasp.c1_vector[0]));
  EXPECT_EQ(0, static_cast<int>(grasp.c2_vector[0]));

  EXPECT_GT(grasp.checkpoint_startx.size(), 0);
  EXPECT_GT(grasp.checkpoint_starty.size(), 0);
  EXPECT_GT(grasp.checkpoint_endx.size(), 0);
  EXPECT_GT(grasp.checkpoint_endy.size(), 0);

  grasp.get_best_grasp(message, depth_img, 0);
  EXPECT_GE(grasp.centre[0], 229);
  EXPECT_LE(grasp.centre[0], 434);
  EXPECT_GE(grasp.centre[1], 162);
  EXPECT_LE(grasp.centre[1], 307);

  std::vector<std::vector<int>> test_points = grasp.find_2_points(
    {209, 109},
    {250, 359}, 0);

  EXPECT_NEAR(250, test_points[0][0], 3);
  EXPECT_NEAR(109, test_points[0][1], 3);
  EXPECT_NEAR(209, test_points[1][0], 3);
  EXPECT_NEAR(359, test_points[1][1], 3);

  std::vector<std::vector<int>> test_points_neg1 = grasp.find_2_points(
    {0, 0},
    {2, 2}, 0);
  EXPECT_EQ(0, static_cast<int>(test_points_neg1.size()));

  std::vector<std::vector<int>> test_points_neg2 = grasp.find_2_points(
    {50, 20},
    {-30, -359}, 0);
  EXPECT_EQ(0, static_cast<int>(test_points_neg2.size()));

  int valid_grasp = grasp.get_gdi_value(
    depth_img,
    {217, 154},
    {209, 109},
    {250, 359},
    {249, 109},
    {210, 359});
  EXPECT_EQ(valid_grasp, 0);

  int invalid_grasp = grasp.get_gdi_value(
    depth_img,
    {18, 15},
    {10, 10},
    {25, 10},
    {25, 20},
    {10, 20});
  EXPECT_EQ(0, invalid_grasp);

  int valid_corner = grasp.get_corner_gdi({230, 235}, 40, depth_img, 500);
  EXPECT_EQ(valid_corner, 0);

  int invalid_corner = grasp.get_corner_gdi({21, 15}, 40, depth_img, 50);
  EXPECT_EQ(0, invalid_corner);
}

TEST(FingerGripperTest, VerticalObject)
{
  goto_folder_filepath("test/perception_output/regular_object/vertical", "grasp_planning");

  // REPLACE WHEN FINAL RELEASE
  epd_msgs::msg::EPDObjectLocalization::SharedPtr msg = LoadPerception();

  cv::Mat depth_img = load_depth_img();
  std::vector<float> angles;
  for (int i = 0; i < static_cast<int>(msg->num_objects); i++) {
    double yaw, pitch, roll;
    tf2::Matrix3x3(
      tf2::Quaternion(
        msg->objects[i].pos.pose.orientation.x,
        msg->objects[i].pos.pose.orientation.y,
        msg->objects[i].pos.pose.orientation.z,
        msg->objects[i].pos.pose.orientation.w))
    .getEulerYPR(yaw, pitch, roll);

    angles.push_back(yaw);
  }
  Msg message(msg, angles);
  TwoFinger grasp = load_finger_attributes(
    "attributes.yaml",
    "test/test_yaml",
    "grasp_planning");
  grasp.get_checkpoints(message);

  EXPECT_EQ(1, static_cast<int>(grasp.gradient_vector.size()));
  EXPECT_EQ(1, static_cast<int>(grasp.c1_vector.size()));
  EXPECT_EQ(1, static_cast<int>(grasp.c2_vector.size()));
  EXPECT_EQ(0, static_cast<int>(grasp.c1_vector[0]));
  EXPECT_EQ(0, static_cast<int>(grasp.c2_vector[0]));

  EXPECT_GT(grasp.checkpoint_startx.size(), 0);
  EXPECT_GT(grasp.checkpoint_starty.size(), 0);
  EXPECT_GT(grasp.checkpoint_endx.size(), 0);
  EXPECT_GT(grasp.checkpoint_endy.size(), 0);

  std::vector<std::vector<int>> test_points = grasp.find_2_points(
    {446, 359},
    {196, 317}, 0);

  EXPECT_NEAR(446, test_points[0][0], 3);
  EXPECT_NEAR(317, test_points[0][1], 3);
  EXPECT_NEAR(196, test_points[1][0], 3);
  EXPECT_NEAR(359, test_points[1][1], 3);

  int valid_grasp = grasp.get_gdi_value(
    depth_img,
    {321, 338},
    {446, 359},
    {196, 317},
    {446, 319},
    {196, 357});

  EXPECT_EQ(valid_grasp, 0);
  int valid_corner = grasp.get_corner_gdi({446, 339}, 40, depth_img, 500);
  EXPECT_GT(valid_corner, 0);
}

TEST(FingerGripperTest, AngledObject)
{
  goto_folder_filepath("test/perception_output/regular_object/angled_left", "grasp_planning");

  epd_msgs::msg::EPDObjectLocalization::SharedPtr msg = LoadPerception();

  cv::Mat depth_img = load_depth_img();
  std::vector<float> angles;
  for (int i = 0; i < static_cast<int>(msg->num_objects); i++) {
    double yaw, pitch, roll;
    tf2::Matrix3x3(
      tf2::Quaternion(
        msg->objects[i].pos.pose.orientation.x,
        msg->objects[i].pos.pose.orientation.y,
        msg->objects[i].pos.pose.orientation.z,
        msg->objects[i].pos.pose.orientation.w))
    .getEulerYPR(yaw, pitch, roll);

    angles.push_back(yaw);
  }
  Msg message(msg, angles);
  TwoFinger grasp = load_finger_attributes(
    "attributes.yaml",
    "test/test_yaml",
    "grasp_planning");
  grasp.get_checkpoints(message);

  EXPECT_EQ(1, static_cast<int>(grasp.gradient_vector.size()));
  EXPECT_EQ(1, static_cast<int>(grasp.c1_vector.size()));
  EXPECT_EQ(1, static_cast<int>(grasp.c2_vector.size()));

  EXPECT_GT(grasp.checkpoint_startx.size(), 0);
  EXPECT_GT(grasp.checkpoint_starty.size(), 0);
  EXPECT_GT(grasp.checkpoint_endx.size(), 0);
  EXPECT_GT(grasp.checkpoint_endy.size(), 0);

  std::vector<std::vector<int>> test_points = grasp.find_2_points({374, 376}, {242, 161}, 0);
  EXPECT_NEAR(405, test_points[0][0], 3);
  EXPECT_NEAR(351, test_points[0][1], 3);
  EXPECT_NEAR(210, test_points[1][0], 3);
  EXPECT_NEAR(186, test_points[1][1], 3);

  int valid_grasp = grasp.get_gdi_value(
    depth_img,
    {308, 269},
    {374, 376},
    {242, 161},
    {405, 351},
    {211, 186});
  EXPECT_EQ(valid_grasp, 0);

  int valid_corner = grasp.get_corner_gdi({227, 174}, 40, depth_img, 500);
  EXPECT_GT(valid_corner, 0);
}

TEST(SuctionGripperTest, EmptySuction)
{
  SuctionCupArray empty_suction;

  EXPECT_EQ(0, empty_suction.gdi);
  EXPECT_EQ(0, empty_suction.length_cup_num);
  EXPECT_EQ(0, empty_suction.breadth_cup_num);
  EXPECT_EQ(0, empty_suction.radius);
  EXPECT_EQ(0, empty_suction.table_height);
  EXPECT_EQ(0, empty_suction.cup_area);
  EXPECT_EQ(0, empty_suction.array_area);
}
TEST(SuctionGripperTest, HorizontalObject)
{
  goto_folder_filepath("test/perception_output/regular_object/horizontal", "grasp_planning");
  epd_msgs::msg::EPDObjectLocalization::SharedPtr msg = LoadPerception();

  cv::Mat depth_img = load_depth_img();
  std::vector<float> angles;
  for (int i = 0; i < static_cast<int>(msg->num_objects); i++) {
    double yaw, pitch, roll;
    tf2::Matrix3x3(
      tf2::Quaternion(
        msg->objects[i].pos.pose.orientation.x,
        msg->objects[i].pos.pose.orientation.y,
        msg->objects[i].pos.pose.orientation.z,
        msg->objects[i].pos.pose.orientation.w))
    .getEulerYPR(yaw, pitch, roll);
    angles.push_back(yaw);
  }
  Msg message(msg, angles);
  SuctionCupArray grasp = load_suction_attributes(
    "suction.yaml",
    "test/test_yaml",
    "grasp_planning");
  EXPECT_EQ(0, grasp.get_cup_gdi(message, depth_img, {100, 100}, 0));
  EXPECT_GT(grasp.get_cup_gdi(message, depth_img, {332, 235}, 0), 0);
}

TEST(SuctionGripperTest, GraspQualityCheck)
{
  goto_folder_filepath("test/perception_output/regular_object/horizontal", "grasp_planning");

  epd_msgs::msg::EPDObjectLocalization::SharedPtr msg = LoadPerception();

  cv::Mat depth_img = load_depth_img();
  std::vector<float> angles;
  for (int i = 0; i < static_cast<int>(msg->num_objects); i++) {
    double yaw, pitch, roll;
    tf2::Matrix3x3(
      tf2::Quaternion(
        msg->objects[i].pos.pose.orientation.x,
        msg->objects[i].pos.pose.orientation.y,
        msg->objects[i].pos.pose.orientation.z,
        msg->objects[i].pos.pose.orientation.w))
    .getEulerYPR(yaw, pitch, roll);
    angles.push_back(yaw);
  }
  Msg message(msg, angles);
  SuctionCupArray grasp = load_suction_attributes(
    "suction.yaml",
    "test/test_yaml",
    "grasp_planning");
  grasp.radius = length_to_pixel(
    grasp.radius, depth_img.at<ushort>(
      message.center_y[0],
      message.center_x[0]),
    msg->camera_info);

  EXPECT_EQ(0, grasp.get_cup_gdi(message, depth_img, {100, 100}, 0));
  EXPECT_GT(grasp.get_cup_gdi(message, depth_img, {332, 235}, 0), 0);
  grasp.get_best_grasp(message, depth_img, 0);

  EXPECT_GE(grasp.chosen_grasp[0], 229);
  EXPECT_LE(grasp.chosen_grasp[0], 434);
  EXPECT_GE(grasp.chosen_grasp[1], 162);
  EXPECT_LE(grasp.chosen_grasp[1], 307);
}
