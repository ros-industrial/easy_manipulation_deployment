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
#include <vector>
#include <memory>
#include "msg.hpp"

epd_msgs::msg::EPDObjectLocalization::SharedPtr
generate_mock_perception()
{
  sensor_msgs::msg::Image input_image;

  input_image.width = 1280;
  input_image.height = 720;

  std::vector<sensor_msgs::msg::RegionOfInterest> roi_list;
  sensor_msgs::msg::RegionOfInterest roi;
  roi.height = 40;
  roi.width = 50;
  roi.x_offset = 20;
  roi.y_offset = 30;
  roi_list.push_back(roi);
  epd_msgs::msg::EPDObjectLocalization mock_perception;
  mock_perception.num_objects = 1;
  mock_perception.roi_array = roi_list;
  mock_perception.depth_image = input_image;

  std::shared_ptr<epd_msgs::msg::EPDObjectLocalization> mock_perception_ptr =
    std::make_shared<epd_msgs::msg::EPDObjectLocalization>(mock_perception);

  return mock_perception_ptr;
}

epd_msgs::msg::EPDObjectLocalization::SharedPtr
generate_mock_empty_perception()
{
  sensor_msgs::msg::Image input_image;

  input_image.width = 1280;
  input_image.height = 720;

  std::vector<sensor_msgs::msg::RegionOfInterest> roi_list;
  sensor_msgs::msg::RegionOfInterest roi;
  roi.height = 40;
  roi.width = 50;
  roi.x_offset = 20;
  roi.y_offset = 30;
  roi_list.push_back(roi);
  epd_msgs::msg::EPDObjectLocalization mock_perception;
  mock_perception.num_objects = 0;
  mock_perception.roi_array = roi_list;
  mock_perception.depth_image = input_image;

  std::shared_ptr<epd_msgs::msg::EPDObjectLocalization> mock_perception_ptr =
    std::make_shared<epd_msgs::msg::EPDObjectLocalization>(mock_perception);

  return mock_perception_ptr;
}

TEST(MsgTest, VarTest)
{
  Msg result(generate_mock_perception(), {1.5708});

  EXPECT_TRUE(static_cast<int>(result.bb_height.size()) == 1);
  EXPECT_TRUE(static_cast<int>(result.bb_width.size()) == 1);
  EXPECT_TRUE(static_cast<int>(result.tl_x.size()) == 1);
  EXPECT_TRUE(static_cast<int>(result.tl_y.size()) == 1);
  EXPECT_TRUE(static_cast<int>(result.tr_x.size()) == 1);
  EXPECT_TRUE(static_cast<int>(result.tr_y.size()) == 1);
  EXPECT_TRUE(static_cast<int>(result.br_x.size()) == 1);
  EXPECT_TRUE(static_cast<int>(result.br_y.size()) == 1);
  EXPECT_TRUE(static_cast<int>(result.bl_x.size()) == 1);
  EXPECT_TRUE(static_cast<int>(result.bl_y.size()) == 1);
  EXPECT_TRUE(static_cast<int>(result.center_x.size()) == 1);
  EXPECT_TRUE(static_cast<int>(result.center_y.size()) == 1);
  EXPECT_TRUE(static_cast<int>(result.angle.size()) == 1);

  EXPECT_EQ(result.tl_x[0], 20);
  EXPECT_EQ(result.tl_y[0], 30);
  EXPECT_EQ(result.tr_x[0], 70);
  EXPECT_EQ(result.tr_y[0], 30);

  EXPECT_EQ(result.br_x[0], 70);
  EXPECT_EQ(result.br_y[0], 70);
  EXPECT_EQ(result.bl_x[0], 20);
  EXPECT_EQ(result.bl_y[0], 70);
  EXPECT_EQ(result.center_x[0], 45);
  EXPECT_EQ(result.center_y[0], 50);

  EXPECT_EQ(result.bb_height[0], 40);
  EXPECT_EQ(result.bb_width[0], 50);
  EXPECT_EQ(result.img_height, 720);
  EXPECT_EQ(result.img_width, 1280);
  EXPECT_EQ(result.num_objects, 1);
  EXPECT_NEAR(result.angle[0], 1.5708, 0.01);
}

TEST(MsgTest, NegAngleTest)
{
  Msg result(generate_mock_perception(), {-0.567});

  EXPECT_NEAR(result.angle[0], 2.575, 0.01);
}

TEST(MsgTest, LargeAngleTest)
{
  Msg result(generate_mock_perception(), {6.44});

  EXPECT_NEAR(result.angle[0], 0.157, 0.01);
}

TEST(MsgTest, NoObjTest)
{
  Msg result(generate_mock_empty_perception(), {2, 4});

  EXPECT_TRUE(result.bb_width.empty());
  EXPECT_TRUE(result.bb_height.empty());
  EXPECT_TRUE(static_cast<int>(result.tl_x.size()) == 0);
  EXPECT_TRUE(static_cast<int>(result.tl_y.size()) == 0);
  EXPECT_TRUE(static_cast<int>(result.tr_x.size()) == 0);
  EXPECT_TRUE(static_cast<int>(result.tr_y.size()) == 0);
  EXPECT_TRUE(static_cast<int>(result.br_x.size()) == 0);
  EXPECT_TRUE(static_cast<int>(result.br_y.size()) == 0);
  EXPECT_TRUE(static_cast<int>(result.bl_x.size()) == 0);
  EXPECT_TRUE(static_cast<int>(result.bl_y.size()) == 0);
  EXPECT_TRUE(static_cast<int>(result.center_x.size()) == 0);
  EXPECT_TRUE(static_cast<int>(result.center_y.size()) == 0);
  EXPECT_TRUE(static_cast<int>(result.angle.size()) == 0);

  EXPECT_EQ(result.img_height, 720);
  EXPECT_EQ(result.img_width, 1280);
  EXPECT_EQ(result.num_objects, 0);
}
