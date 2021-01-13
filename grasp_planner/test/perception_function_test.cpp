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
#include "perception_functions.hpp"
#include "load_perception.hpp"

TEST(PerceptionFunctionTest, PixelToReal)
{
  goto_folder_filepath("test/perception_output/regular_object/horizontal", "grasp_planning");

  epd_msgs::msg::EPDObjectLocalization::SharedPtr msg = LoadPerception();

  cv::Mat depth_img = load_depth_img();
  cv::Point2f grasp_point_pixel(217, 154);
  cv::Point2f grasp_point_real = pixel_to_real(grasp_point_pixel, msg->camera_info, depth_img);
  EXPECT_NEAR(-0.101018, grasp_point_real.x, 0.05);
  EXPECT_NEAR(-0.0774468, grasp_point_real.y, 0.05);
}

TEST(PerceptionFunctionTest, LengthToPixel)
{
  goto_folder_filepath("test/perception_output/regular_object/horizontal", "grasp_planning");

  epd_msgs::msg::EPDObjectLocalization::SharedPtr msg = LoadPerception();

  cv::Mat depth_img = load_depth_img();
  float test_length = length_to_pixel(40, depth_img.at<ushort>(217, 154), msg->camera_info);
  EXPECT_NEAR(42, test_length, 1);
}
