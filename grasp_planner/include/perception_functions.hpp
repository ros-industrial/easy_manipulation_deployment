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

#ifndef PERCEPTION_FUNCTIONS_HPP_
#define PERCEPTION_FUNCTIONS_HPP_
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"


/**
 * Function that converts pixel coordinates to real coordinates with respect to the camera axis
 */
cv::Point2f pixel_to_real(
  cv::Point2f pixel_coordinate,
  const sensor_msgs::msg::CameraInfo camera_info,
  const cv::Mat & depthImg)
{
  cv::Point2f temp;
  auto ppx = camera_info.k.at(2);
  auto fx = camera_info.k.at(0);
  auto ppy = camera_info.k.at(5);
  auto fy = camera_info.k.at(4);

  // auto depth = depthImg.at<unsigned short>(pixel_coordinate) * 0.001;
  auto depth = depthImg.at<unsigned short>(pixel_coordinate) * 0.001;    // NOLINT
  temp.x = (pixel_coordinate.x - ppx) / fx * depth;
  temp.y = (pixel_coordinate.y - ppy) / fy * depth;
  return temp;
}

/**
 * Function that converts pixel length to real length
 */
float length_to_pixel(
  float distance,
  const float depth,
  const sensor_msgs::msg::CameraInfo camera_info)
{
  auto fx = camera_info.k.at(0);
  return distance / depth * fx;
}

#endif  // PERCEPTION_FUNCTIONS_HPP_
