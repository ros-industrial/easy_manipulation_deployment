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

#ifndef PLANNING_FUNCTIONS_HPP_
#define PLANNING_FUNCTIONS_HPP_

#include <boost/filesystem.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include "cmath"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "yaml-cpp/yaml.h"

float PI = 3.14159265;
/**
 * Calculate length of two points in 2D space
 */
float length(int x1, int y1, int x2, int y2)
{
  return sqrt(pow(static_cast<float>(x1 - x2), 2) + pow(static_cast<float>(y1 - y2), 2));
}

/**
 * Checks if a point is in a circle.
 */
bool in_circle(float radius, std::vector<int> centre, std::vector<int> point)
{
  if (pow((point[0] - centre[0]), 2) + pow((point[1] - centre[1]), 2) - pow(radius, 2) < 0) {
    return true;
  } else {
    return false;
  }
}

/**
 * Quadratic equation solver
 */
std::vector<float> quadratic_equation(float a, float b, float c)
{
  std::vector<float> result;
  float discriminant = pow(b, 2) - (4 * a * c);
  if (discriminant < 0) {
    return result;
  } else if (discriminant == 0) {
    result.push_back(-b / (2 * a));
  } else {
    result.push_back(((-b + sqrt(discriminant)) / (2 * a)));
    result.push_back(((-b - sqrt(discriminant)) / (2 * a)));
  }
  return result;
}

/**
 * Get the intersecting coordinates between a circle and a line
 */
std::vector<std::vector<int>> circle_line_intersect(
  float gradient,
  float intersect,
  float radius,
  std::vector<float> circle_centre)
{
  std::vector<std::vector<int>> result;

  if (circle_centre.size() == 2 && radius > 0) {
    // Get the quadratic equation to determine the x coordinates of point
    float a = 1 + pow(gradient, 2);
    float b = 2 * ((gradient * (intersect - circle_centre[1])) - circle_centre[0]);
    float c = pow(circle_centre[0], 2) +
      pow((intersect - circle_centre[1]), 2) - pow(radius, 2);

    std::vector<float> x_coords = quadratic_equation(a, b, c);

    // Solve the equation to get the x coordinates
    if (static_cast<int>(x_coords.size()) == 2) {
      for (int i = 0; i < static_cast<int>(x_coords.size()); i++) {
        std::vector<int> temp_coords {static_cast<int>(round(x_coords[i])),
          static_cast<int>(round(gradient * x_coords[i] + intersect))};
        result.push_back(temp_coords);
      }
    } else {
      return result;
    }
  }

  return result;
}

/**
 * Get a bounding box based on coodinate centre and radius of box area.
 */

std::vector<std::vector<int>> get_border_corners(std::vector<int> centre, float radius)
{
  std::vector<std::vector<int>> result;
  if (centre.size() == 2 && radius > 0 && centre[0] >= 0 && centre[1] >= 0) {
    std::vector<int> top_left;
    std::vector<int> bottom_right;
    if (centre[0] - radius < 0) {
      top_left.push_back(0);
    } else {
      top_left.push_back(static_cast<int>(round(centre[0] - radius)));
    }
    if (centre[1] - radius < 0) {
      top_left.push_back(0);
    } else {
      top_left.push_back(static_cast<int>(round(centre[1] - radius)));
    }
    bottom_right = {static_cast<int>(round(centre[0] + radius)),
      static_cast<int>(round(centre[1] + radius))};
    result = {top_left, bottom_right};
  } else {
    return result;
  }
  return result;
}

/**
 * Make sure input is within 0 to Pi
 */

float keep_angle_in_bounds(float input)
{
  if (input < 0 || input >= PI) {
    if (input < 0) {
      while (input < 0) {
        input += PI;
      }
    }
    if (input >= PI) {
      while (input >= PI) {
        input -= PI;
      }
    }
  }
  return input;
}

#endif  // PLANNING_FUNCTIONS_HPP_
