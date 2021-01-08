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

#ifndef MSG_HPP_
#define MSG_HPP_

#include <epd_msgs/msg/epd_object_localization.hpp>

#include <iostream>
#include <vector>

#include "planning_functions.hpp"
class Msg
{
public:
  /*! \brief Bounding box height */
  std::vector<float> bb_height;
  /*! \brief Bounding box width */
  std::vector<float> bb_width;
  /*! \brief Bounding box center (x) */
  std::vector<float> center_x;
  /*! \brief Bounding box center (y) */
  std::vector<float> center_y;

  /*! \brief Bounding box Top left corner (x) */
  std::vector<float> tl_x;
  /*! \brief Bounding box Top left corner (y) */
  std::vector<float> tl_y;
  /*! \brief Bounding box Top right corner (x) */
  std::vector<float> tr_x;
  /*! \brief Bounding box Top right corner (y) */
  std::vector<float> tr_y;
  /*! \brief Bounding box Bottom right corner (x) */
  std::vector<float> br_x;
  /*! \brief Bounding box Bottom right corner (y) */
  std::vector<float> br_y;
  /*! \brief Bounding box Bottom left corner (x) */
  std::vector<float> bl_x;
  /*! \brief Bounding box Bottom left corner (y) */
  std::vector<float> bl_y;
  /*! \brief Angle of the OBJECT wrt horizontal image plane. */
  std::vector<float> angle;
  /*! \brief Image width in pixels */
  float img_width;
  /*! \brief Image width in pixels */
  float img_height;
  /*! \brief Number of objects  */
  int num_objects;

  /**
   * Class constructor taking in percpeption message type
   */
  Msg(const epd_msgs::msg::EPDObjectLocalization::SharedPtr msg, std::vector<float> angle_)
  // Msg(const grasp_planning::msg::RectOutput::SharedPtr msg, std::vector<float> angle_)
  {
    img_width = msg->depth_image.width;
    img_height = msg->depth_image.height;
    num_objects = msg->num_objects;
    for (int i = 0; i < static_cast<int>(msg->num_objects); i++) {
      bb_height.push_back(msg->roi_array[i].height);
      bb_width.push_back(msg->roi_array[i].width);

      center_x.push_back(msg->roi_array[i].x_offset + round(msg->roi_array[i].width / 2));
      center_y.push_back(msg->roi_array[i].y_offset + round(msg->roi_array[i].height / 2));

      tl_x.push_back(msg->roi_array[i].x_offset);
      tl_y.push_back(msg->roi_array[i].y_offset);

      tr_x.push_back(msg->roi_array[i].x_offset + msg->roi_array[i].width);
      tr_y.push_back(msg->roi_array[i].y_offset);

      br_x.push_back(msg->roi_array[i].x_offset + msg->roi_array[i].width);
      br_y.push_back(msg->roi_array[i].y_offset + msg->roi_array[i].height);

      bl_x.push_back(msg->roi_array[i].x_offset);
      bl_y.push_back(msg->roi_array[i].y_offset + msg->roi_array[i].height);
      angle.push_back(keep_angle_in_bounds(angle_[i]));
    }
  }
};


#endif  // MSG_HPP_
