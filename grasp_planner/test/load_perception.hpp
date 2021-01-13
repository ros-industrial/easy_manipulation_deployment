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

#ifndef LOAD_PERCEPTION_HPP_
#define LOAD_PERCEPTION_HPP_

#include <grasp_planning/msg/grasp_pose.hpp>
#include <epd_msgs/msg/epd_object_localization.hpp>
#include <boost/filesystem.hpp>
#include <memory>
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"
#include "perception_functions.hpp"
#include "rclcpp/rclcpp.hpp"


using std::placeholders::_1;

epd_msgs::msg::EPDObjectLocalization::SharedPtr
LoadPerception()
{
  epd_msgs::msg::EPDObjectLocalization mock_perception;

  YAML::Node yaml;

  // Load Yaml File.
  yaml = YAML::LoadFile("perception_output.yaml");

  // TODO(Glenn) : add error catch if trying to create directory with same name,
  // or maybe add a number to it, eg table1, table2.
  for (YAML::iterator it = yaml.begin(); it != yaml.end(); ++it) {
    std::string key = it->first.as<std::string>();
    if (key.compare("objects") == 0) {
      YAML::Node object_array = it->second;
      for (YAML::iterator object_array_it = object_array.begin();
        object_array_it != object_array.end();
        ++object_array_it)
      {
        epd_msgs::msg::LocalizedObject object;
        object.name = object_array_it->first.as<std::string>();
        YAML::Node curr_obj = object_array_it->second;
        for (YAML::iterator curr_obj_it = curr_obj.begin();
          curr_obj_it != curr_obj.end();
          ++curr_obj_it)
        {
          if (curr_obj_it->first.as<std::string>().compare("pos") == 0) {
            YAML::Node pose = curr_obj_it->second;
            for (YAML::iterator pose_it = pose.begin();
              pose_it != pose.end();
              ++pose_it)
            {
              if (pose_it->first.as<std::string>().compare("position") == 0) {
                YAML::Node position = pose_it->second;
                for (YAML::iterator position_it = position.begin();
                  position_it != position.end();
                  ++position_it)
                {
                  if (position_it->first.as<std::string>().compare("x") == 0) {
                    object.pos.pose.position.x = position_it->second.as<float>();
                  }
                  if (position_it->first.as<std::string>().compare("y") == 0) {
                    object.pos.pose.position.y = position_it->second.as<float>();
                  }
                  if (position_it->first.as<std::string>().compare("z") == 0) {
                    object.pos.pose.position.z = position_it->second.as<float>();
                  }
                }
              }
              if (pose_it->first.as<std::string>().compare("orientation") == 0) {
                YAML::Node orientation = pose_it->second;
                for (YAML::iterator orientation_it = orientation.begin();
                  orientation_it != orientation.end();
                  ++orientation_it)
                {
                  if (orientation_it->first.as<std::string>().compare("x") == 0) {
                    object.pos.pose.orientation.x = orientation_it->second.as<float>();
                  }
                  if (orientation_it->first.as<std::string>().compare("y") == 0) {
                    object.pos.pose.orientation.y = orientation_it->second.as<float>();
                  }
                  if (orientation_it->first.as<std::string>().compare("z") == 0) {
                    object.pos.pose.orientation.z = orientation_it->second.as<float>();
                  }
                  if (orientation_it->first.as<std::string>().compare("w") == 0) {
                    object.pos.pose.orientation.w = orientation_it->second.as<float>();
                  }
                }
              }
            }
          }

          if (curr_obj_it->first.as<std::string>().compare("roi") == 0) {
            YAML::Node roi = curr_obj_it->second;
            for (YAML::iterator roi_it = roi.begin(); roi_it != roi.end(); ++roi_it) {
              if (roi_it->first.as<std::string>().compare("x_offset") == 0) {
                object.roi.x_offset = roi_it->second.as<float>();
              }
              if (roi_it->first.as<std::string>().compare("y_offset") == 0) {
                object.roi.y_offset = roi_it->second.as<float>();
              }
              if (roi_it->first.as<std::string>().compare("height") == 0) {
                object.roi.height = roi_it->second.as<float>();
              }
              if (roi_it->first.as<std::string>().compare("width") == 0) {
                object.roi.width = roi_it->second.as<float>();
              }
            }
          }
          if (curr_obj_it->first.as<std::string>().compare("length") == 0) {
            object.length = curr_obj_it->second.as<float>();
          }
          if (curr_obj_it->first.as<std::string>().compare("breadth") == 0) {
            object.breadth = curr_obj_it->second.as<float>();
          }
          if (curr_obj_it->first.as<std::string>().compare("height") == 0) {
            object.height = curr_obj_it->second.as<float>();
          }
        }
        mock_perception.objects.push_back(object);
      }
    }
    if (key.compare("frame_width") == 0) {
      mock_perception.frame_width = it->second.as<float>();
    }
    if (key.compare("frame_height") == 0) {
      mock_perception.frame_height = it->second.as<float>();
    }

    if (key.compare("num_objects") == 0) {
      mock_perception.num_objects = it->second.as<float>();
    }

    if (key.compare("camera_info") == 0) {
      YAML::Node cam_info = it->second;
      sensor_msgs::msg::CameraInfo camera_info;
      for (YAML::iterator cam_info_it = cam_info.begin();
        cam_info_it != cam_info.end();
        ++cam_info_it)
      {
        if (cam_info_it->first.as<std::string>().compare("D") == 0) {
          YAML::Node d = cam_info_it->second;
          std::vector<double> d_input;
          for (YAML::iterator d_it = d.begin(); d_it != d.end(); ++d_it) {
            d_input.push_back(d_it->second.as<double>());
          }
          camera_info.d = d_input;
        }

        if (cam_info_it->first.as<std::string>().compare("K") == 0) {
          YAML::Node k = cam_info_it->second;
          std::array<double, 9> k_input;
          for (YAML::iterator k_it = k.begin(); k_it != k.end(); ++k_it) {
            std::string name = k_it->first.as<std::string>();
            k_input[name.back() - '0'] = k_it->second.as<double>();
          }
          camera_info.k = k_input;
        }
        if (cam_info_it->first.as<std::string>().compare("R") == 0) {
          YAML::Node r = cam_info_it->second;
          std::array<double, 9> r_input;
          for (YAML::iterator r_it = r.begin(); r_it != r.end(); ++r_it) {
            std::string name = r_it->first.as<std::string>();
            r_input[name.back() - '0'] = r_it->second.as<double>();
          }
          camera_info.r = r_input;
        }
        if (cam_info_it->first.as<std::string>().compare("P") == 0) {
          YAML::Node p = cam_info_it->second;
          std::array<double, 12> p_input;
          for (YAML::iterator p_it = p.begin(); p_it != p.end(); ++p_it) {
            std::string name = p_it->first.as<std::string>();
            p_input[name.back() - '0'] = p_it->second.as<double>();
          }
          camera_info.p = p_input;
        }
      }
      mock_perception.camera_info = camera_info;
    }

    if (key.compare("roi_array") == 0) {
      YAML::Node roi_array = it->second;
      for (YAML::iterator roi_array_it = roi_array.begin();
        roi_array_it != roi_array.end();
        ++roi_array_it)
      {
        YAML::Node roi = roi_array_it->second;
        sensor_msgs::msg::RegionOfInterest temp_roi;
        for (YAML::iterator roi_it = roi.begin(); roi_it != roi.end(); ++roi_it) {
          if (roi_it->first.as<std::string>().compare("x_offset") == 0) {
            temp_roi.x_offset = roi_it->second.as<float>();
          }
          if (roi_it->first.as<std::string>().compare("y_offset") == 0) {
            temp_roi.y_offset = roi_it->second.as<float>();
          }
          if (roi_it->first.as<std::string>().compare("height") == 0) {
            temp_roi.height = roi_it->second.as<float>();
          }
          if (roi_it->first.as<std::string>().compare("width") == 0) {
            temp_roi.width = roi_it->second.as<float>();
          }
        }
        mock_perception.roi_array.push_back(temp_roi);
      }
    }

    if (key.compare("depth_image") == 0) {
      YAML::Node depth_image = it->second;
      sensor_msgs::msg::Image input_image;
      for (YAML::iterator depth_image_it = depth_image.begin();
        depth_image_it != depth_image.end();
        ++depth_image_it)
      {
        if (depth_image_it->first.as<std::string>().compare("height") == 0) {
          input_image.height = depth_image_it->second.as<float>();
        }

        if (depth_image_it->first.as<std::string>().compare("width") == 0) {
          input_image.width = depth_image_it->second.as<float>();
        }
      }
      mock_perception.depth_image = input_image;
    }
  }
  std::shared_ptr<epd_msgs::msg::EPDObjectLocalization> mock_perception_ptr =
    std::make_shared<epd_msgs::msg::EPDObjectLocalization>(mock_perception);
  return mock_perception_ptr;
}

cv::Mat
load_depth_img()
{
  cv::Mat depth_img(cv::Size(640, 480), 2);

  std::ifstream infile("depth_image.yaml");
  int counter_width = 0;
  int counter_height = 0;
  std::string line;

  while (std::getline(infile, line)) {
    counter_width = 0;
    std::istringstream ss(line);
    std::string depth_val;
    while (std::getline(ss, depth_val, ',')) {
      depth_img.at<ushort>(counter_width, counter_height) = std::stoi(depth_val);
      counter_width++;
    }
    counter_height++;
  }

  return depth_img;
}

#endif  // LOAD_PERCEPTION_HPP_
