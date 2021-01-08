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

#ifndef PERCEPTION_SAVE_HPP_
#define PERCEPTION_SAVE_HPP_

#include <grasp_planning/msg/grasp_pose.hpp>
#include <epd_msgs/msg/epd_object_localization.hpp>
#include <boost/filesystem.hpp>
#include <memory>
#include "yaml-cpp/yaml.h"
#include "perception_functions.hpp"
#include "load_perception.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

class PerceptionSaver : public rclcpp::Node
{
public:
  rclcpp::Subscription<epd_msgs::msg::EPDObjectLocalization>::SharedPtr perception_sub;

  PerceptionSaver()
  : Node("perception_saver")
  {
    perception_sub = this->create_subscription<epd_msgs::msg::EPDObjectLocalization>(
      "/perception/output", 10, std::bind(&PerceptionSaver::planning_init, this, _1));
    std::cout << "Waiting for topic for perception saver...." << std::endl;
  }

  void
  planning_init(const epd_msgs::msg::EPDObjectLocalization::SharedPtr msg) const
  {
    YAML::Emitter out;

    out << YAML::BeginMap;
    out << YAML::Key << "header";
    out << YAML::Value;
    out << YAML::BeginMap;
    out << YAML::Key << "stamp";
    out << YAML::Value;
    out << YAML::BeginMap;
    out << YAML::Key << "stamp.sec";
    out << YAML::Value << std::to_string(msg->header.stamp.sec);
    out << YAML::Key << "stamp.nanosec";
    out << YAML::Value << std::to_string(msg->header.stamp.nanosec);
    out << YAML::EndMap;
    out << YAML::Key << "frame_id";
    out << YAML::Value << msg->header.frame_id;
    out << YAML::EndMap;
    out << YAML::Key << "objects";
    out << YAML::Value;
    out << YAML::BeginMap;

    for (int object = 0; object < static_cast<int>(msg->objects.size()); object++) {
      out << YAML::Key << msg->objects[object].name;
      out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "pos";
      out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "position";
      out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "x";
      out << YAML::Value << std::to_string(msg->objects[object].pos.pose.position.x);
      out << YAML::Key << "y";
      out << YAML::Value << std::to_string(msg->objects[object].pos.pose.position.y);
      out << YAML::Key << "z";
      out << YAML::Value << std::to_string(msg->objects[object].pos.pose.position.z);
      out << YAML::EndMap;
      out << YAML::Key << "orientation";
      out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "x";
      out << YAML::Value << std::to_string(msg->objects[object].pos.pose.orientation.x);
      out << YAML::Key << "y";
      out << YAML::Value << std::to_string(msg->objects[object].pos.pose.orientation.y);
      out << YAML::Key << "z";
      out << YAML::Value << std::to_string(msg->objects[object].pos.pose.orientation.z);
      out << YAML::Key << "w";
      out << YAML::Value << std::to_string(msg->objects[object].pos.pose.orientation.w);
      out << YAML::EndMap;
      out << YAML::EndMap;
      out << YAML::Key << "roi";
      out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "x_offset";
      out << YAML::Value << std::to_string(msg->objects[object].roi.x_offset);
      out << YAML::Key << "y_offset";
      out << YAML::Value << std::to_string(msg->objects[object].roi.y_offset);
      out << YAML::Key << "height";
      out << YAML::Value << std::to_string(msg->objects[object].roi.height);
      out << YAML::Key << "width";
      out << YAML::Value << std::to_string(msg->objects[object].roi.width);
      out << YAML::Key << "do_rectify";
      out << YAML::Value << std::to_string(msg->objects[object].roi.do_rectify);
      out << YAML::EndMap;


      out << YAML::Key << "breadth";
      out << YAML::Value << std::to_string(msg->objects[object].breadth);
      out << YAML::Key << "length";
      out << YAML::Value << std::to_string(msg->objects[object].length);
      out << YAML::Key << "height";
      out << YAML::Value << std::to_string(msg->objects[object].height);
      out << YAML::EndMap;
    }
    out << YAML::EndMap;

    out << YAML::Key << "frame_width";
    out << YAML::Value << std::to_string(msg->frame_width);

    out << YAML::Key << "frame_height";
    out << YAML::Value << std::to_string(msg->frame_height);

    out << YAML::Key << "num_objects";
    out << YAML::Value << std::to_string(msg->num_objects);

    out << YAML::Key << "camera_info";
    out << YAML::Value;
    out << YAML::BeginMap;

    out << YAML::Key << "header";
    out << YAML::Value;
    out << YAML::BeginMap;
    out << YAML::Key << "stamp";
    out << YAML::Value;
    out << YAML::BeginMap;
    out << YAML::Key << "stamp.sec";
    out << YAML::Value << std::to_string(msg->camera_info.header.stamp.sec);
    out << YAML::Key << "stamp.nanosec";
    out << YAML::Value << std::to_string(msg->camera_info.header.stamp.nanosec);
    out << YAML::EndMap;
    out << YAML::Key << "frame_id";
    out << YAML::Value << msg->camera_info.header.frame_id;
    out << YAML::EndMap;

    out << YAML::Key << "height";
    out << YAML::Value << std::to_string(msg->camera_info.height);

    out << YAML::Key << "width";
    out << YAML::Value << std::to_string(msg->camera_info.width);

    out << YAML::Key << "distortion_model";
    out << YAML::Value << msg->camera_info.distortion_model;

    out << YAML::Key << "D";
    out << YAML::Value;
    out << YAML::BeginMap;
    for (int d = 0; d < static_cast<int>(msg->camera_info.d.size()); d++) {
      out << YAML::Key << "d_" + std::to_string(d);
      out << YAML::Value << std::to_string(msg->camera_info.d[d]);
    }
    out << YAML::EndMap;

    out << YAML::Key << "K";
    out << YAML::Value;
    out << YAML::BeginMap;
    for (int k = 0; k < static_cast<int>(msg->camera_info.k.size()); k++) {
      out << YAML::Key << "k_" + std::to_string(k);
      out << YAML::Value << std::to_string(msg->camera_info.k[k]);
    }
    out << YAML::EndMap;

    out << YAML::Key << "R";
    out << YAML::Value;
    out << YAML::BeginMap;
    for (int r = 0; r < static_cast<int>(msg->camera_info.r.size()); r++) {
      out << YAML::Key << "r_" + std::to_string(r);
      out << YAML::Value << std::to_string(msg->camera_info.r[r]);
    }
    out << YAML::EndMap;

    out << YAML::Key << "P";
    out << YAML::Value;
    out << YAML::BeginMap;
    for (int p = 0; p < static_cast<int>(msg->camera_info.p.size()); p++) {
      out << YAML::Key << "p_" + std::to_string(p);
      out << YAML::Value << std::to_string(msg->camera_info.p[p]);
    }
    out << YAML::EndMap;

    out << YAML::Key << "binning_x";
    out << YAML::Value << std::to_string(msg->camera_info.binning_x);

    out << YAML::Key << "binning_y";
    out << YAML::Value << std::to_string(msg->camera_info.binning_y);

    out << YAML::Key << "roi";
    out << YAML::Value;
    out << YAML::BeginMap;
    out << YAML::Key << "x_offset";
    out << YAML::Value << std::to_string(msg->camera_info.roi.x_offset);
    out << YAML::Key << "y_offset";
    out << YAML::Value << std::to_string(msg->camera_info.roi.y_offset);
    out << YAML::Key << "height";
    out << YAML::Value << std::to_string(msg->camera_info.roi.height);
    out << YAML::Key << "width";
    out << YAML::Value << std::to_string(msg->camera_info.roi.width);
    out << YAML::Key << "do_rectify";
    out << YAML::Value << std::to_string(msg->camera_info.roi.do_rectify);
    out << YAML::EndMap;

    out << YAML::EndMap;

    out << YAML::Key << "roi_array";
    out << YAML::Value;
    out << YAML::BeginMap;
    for (int roi = 0; roi < static_cast<int>(msg->roi_array.size()); roi++) {
      out << YAML::Key << "roi_" + std::to_string(roi);
      out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "x_offset";
      out << YAML::Value << std::to_string(msg->roi_array[roi].x_offset);
      out << YAML::Key << "y_offset";
      out << YAML::Value << std::to_string(msg->roi_array[roi].y_offset);
      out << YAML::Key << "height";
      out << YAML::Value << std::to_string(msg->roi_array[roi].height);
      out << YAML::Key << "width";
      out << YAML::Value << std::to_string(msg->roi_array[roi].width);
      out << YAML::Key << "do_rectify";
      out << YAML::Value << std::to_string(msg->roi_array[roi].do_rectify);
      out << YAML::EndMap;
    }
    out << YAML::EndMap;

    out << YAML::Key << "depth_image";
    out << YAML::Value;
    out << YAML::BeginMap;

    out << YAML::Key << "header";
    out << YAML::Value;
    out << YAML::BeginMap;
    out << YAML::Key << "stamp";
    out << YAML::Value;
    out << YAML::BeginMap;
    out << YAML::Key << "stamp.sec";
    out << YAML::Value << std::to_string(msg->depth_image.header.stamp.sec);
    out << YAML::Key << "stamp.nanosec";
    out << YAML::Value << std::to_string(msg->depth_image.header.stamp.nanosec);
    out << YAML::EndMap;
    out << YAML::Key << "frame_id";
    out << YAML::Value << msg->depth_image.header.frame_id;
    out << YAML::EndMap;

    out << YAML::Key << "width";
    out << YAML::Value << std::to_string(msg->depth_image.width);

    out << YAML::Key << "height";
    out << YAML::Value << std::to_string(msg->depth_image.height);

    out << YAML::Key << "encoding";
    out << YAML::Value << msg->depth_image.encoding;

    out << YAML::Key << "is_bigendian";
    out << YAML::Value << std::to_string(msg->depth_image.is_bigendian);

    out << YAML::Key << "step";
    out << YAML::Value << std::to_string(msg->depth_image.step);

    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream myfile;
    myfile.open("perception_output.yaml");
    myfile << out.c_str();
    myfile.close();

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg->depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth_img = cv_ptr->image;

    std::ofstream myfile2;
    myfile2.open("depth_image.yaml");
    for (int j = 0; j < static_cast<int>(msg->depth_image.width); j++) {
      for (int i = 0; i < static_cast<int>(msg->depth_image.height); i++) {
        myfile2 << depth_img.at<ushort>(i, j) << ",";
      }
      myfile2 << std::endl;
    }
    myfile2.close();
  }
};


#endif  // PERCEPTION_SAVE_HPP_
