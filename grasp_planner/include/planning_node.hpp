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

#ifndef PLANNING_NODE_HPP_
#define PLANNING_NODE_HPP_
#include <epd_msgs/msg/epd_object_localization.hpp>
#include <grasp_planning/msg/grasp_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>

#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "perception_functions.hpp"
#include "grasps.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ee_config.hpp"
#include "load_perception.hpp"
using std::placeholders::_1;

class GraspPlanNode : public rclcpp::Node
{
public:
  rclcpp::Publisher<grasp_planning::msg::GraspPose>::SharedPtr output_pub;
  rclcpp::Subscription<epd_msgs::msg::EPDObjectLocalization>::SharedPtr perception_sub;
  boost::filesystem::path workspace_path;

  GraspPlanNode()
  : Node("grasp_planning_node")
  {
    output_pub = this->create_publisher<grasp_planning::msg::GraspPose>("/grasp_poses", 10);
    perception_sub = this->create_subscription<epd_msgs::msg::EPDObjectLocalization>(
      "/processor/epd_localize_output", 10, std::bind(&GraspPlanNode::planning_init, this, _1));
    std::cout << "[easy_manipulation_deployment][Grasp Planner] "
      "Waiting for topic...." << std::endl;
    workspace_path = boost::filesystem::current_path();
  }

private:
  void planning_init(const epd_msgs::msg::EPDObjectLocalization::SharedPtr msg) const
  {
    if (msg->num_objects <= 0) {
      std::cout << "[easy_manipulation_deployment][Grasp Planner] No objects found!" << std::endl;
      return;
    }
    std::cout << "[easy_manipulation_deployment][Grasp Planner] Objects Detected!" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    float table_height = 0;
    // Populate angles from Subscribed messages
    std::vector<float> angles;
    for (int i = 0; i < static_cast<int>(msg->num_objects); i++) {
      double yaw, pitch, roll;
      tf2::Matrix3x3(
        tf2::Quaternion(
          msg->objects[i].pos.pose.orientation.x,
          msg->objects[i].pos.pose.orientation.y,
          msg->objects[i].pos.pose.orientation.z,
          msg->objects[i].pos.pose.orientation.w)).getEulerYPR(
        yaw, pitch, roll);
      angles.push_back(yaw);
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg->depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth_img = cv_ptr->image;
    Msg message(msg, angles);

    std::string gripper_type;
    try {
      gripper_type = get_ee_type("attributes.yaml", "config", "grasp_planning");
    } catch (const char * exp) {
      std::cout << exp << std::endl;
      return;
    }
    std::cout << "[easy_manipulation_deployment][Grasp Planner] "
      "Gripper Type Detected: " << gripper_type << std::endl;

    std::string output;
    std::vector<cv::Point2f> grasp_point_vector;
    if (gripper_type.compare("suction") == 0) {   // Suction Gripper
      std::cout << "[easy_manipulation_deployment][Grasp Planner] "
        "Load Suction Gripper" << std::endl;
      try {
        for (int j = 0; j < static_cast<int>(msg->num_objects); j++) {
          SuctionCupArray suction = load_suction_attributes(
            "attributes.yaml",
            "config",
            "grasp_planning");
          table_height = suction.table_height;
          suction.radius = length_to_pixel(
            suction.radius,
            depth_img.at<ushort>(
              message.center_y[j],
              message.center_x[j]), msg->camera_info);

          if (suction.radius > message.bb_width[j] ||
            suction.radius > message.bb_height[j])
          {
            std::cout << "[easy_manipulation_deployment][Grasp Planner] "
              "Not possible to grasp, item is too small for any "
              "suction cup to be grasped" << std::endl;
          } else {
            std::cout << "[easy_manipulation_deployment][Grasp Planner] "
              "Generating Suction Cup grasps" << std::endl;

            if (suction.get_best_grasp(message, depth_img, j)) {
              std::cout << "[easy_manipulation_deployment][Grasp Planner] "
                "Best Grasp found! GDI Score: " << suction.gdi << std::endl;

              // Store the coordinate of the object pixel in the text file,
              // and add it to the vector of object pixels
              output = std::to_string(suction.chosen_grasp[0]) + "," +
                std::to_string(suction.chosen_grasp[1]) + "," +
                std::to_string(static_cast<int>(suction.radius));

              cv::Point2f grasp_point_real;
              cv::Point2f grasp_point_pixel(suction.chosen_grasp[0],
                suction.chosen_grasp[1]);

              grasp_point_real = pixel_to_real(
                grasp_point_pixel,
                msg->camera_info, depth_img);

              grasp_point_vector.push_back(grasp_point_real);
            } else {
              std::cout << "[easy_manipulation_deployment][Grasp Planner] "
                "No Grasp Found" << std::endl;
            }
          }
        }
      } catch (YAML::BadFile & error) {
        std::cout << error.what();
        return;
      } catch (const char * exp) {
        std::cout << exp << std::endl;
        return;
      } catch (const boost::filesystem::filesystem_error & error) {
        std::cout << error.what() << std::endl;
        return;
      } catch (std::exception & error) {
        std::cout << error.what() << std::endl;
        return;
      }
    } else if (gripper_type.compare("finger") == 0) {      // Finger Gripper is loaded
      std::cout << " [easy_manipulation_deployment][Grasp Planner] "
        "Load Finger Gripper" << std::endl;
      try {
        TwoFinger grasp = load_finger_attributes(
          "attributes.yaml",
          "config", "grasp_planning");
        table_height = grasp.table_height;
        for (int j = 0; j < static_cast<int>(msg->num_objects); j++) {
          std::cout << " [easy_manipulation_deployment][Grasp Planner] "
            "Generating Finger grasps" << std::endl;
          grasp.distance_between_fingers = round(length_to_pixel(grasp.distance_between_fingers,
            depth_img.at<ushort>(
              message.center_y[j],
              message.center_x[j]), msg->camera_info));
          grasp.gripper_thickness = round(length_to_pixel(grasp.gripper_thickness,
            depth_img.at<ushort>(
              message.center_y[j],
              message.center_x[j]), msg->camera_info));
          grasp.target_length = sqrt(pow(grasp.distance_between_fingers, 2) + pow(grasp.gripper_thickness, 2));
          grasp.get_checkpoints(message);
          if (grasp.get_best_grasp(message, depth_img, j) > 0) {
            // Store the coordinate of the object pixel in the text file,
            // and add it to the vector of object pixels
            output = std::to_string(grasp.corner1[0]) + "," +
              std::to_string(grasp.corner1[1]) + "," +
              std::to_string(grasp.corner2[0]) + "," +
              std::to_string(grasp.corner2[1]) + "," +
              std::to_string(grasp.corner3[0]) + "," +
              std::to_string(grasp.corner3[1]) + "," +
              std::to_string(grasp.corner4[0]) + "," +
              std::to_string(grasp.corner4[1]);

            std::cout << "[easy_manipulation_deployment][Grasp Planner] "
              "Best Grasp found! GDI Score: " <<
              grasp.final_gdi << std::endl;
            cv::Point2f grasp_point_pixel(grasp.centre[0], grasp.centre[1]);

            cv::Point2f grasp_point_real = pixel_to_real(
              grasp_point_pixel,
              msg->camera_info, depth_img);

            grasp_point_vector.push_back(grasp_point_real);
          } else {
            std::cout << "[easy_manipulation_deployment][Grasp Planner] "
              "No Grasp Found" << std::endl;
          }
        }
      } catch (YAML::BadFile & error) {
        std::cout << error.what();
        return;
      } catch (const char * exp) {
        std::cout << exp << std::endl;
        return;
      } catch (const boost::filesystem::filesystem_error & error) {
        std::cout << error.what() << std::endl;
        return;
      } catch (std::exception & error) {
        std::cout << error.what() << std::endl;
        return;
      }
    } else {
      std::cout << "[easy_manipulation_deployment][Grasp Planner] Error: "
        "gripper type not specified" << std::endl;
      return;
    }

    std::cout << "[easy_manipulation_deployment][Grasp Planner] Generate Result Files... " <<
      std::endl;

    try {
      boost::filesystem::current_path(workspace_path);
      boost::filesystem::current_path(
        "src/easy_manipulation_deployment/"
        "grasp_validator/results");
      std::ofstream myfile;
      myfile.open("grasps.txt");
      myfile << output << std::endl;
      myfile.close();

      // Used to check depth values of a depth image. Not used during actual grasp planning.
      std::ofstream myfile2;
      myfile2.open("object_coordinates.txt");
      for (int i = 0; i < static_cast<int>(msg->depth_image.height); i++) {
        for (int j = 0; j < static_cast<int>(msg->depth_image.width); j++) {
          if (depth_img.at<ushort>(i, j) < table_height && depth_img.at<ushort>(i, j) > 0) {
            myfile2 << std::to_string(j) << "," << std::to_string(i) << std::endl;
          }
        }
      }
      myfile2.close();
    } catch (const boost::filesystem::filesystem_error & error) {
      std::cout << error.what() << std::endl;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto fs = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "[easy_manipulation_deployment][Grasp Planner] Time elapsed(ms): " <<
      fs.count() << std::endl;

    std::string camera_frame = "camera_frame";
    std::string robot_frame = "base_link";

    grasp_planning::msg::GraspPose grasp_planning_output;
    grasp_planning_output.num_objects = msg->num_objects;

    for (int item_num = 0; item_num < static_cast<int>(msg->num_objects); item_num++) {
      shape_msgs::msg::SolidPrimitive object_shape;
      object_shape.type = object_shape.BOX;
      object_shape.dimensions.resize(3);

      object_shape.dimensions[0] = abs(msg->objects[item_num].breadth);
      object_shape.dimensions[1] = abs(msg->objects[item_num].length);
      object_shape.dimensions[2] = abs(msg->objects[item_num].height);

      geometry_msgs::msg::PoseStamped object_pose;
      object_pose = msg->objects[item_num].pos;
      // object_pose.pose.position.z -= abs(msg->objects[item_num].height)/2;
      object_pose.header.frame_id = camera_frame;

      if (grasp_point_vector.size() > 0) {
        geometry_msgs::msg::PoseStamped grasp_pose;
        tf2::Quaternion myQuaternion;
        grasp_pose.header.frame_id = camera_frame;
        grasp_pose.pose.orientation = msg->objects[item_num].pos.pose.orientation;
        grasp_pose.pose.position.x = grasp_point_vector[item_num].x;
        grasp_pose.pose.position.y = grasp_point_vector[item_num].y;
        grasp_pose.pose.position.z = object_pose.pose.position.z;
        grasp_planning_output.grasp_poses.push_back(grasp_pose);
        grasp_planning_output.object_poses.push_back(object_pose);
        grasp_planning_output.object_shapes.push_back(object_shape);
      }
    }

    output_pub->publish(grasp_planning_output);
    // while (rclcpp::ok()) {}
  }
};


#endif  // PLANNING_NODE_HPP_
