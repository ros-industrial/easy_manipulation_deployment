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


#ifndef SCENE_XACRO_PARSER_H_
#define SCENE_XACRO_PARSER_H_


#include <iostream>
#include <fstream>
#include <string>

#include "attributes/workcell.h"


void generate_scene_xacro(Scene scene)
{
  std::ofstream MyFile("scene.urdf.xacro");
  MyFile << "<?xml version=\"1.0\" ?> \n\n";
  MyFile << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"" +
    scene.name + "\">\n\n";  // Change it if you are generating multiple robots
  MyFile << " <link name=\"world\"/>\n\n";  // Declare world joint

  if (scene.robot_loaded) {
    for (int i = 0; i < static_cast < int > (scene.robot_vector.size()); i++) {
      if (scene.robot_vector[i].brand.compare("universal_robot") == 0) {
        // Current ur packages are done differently
        MyFile << " <xacro:include filename=\"$(find ur_description)/urdf/" +
          scene.robot_vector[i].name + ".urdf.xacro\"/>\n";
        MyFile << " <xacro:" + scene.robot_vector[i].name + "_robot" +
          " prefix=\"\" joint_limited=\"false\"/>\n";
      } else {
        MyFile << " <xacro:include filename=\"$(find " + scene.robot_vector[i].name +
          "_description)/urdf/" + scene.robot_vector[i].name + ".urdf.xacro\"/>\n";
        MyFile << " <xacro:" + scene.robot_vector[i].name + "_robot/>\n";
      }

      MyFile << "  <joint name=\"world_" + scene.robot_vector[i].name + "\" type=\"" +
        scene.robot_vector[i].parent_robot_joint_type + "\">\n";
      MyFile << "\t<parent link=\"" + scene.robot_vector[i].parent_link + "\" />\n";
      MyFile << "\t<child link=\"" + scene.robot_vector[i].base_link + "\" />\n";
      if (scene.robot_vector[i].origin.is_origin) {
        MyFile << "\t<origin xyz=\"" + std::to_string(scene.robot_vector[i].origin.x) + " " +
          std::to_string(scene.robot_vector[i].origin.y) + " " + std::to_string(
          scene.robot_vector[i].origin.z) + "\" rpy=\"" + std::to_string(
          scene.robot_vector[i].origin.roll) + " " + std::to_string(
          scene.robot_vector[i].origin.pitch) + " " + std::to_string(
          scene.robot_vector[i].origin.yaw) + "\"/>\n";
      } else {
        MyFile << "\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
      }
      MyFile << "  </joint>\n";
    }
    MyFile << "\n";
  }

  if (scene.ee_loaded) {
    for (int i = 0; i < static_cast < int > (scene.ee_vector.size()); i++) {
      MyFile << " <xacro:include filename=\"$(find " + scene.ee_vector[i].name +
        "_description)/urdf/" + scene.ee_vector[i].name + "_gripper.urdf.xacro\"/>\n";

      MyFile << " <xacro:" + scene.ee_vector[i].name + "_gripper" + " prefix=\"\" parent=\"" +
        scene.robot_vector[scene.ee_vector[i].robot_pos].ee_link + "\">\n";
      if (scene.ee_vector[i].origin.is_origin) {
        std::cout << "Xacro parser has origin" << std::endl;
        MyFile << "\t<origin xyz=\"" + std::to_string(scene.ee_vector[i].origin.x) + " " +
          std::to_string(scene.ee_vector[i].origin.y) + " " + std::to_string(
          scene.ee_vector[i].origin.z) + "\" rpy=\"" +
          std::to_string(scene.ee_vector[i].origin.roll) + " " + std::to_string(
          scene.ee_vector[i].origin.pitch) + " " + std::to_string(scene.ee_vector[i].origin.yaw) +
          "\"/>\n";
      } else {
        std::cout << "Xacro parser has no origin" << std::endl;
        MyFile << "\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
      }
      MyFile << " </xacro:" + scene.ee_vector[i].name + "_gripper" + ">\n";
    }
    MyFile << "\n";
  }

  for (int i = 0; i < static_cast < int > (scene.object_vector.size()); i++) {
    std::string parent_link;
    if (scene.object_vector[i].ext_joint.parent_obj_pos >= 0 ||
      scene.object_vector[i].ext_joint.parent_link_pos >= 0)
    {
      parent_link =
        scene.object_vector[scene.object_vector[i].ext_joint.parent_obj_pos].link_vector[scene.
          object_vector[i].ext_joint.parent_link_pos].name;
    } else {
      parent_link = "world";
    }
    MyFile << " <xacro:include filename=\"$(find " + scene.object_vector[i].name +
      "_description)/urdf/" + scene.object_vector[i].name + ".urdf.xacro\"/>\n";
    MyFile << " <xacro:" + scene.object_vector[i].name + " prefix=\"\" parent=\"" + parent_link +
      "\">\n";
    if (scene.object_vector[i].ext_joint.origin.is_origin) {
      MyFile << "\t<origin xyz=\"" + std::to_string(scene.object_vector[i].ext_joint.origin.x) +
        " " + std::to_string(scene.object_vector[i].ext_joint.origin.y) + " " + std::to_string(
        scene.object_vector[i].ext_joint.origin.z) + "\" rpy=\"" + std::to_string(
        scene.object_vector[i].ext_joint.origin.roll) + " " + std::to_string(
        scene.object_vector[i].ext_joint.origin.pitch) + " " + std::to_string(
        scene.object_vector[i].ext_joint.origin.yaw) + "\"/>\n";
    } else {
      MyFile << "\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
    }
    MyFile << " </xacro:" + scene.object_vector[i].name + ">\n";
  }
  MyFile << "\n";
  MyFile << "</robot>";
  // connect EE to robot
}
#endif  // SCENE_XACRO_PARSER_H_
