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


#ifndef ARMHAND_XACRO_PARSER_H_
#define ARMHAND_XACRO_PARSER_H_

#include <iostream>
#include <fstream>
#include <string>

#include "attributes/workcell.h"


void generate_armhand_xacro(Robot robot, EndEffector ee, std::string scene_name)
{
  std::ofstream MyFile("arm_hand.srdf.xacro");
  MyFile << "<?xml version=\"1.0\" ?>\n\n";
  MyFile << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"" + scene_name + "\">\n\n";
  MyFile << "  <xacro:include filename=\"$(find " + robot.name + "_moveit_config)/config/" +
    robot.name + ".srdf.xacro\" />\n\n";
  MyFile << "  <xacro:include filename=\"$(find " + ee.name + "_moveit_config)/config/" + ee.name +
    "_gripper.srdf.xacro\" />\n\n";
  MyFile << "  <xacro:" + robot.name + "/>\n\n";
  MyFile << "  <xacro:" + ee.name + "_gripper/>\n\n";

  // For now we will only disable the base link of the  gripepr with all links of the robot.
  // Will change in the future if collisions with other links of the ee happens
  for (int i = 0; i < static_cast < int > (robot.robot_links.size()); i++) {
    MyFile << "  <disable_collisions link1=\"" + ee.base_link + "\" link2=\"" +
      robot.robot_links[i] + "\" reason=\"Never\" />\n";
  }
  MyFile << "\n\n</robot>";
}

void generate_armhand_xacro(Robot robot, std::string scene_name)
{
  std::ofstream MyFile("arm_hand.srdf.xacro");
  MyFile << "<?xml version=\"1.0\" ?>\n\n";
  MyFile << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"" + scene_name + "\">\n\n";
  MyFile << "  <xacro:include filename=\"$(find " + robot.name + "_moveit_config)/config/" +
    robot.name + ".srdf.xacro\" />\n\n";
  MyFile << "  <xacro:" + robot.name + "/>\n\n";
  MyFile << "\n\n</robot>";
}

void generate_armhand_xacro(std::string scene_name)
{
  std::cout << "No robot or hand" << std::endl;
  std::ofstream MyFile("arm_hand.srdf.xacro");
  MyFile << "<?xml version=\"1.0\" ?>\n\n";
  MyFile << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"" + scene_name + "\">\n\n";
  MyFile << "\n\n</robot>";
}
#endif  // ARMHAND_XACRO_PARSER_H_
