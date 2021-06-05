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


#ifndef YAML_PARSER__EXTERNALJOINT_PARSER_H_
#define YAML_PARSER__EXTERNALJOINT_PARSER_H_

#include <string>
#include <iostream>

#include "attributes/external_joint.h"
#include "attributes/object.h"
#include "yaml-cpp/yaml.h"
#include "yaml_parser/origin_parser.h"
#include "yaml_parser/axis_parser.h"


class ExternalJointParser
{
public:
  static void generate_ext_joints(
    YAML::Emitter * out, ExternalJoint ext_joint,
    std::string parent_object, std::string parent_link)
  {
    std::cout << "Populate the ext joint" << std::endl;
    *out << YAML::Key << ext_joint.child_object;
    *out << YAML::Value;
    *out << YAML::BeginMap;
//        *out<<YAML::Key << "type";
//        *out<<YAML::Value << ext_joint.type;
    std::cout << "Populate the parent link" << std::endl;
    *out << YAML::Key << "parent object";
    *out << YAML::Value << parent_object;
    *out << YAML::Key << "parent link";
    *out << YAML::Value << parent_link;
//        *out<<YAML::Key << "child";
//        *out<<YAML::Value << object_vector[ext_joint.child_link_pos];
    if (ext_joint.origin.is_origin) {
      std::cout << "Populate the Origin" << std::endl;
      OriginParser::generate_origin(&out, ext_joint.origin);
    }
    if (ext_joint.axis.is_axis) {
      std::cout << "Populate the ext joint" << std::endl;
      AxisParser::generate_axis(&out, ext_joint.axis);
    }
    std::cout << "Done populating" << std::endl;
    *out << YAML::EndMap;
  }
};


#endif  // YAML_PARSER__EXTERNALJOINT_PARSER_H_
