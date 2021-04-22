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


#ifndef YAML_PARSER__OBJECT_PARSER_H_
#define YAML_PARSER__OBJECT_PARSER_H_

#include "yaml-cpp/yaml.h"

#include "attributes/object.h"
#include "yaml_parser/joint_parser.h"
#include "yaml_parser/link_parser.h"


class ObjectParser
{
public:
  static void generate_object(YAML::Emitter * out, Object object)
  {
    *out << YAML::Key << object.name;
    *out << YAML::Value;
    if (object.link_vector.size() > 0 || object.joint_vector.size() > 0) {
      *out << YAML::BeginMap;
      *out << YAML::Key << "child_link";
      *out << YAML::Value << object.link_vector[object.ext_joint.child_link_pos].name;
      if (object.link_vector.size() > 0) {
        *out << YAML::Key << "links";
        *out << YAML::Value;
        *out << YAML::BeginMap;
        for (int i = 0; i < static_cast < int > (object.link_vector.size()); i++) {
          LinkParser::generate_link(&out, object.link_vector[i]);
        }
        *out << YAML::EndMap;
      }

      if (object.joint_vector.size() > 0) {
        *out << YAML::Key << "joints";
        *out << YAML::Value;
        *out << YAML::BeginMap;

        for (int i = 0; i < static_cast < int > (object.joint_vector.size()); i++) {
          JointParser::generate_joint(&out, object.joint_vector[i]);
        }
        *out << YAML::EndMap;
      }

      *out << YAML::Key << object.ext_joint.name;
      *out << YAML::Value;
      *out << YAML::BeginMap;
      *out << YAML::Key << "ext_joint_type";
      *out << YAML::Value << object.ext_joint.type;
      *out << YAML::Key << "child_link";
      *out << YAML::Value << object.link_vector[object.ext_joint.child_link_pos].name;
      *out << YAML::EndMap;

      *out << YAML::EndMap;
    }
  }
};

#endif  // YAML_PARSER__OBJECT_PARSER_H_
