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


#ifndef YAML_PARSER__LINK_PARSER_H_
#define YAML_PARSER__LINK_PARSER_H_

#include <vector>

#include "yaml-cpp/yaml.h"

#include "attributes/link.h"
#include "yaml_parser/collision_parser.h"
#include "yaml_parser/inertial_parser.h"
#include "yaml_parser/visual_parser.h"


class LinkParser
{
public:
  static void generate_link(YAML::Emitter ** out, Link link)
  {
    **out << YAML::Key << link.name;
    **out << YAML::Value;
    **out << YAML::BeginMap;
    if (link.is_visual || link.is_collision || link.is_inertial) {
      // **out<<YAML::BeginMap;

      if (link.is_visual) {
        VisualParser::generate_visual(&out, link.visual_vector[0]);
      }

      if (link.is_collision) {
        CollisionParser::generate_collision(&out, link.collision_vector[0]);
      }
      if (link.is_inertial) {
        InertialParser::generate_inertial(&out, link.inertial_vector[0]);
      }
      // **out<<YAML::EndMap;
    }
    **out << YAML::EndMap;
  }
};

#endif  // YAML_PARSER__LINK_PARSER_H_
