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

#ifndef YAML_PARSER__INERTIAL_PARSER_H_
#define YAML_PARSER__INERTIAL_PARSER_H_

#include <string>

#include "yaml-cpp/yaml.h"

#include "attributes/inertia.h"
#include "yaml_parser/origin_parser.h"


struct InertialParser
{
  std::string name;
  Geometry geometry;
  Origin origin;
  Material material;

  bool is_origin;
  bool is_material;


  static void generate_inertial(YAML::Emitter *** out, Inertial inertial)
  {
    // ***out<<YAML::BeginMap;
    *** out << YAML::Key << "inertial";
    *** out << YAML::Value;
    *** out << YAML::BeginMap;

    *** out << YAML::Key << "mass";
    *** out << YAML::Value << inertial.mass;

    *** out << YAML::Key << "inertia";
    *** out << YAML::Value;
    *** out << YAML::BeginMap;

    *** out << YAML::Key << "ixx";
    *** out << YAML::Value << inertial.ixx;
    *** out << YAML::Key << "ixy";
    *** out << YAML::Value << inertial.ixy;
    *** out << YAML::Key << "ixz";
    *** out << YAML::Value << inertial.ixz;
    *** out << YAML::Key << "iyy";
    *** out << YAML::Value << inertial.iyy;
    *** out << YAML::Key << "iyz";
    *** out << YAML::Value << inertial.iyz;
    *** out << YAML::Key << "izz";
    *** out << YAML::Value << inertial.izz;


    *** out << YAML::EndMap;
    if (inertial.origin.is_origin) {
      OriginParser::generate_origin(&out, inertial.origin);
    }
    *** out << YAML::EndMap;
    // ***out<<YAML::EndMap;
  }
};

#endif  // YAML_PARSER__INERTIAL_PARSER_H_
