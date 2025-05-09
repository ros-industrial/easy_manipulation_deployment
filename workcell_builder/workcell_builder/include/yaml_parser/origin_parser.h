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


#ifndef YAML_PARSER__ORIGIN_PARSER_H_
#define YAML_PARSER__ORIGIN_PARSER_H_

#include "yaml-cpp/yaml.h"
#include "attributes/origin.h"

class OriginParser
{
public:
  static void generate_origin(YAML::Emitter **** out, Origin origin)
  {
    ****out << YAML::Key << "origin";
    ****out << YAML::Value;

    ****out << YAML::BeginMap;

    ****out << YAML::Key << "x";
    ****out << YAML::Value << origin.x;

    ****out << YAML::Key << "y";
    ****out << YAML::Value << origin.y;

    ****out << YAML::Key << "z";
    ****out << YAML::Value << origin.z;

    ****out << YAML::Key << "roll";
    ****out << YAML::Value << origin.roll;

    ****out << YAML::Key << "pitch";
    ****out << YAML::Value << origin.pitch;

    ****out << YAML::Key << "yaw";
    ****out << YAML::Value << origin.yaw;
    ****out << YAML::EndMap;
  }

  static void generate_origin(YAML::Emitter *** out, Origin origin)
  {
    ***out << YAML::Key << "origin";
    ***out << YAML::Value;

    ***out << YAML::BeginMap;


    ***out << YAML::Key << "x";
    ***out << YAML::Value << origin.x;

    ***out << YAML::Key << "y";
    ***out << YAML::Value << origin.y;

    ***out << YAML::Key << "z";
    ***out << YAML::Value << origin.z;

    ***out << YAML::Key << "roll";
    ***out << YAML::Value << origin.roll;

    ***out << YAML::Key << "pitch";
    ***out << YAML::Value << origin.pitch;

    ***out << YAML::Key << "yaw";
    ***out << YAML::Value << origin.yaw;

    // ***out<<YAML::EndSeq;
    ***out << YAML::EndMap;
  }

  static void generate_origin(YAML::Emitter ** out, Origin origin)
  {
    **out << YAML::Key << "origin";
    **out << YAML::Value;
    **out << YAML::BeginMap;


    // **out<<YAML::Key << "origin";
    // **out<<YAML::Value<<YAML::BeginSeq;

    **out << YAML::Key << "x";
    **out << YAML::Value << origin.x;

    **out << YAML::Key << "y";
    **out << YAML::Value << origin.y;

    **out << YAML::Key << "z";
    **out << YAML::Value << origin.z;

    **out << YAML::Key << "roll";
    **out << YAML::Value << origin.roll;

    **out << YAML::Key << "pitch";
    **out << YAML::Value << origin.pitch;

    **out << YAML::Key << "yaw";
    **out << YAML::Value << origin.yaw;

    // **out<<YAML::EndSeq;
    **out << YAML::EndMap;
  }

  static void generate_origin(YAML::Emitter * out, Origin origin)
  {
    *out << YAML::Key << "origin";
    *out << YAML::Value;
    *out << YAML::BeginMap;

    // *out<<YAML::Key << "origin";
    // *out<<YAML::Value<<YAML::BeginSeq;

    *out << YAML::Key << "x";
    *out << YAML::Value << origin.x;

    *out << YAML::Key << "y";
    *out << YAML::Value << origin.y;

    *out << YAML::Key << "z";
    *out << YAML::Value << origin.z;

    *out << YAML::Key << "roll";
    *out << YAML::Value << origin.roll;

    *out << YAML::Key << "pitch";
    *out << YAML::Value << origin.pitch;

    *out << YAML::Key << "yaw";
    *out << YAML::Value << origin.yaw;

    // *out<<YAML::EndSeq;

    *out << YAML::EndMap;
  }
};

#endif  // YAML_PARSER__ORIGIN_PARSER_H_
