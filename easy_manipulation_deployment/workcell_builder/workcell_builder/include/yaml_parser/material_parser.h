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


#ifndef YAML_PARSER__MATERIAL_PARSER_H_
#define YAML_PARSER__MATERIAL_PARSER_H_

#include <string>
#include "yaml-cpp/yaml.h"
#include "attributes/material.h"


class MaterialParser
{
public:
  static void generate_material(YAML::Emitter **** out, Material material)
  {
    ****out << YAML::Key << "material";
    ****out << YAML::Value;

    if (material.is_texture) {
      ****out << YAML::BeginMap;
      ****out << YAML::Key << "filepath";
      ****out << YAML::Value << material.filepath;
      ****out << YAML::EndMap;
    } else if (material.is_material) {
      ****out << YAML::BeginMap;
      ****out << YAML::Key << "name";
      ****out << YAML::Value << material.material_name;

      ****out << YAML::Key << "r";
      ****out << YAML::Value << material.r;

      ****out << YAML::Key << "g";
      ****out << YAML::Value << material.g;

      ****out << YAML::Key << "b";
      ****out << YAML::Value << material.b;

      ****out << YAML::Key << "a";
      ****out << YAML::Value << material.a;
      ****out << YAML::EndMap;
    }
  }
};

#endif  // YAML_PARSER__MATERIAL_PARSER_H_
