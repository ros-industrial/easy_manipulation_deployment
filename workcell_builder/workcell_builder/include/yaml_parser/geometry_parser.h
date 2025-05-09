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

#ifndef YAML_PARSER__GEOMETRY_PARSER_H_
#define YAML_PARSER__GEOMETRY_PARSER_H_

#include <string>

#include "yaml-cpp/yaml.h"
#include "attributes/geometry.h"

class GeometryParser
{
public:
  static void generate_geometry(YAML::Emitter **** out, Geometry geometry)
  {
    // ****out<<YAML::BeginMap;
    ****out << YAML::Key << "geometry";
    ****out << YAML::Value;
    if (geometry.is_stl) {
      ****out << YAML::BeginMap;
      ****out << YAML::Key << "filepath";
      ****out << YAML::Value << geometry.filepath_new;

      ****out << YAML::Key << "scale";
      ****out << YAML::Value << YAML::BeginMap;

      ****out << YAML::Key << "scale_x";
      ****out << YAML::Value << geometry.scale_x;

      ****out << YAML::Key << "scale_y";
      ****out << YAML::Value << geometry.scale_y;

      ****out << YAML::Key << "scale_z";
      ****out << YAML::Value << geometry.scale_z;

      ****out << YAML::EndMap;
      ****out << YAML::EndMap;
    } else {
      ****out << YAML::BeginMap;

      ****out << YAML::Key << "shape";
      ****out << YAML::Value << geometry.shape;
      if (geometry.shape.compare("Box") == 0) {
        // if(strcmp(geometry.shape.c_str(), "Box") == 0)//Box
        ****out << YAML::Key << "length" << YAML::Value << geometry.length;
        // ****out<<YAML::Value << geometry.length;

        ****out << YAML::Key << "breadth" << YAML::Value << geometry.breadth;
        // ****out<<YAML::Value << geometry.breadth;

        ****out << YAML::Key << "height" << YAML::Value << geometry.height;
        // ****out<<YAML::Value << geometry.height;
      } else if (geometry.shape.compare("Cylinder") == 0) {
        // else if(strcmp(geometry.shape.c_str(), "Cylinder") == 0) //Cylinder
        ****out << YAML::Key << "height";
        ****out << YAML::Value << geometry.height;

        ****out << YAML::Key << "radius";
        ****out << YAML::Value << geometry.radius;
      } else if (geometry.shape.compare("Sphere") == 0) {
        // else if(strcmp(geometry.shape.c_str(), "Sphere") == 0) //Sphere
        ****out << YAML::Key << "radius";
        ****out << YAML::Value << geometry.radius;
      }
      ****out << YAML::EndMap;
    }
    // ****out<<YAML::EndMap;
  }
};

#endif  // YAML_PARSER__GEOMETRY_PARSER_H_
