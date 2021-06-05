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


#ifndef YAML_PARSER__GENERATE_YAML__COPY__H_
#define YAML_PARSER__GENERATE_YAML__COPY__H_

// For file creation
#include <boost/filesystem.hpp>
#include <fstream>

// General
#include <iostream>
#include <vector>
#include <string>

// For Yaml parsing
#include "yaml-cpp/yaml.h"

#include "attributes/environment.h"
#include "attributes/object.h"
#include "attributes/external_joint.h"


class GenerateYAML
{
public:
  // Environment environment;

  // GenerateYAML(Environment environment_)
  // {
  //     environment = environment_;
  // }

  static void generate_yaml(Environment environment, std::string filepath)
  {
    YAML::Emitter out;
    out << YAML::BeginMap;

    if (environment.robot_vector.size() > 0) {
      out << YAML::Key << "robot";
      out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "name";
      out << YAML::Value << environment.robot_vector[0].name;
      if (environment.robot_vector[0].origin.is_origin) {
        Origin::generate_origin(&out, environment.robot_vector[0].origin);
      }
      out << YAML::Key << "links";
      out << YAML::Value << YAML::BeginSeq;

      for (int i = 0; i < environment.robot_links[0].size(); i++) {
        out << environment.robot_links[0][i];
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
    }

    if (environment.ee_vector.size() > 0) {
      out << YAML::Key << "end_effector";
      out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "name";
      out << YAML::Value << environment.ee_vector[0].name;
      if (environment.ee_vector[0].origin.is_origin) {
        Origin::generate_origin(&out, environment.ee_vector[0].origin);
      }

      out << YAML::Key << "links";
      out << YAML::Value << YAML::BeginSeq;

      for (int i = 0; i < environment.ee_links[0].size(); i++) {
        out << environment.ee_links[0][i];
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
    }

    if (environment.object_vector.size() > 0) {
      out << YAML::Key << "objects";
      out << YAML::Value << YAML::BeginMap;
      for (int i = 0; i < environment.object_vector.size(); i++) {
        Object::generate_object(&out, environment.object_vector[i]);
      }
      out << YAML::EndMap;
    }

    if (environment.ext_joint_vector.size() > 0) {
      out << YAML::Key << "external joints";
      out << YAML::Value << YAML::BeginMap;
      for (int i = 0; i < environment.ext_joint_vector.size(); i++) {
        ExternalJoint::generate_ext_joints(&out, environment.ext_joint_vector[i]);
      }
      out << YAML::EndMap;
    }

    out << YAML::EndMap;
    boost::filesystem::current_path(filepath);
    std::cout << boost::filesystem::current_path() << std::endl;

    std::ofstream myfile;
    myfile.open("environment.yaml");
    myfile << out.c_str();
    myfile.close();

    std::cout << "Output YAML:\n" << out.c_str() << std::endl;
  }
};


#endif  // YAML_PARSER__GENERATE_YAML__COPY__H_
