// Copyright 2021 ROS Industrial Consortium Asia Pacific
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

#include <string>

#include "emd/grasp_execution/context.hpp"
#include "emd/grasp_execution/exception.hpp"

namespace grasp_execution
{

/// Parse optional field from yaml.
/**
 * \prarm[in] field_name Name of the optional field.
 * \param[in] node yaml-cpp node object.
 * \param[in] default_value default value of the optional field.
 * \return value of the optional field.
 */
template<typename T>
T _parse_optional_field(
  const std::string & field_name,
  const YAML::Node & node,
  const T & default_value)
{
  return node[field_name] ? node[field_name].as<T>() : default_value;
}

/////////////////////////////////////////////////
void WorkcellContext::init_from_yaml(const std::string & path)
{
  // TODO(anyone): use rcl_yaml_param_parser instead.
  // https://github.com/ros2/rcl/tree/master/rcl_yaml_param_parser
  YAML::Node config_yaml = YAML::LoadFile(path);

  // Check whether yaml starts with workcell namespace.
  if (!config_yaml["workcell"]) {
    throw ContextLoadingException("workcell");
  }

  const YAML::Node & context_yaml = config_yaml["workcell"];

  // Iterate through to load all the groups.
  for (YAML::const_iterator itr = context_yaml.begin();
    itr != context_yaml.end(); itr++)
  {
    const YAML::Node & group_yaml = *itr;

    // Parse group name, compulsory.
    if (!group_yaml["group_name"]) {
      throw ContextLoadingException("group_name");
    }

    std::string group_name = group_yaml["group_name"].as<std::string>();

    this->init_group(group_name);

    // Parse prefix, optional.
    // TODO(anyone): Implement this properly.
    this->groups[group_name].prefix =
      _parse_optional_field<std::string>("prefix", group_yaml, "");

    // Parse executors, use default executor if none is found.
    if (!group_yaml["executors"]) {
      this->load_execution_method(
        group_name,
        "default", "grasp_execution/DefaultExecutor", "");
    } else {
      const YAML::Node & executors_yaml = group_yaml["executors"];
      for (YAML::const_iterator exe_itr = executors_yaml.begin();
        exe_itr != executors_yaml.end(); exe_itr++)
      {
        std::string execution_method = exe_itr->first.as<std::string>();

        // Parse plugin name, compulsory.
        if (!exe_itr->second["plugin"]) {
          throw ContextLoadingException(
                  "plugin",
                  group_name + ".executors");
        }
        std::string execution_plugin = exe_itr->second["plugin"].as<std::string>();

        // Parse controller name, optional, default: "",
        std::string execution_controller = _parse_optional_field<std::string>(
          "controller", exe_itr->second, "");

        this->load_execution_method(
          group_name, execution_method, execution_plugin, execution_controller);
      }
    }

    // Parse end effector if field exists.
    if (group_yaml["end_effectors"]) {
      const YAML::Node & ees_yaml = group_yaml["end_effectors"];
      for (YAML::const_iterator ee_itr = ees_yaml.begin();
        ee_itr != ees_yaml.end(); ee_itr++)
      {
        std::string ee_name = ee_itr->first.as<std::string>();
        const YAML::Node & ee_yaml = ee_itr->second;

        // Parse brand, compulsory field.
        if (!ee_yaml["brand"]) {
          ContextLoadingException(
            "brand", group_name + ".end_effectors." + ee_name);
        }
        std::string ee_brand = ee_yaml["brand"].as<std::string>();

        // Parse link, compulsory field.
        if (!ee_yaml["link"]) {
          ContextLoadingException(
            "link", group_name + ".end_effectors." + ee_brand);
        }
        std::string ee_link = ee_yaml["link"].as<std::string>();

        // Parse clearance, optional field, default: 0.
        double ee_clearance = _parse_optional_field<double>("clearance", ee_yaml, 0.0);

        // Parse dummy driver if no driver field found.
        if (!ee_yaml["driver"] || !ee_yaml["driver"]["plugin"]) {
          this->load_ee(
            group_name,
            ee_name,
            ee_brand, ee_link, ee_clearance,
            "grasp_execution/DummyGripperDriverPlugin", "");
        } else {
          this->load_ee(
            group_name,
            ee_name,
            ee_brand, ee_link, ee_clearance,
            ee_yaml["driver"]["plugin"].as<std::string>(),
            _parse_optional_field<std::string>("controller", ee_yaml["driver"], ""));
        }
      }
    }
  }
}
}  // namespace grasp_execution
