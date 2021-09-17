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

#ifndef EMD__GRASP_EXECUTION__CONTEXT_HPP_
#define EMD__GRASP_EXECUTION__CONTEXT_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace grasp_execution
{

/// Context for workcell configuration.
struct WorkcellContext
{
  /// Context for executor.
  struct Executor
  {
    /// executor plugin
    std::string plugin;

    /// executor controller
    std::string controller;
  };

  /// Context for gripper.
  struct Gripper
  {
    /// gripper brand.
    /**
     * example: "robotiq_85_2f_gripper"
     */
    std::string brand;

    /// gripper ee link.
    /**
     * example: "ee_palm".
     * This will be used as the ee link for planning.
     */
    std::string link;

    /// clearance needed for gripping action
    /**
     * \todo add clearance orientation
     */
    double clearance;

    /// driver plugin.
    Executor driver;
  };

  /// Context for group.
  struct Group
  {
    /// Group prefix
    /**
     * \todo Used properly.
     */
    std::string prefix;

    /// Available executors and executor plugins.
    std::unordered_map<std::string, Executor> executors;

    /// Available end effectors.
    std::unordered_map<std::string, Gripper> end_effectors;
  };

  /// Initialize context from yaml.
  /**
   * YAML configuration file format.
   * ```yaml
   * workcell:
   *   - group: manipulator1
   *     prefix: ur3e_
   *     executors:
   *       default:
   *         plugin: grasp_execution/DefaultExecutor
   *       <custom-executor>:
   *         plugin: <package-name>/<plugin-name>
   *         controller: <controller-name>
   *     end_effectors:
   *       robotiq_2f0:
   *         brand: robotiq_85_2f_gripper
   *         link: ur3e_gripper_link
   *         driver:
   *           plugin: <package-name>/<plugin-name>
   *           controller: <controller-name>
   *
   *   - group: manipulator2
   *     prefix: ur5_
   *     executors:
   *       default:
   *         plugin: grasp_execution/DefaultExecutor
   *       <custom-executor>:
   *         plugin: <package-name>/<plugin-name>
   *         controller: <controller-name>
   *     end_effectors:
   *       vg10_0:
   *         brand: vg10_gripper
   *         link: ur5_gripper_link
   *         driver:
   *           plugin: <package-name>/<plugin-name>
   *           controller: <controller-name>
   * ```
   *
   * \param[in] path file path to the yaml configuration.
   */
  void init_from_yaml(const std::string & path);

  /// Initialize group
  /**
   * Create an empty planning group.
   * \param[in] group_name group name.
   */
  void init_group(const std::string & group_name)
  {
    if (groups.find(group_name) == groups.end()) {
      groups.emplace(group_name, Group());
    }
  }

  /// Load execution method for a group.
  /**
   * This method can be called multiple times to load multiple
   * execution method.
   * \param[in] group_name group name.
   * \param[in] execution_method execution method name.
   * \param[in] execution_plugin execution method plugin name.
   * \param[in] execution_controller execution method controller.
   */
  void load_execution_method(
    const std::string & group_name,
    const std::string & execution_method,
    const std::string & execution_plugin,
    const std::string & execution_controller)
  {
    if (groups.find(group_name) == groups.end()) {
      this->init_group(group_name);
    }

    auto & executors = groups[group_name].executors;
    if (executors.find(execution_method) == executors.end()) {
      executors.emplace(execution_method, Executor());
      executors[execution_method].plugin = execution_plugin;
      executors[execution_method].controller = execution_controller;
    }
  }

  /// Load end effector for a group.
  /**
   * This method can be called multiple times to load multiple
   * end effectors method.
   * \param[in] group_name group name.
   * \param[in] ee_brand end effector brand.
   * \param[in] ee_link end effector link for planning.
   * \param[in] ee_clearance end effector clearance for grasping action.
   * \param[in] ee_driver_plugin end effector driver plugin.
   * \param[in] ee_driver_controller end effector driver controller.
   */
  void load_ee(
    const std::string & group_name,
    const std::string & ee_name,
    const std::string & ee_brand,
    const std::string & ee_link,
    double ee_clearance,
    const std::string & ee_driver_plugin,
    const std::string & ee_driver_controller)
  {
    if (groups.find(group_name) == groups.end()) {
      this->init_group(group_name);
    }

    auto & ees = groups[group_name].end_effectors;
    if (ees.find(ee_name) == ees.end()) {
      ees.emplace(ee_name, Gripper());
      ees[ee_name].brand = ee_brand;
      ees[ee_name].link = ee_link;
      ees[ee_name].clearance = ee_clearance;
      ees[ee_name].driver.plugin = ee_driver_plugin;
      ees[ee_name].driver.controller = ee_driver_controller;
    }
  }

  /// groups
  std::unordered_map<std::string, Group> groups;
};

}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__CONTEXT_HPP_
