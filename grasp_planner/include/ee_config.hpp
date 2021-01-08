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

#ifndef EE_CONFIG_HPP_
#define EE_CONFIG_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include "grasps.hpp"
#include "yaml-cpp/yaml.h"

/**
 * Get the filepath of a package from the shared directory
 */
std::string get_package_filepath(std::string package_name)
{
  std::string package_path = "";
  try {
    package_path = ament_index_cpp::get_package_share_directory(package_name);
  } catch (std::exception & err) {
    throw(err);
  }
  return package_path;
}

/**
 * Move to folder
 */
void goto_folder_filepath(std::string folder_name, std::string package_name)
{
  std::string package_path = get_package_filepath(package_name);
  try {
    boost::filesystem::current_path(package_path + "/" + folder_name);
  } catch (const boost::filesystem::filesystem_error & error) {
    throw(error);
  }
}

/**
 * Load the configuration yaml file containing the End Effector information
 */
YAML::Node load_config_yaml(
  std::string yaml_filename,
  std::string folder_name,
  std::string package_name)
{
  YAML::Node yaml;
  try {
    goto_folder_filepath(folder_name, package_name);
    yaml = YAML::LoadFile(yaml_filename);
  } catch (YAML::BadFile & error) {
    throw(error);
  } catch (const boost::filesystem::filesystem_error & error) {
    throw(error);
  } catch (std::exception & error) {
    throw(error);
  }
  return yaml;
}

/**
 * Get the end effector type
 */
std::string get_ee_type(
  std::string yaml_filename,
  std::string folder_filename,
  std::string package_name)
{
  bool ee_type_loaded = false;
  std::string ee_type = "";
  try {
    YAML::Node config = load_config_yaml(yaml_filename, folder_filename, package_name);
    for (YAML::iterator config_it = config.begin(); config_it != config.end(); ++config_it) {
      if (config_it->first.as<std::string>().compare("end_effector") == 0) {
        for (YAML::iterator ee_it = config_it->second.begin();
          ee_it != config_it->second.end(); ++ee_it)
        {
          if (ee_it->first.as<std::string>().compare("type") == 0) {
            ee_type = ee_it->second.as<std::string>();
            ee_type_loaded = true;
          }
        }
      }
    }
    if (!ee_type_loaded) {
      throw("[ERROR] No end effector type specified in config.yaml");
    }
  } catch (YAML::BadFile & error) {
    throw(error);
  } catch (const char * exp) {
    throw(exp);
  } catch (const boost::filesystem::filesystem_error & error) {
    throw(error);
  } catch (std::exception & error) {
    throw(error);
  }
  return ee_type;
}

/**
 * Load finger gripper attributes from config.yaml file.
 */
TwoFinger load_finger_attributes(
  std::string yaml_filename,
  std::string folder_name,
  std::string package_name)
{
  float table_height = 0;
  bool table_height_loaded = false;
  float distance_between_fingers = 0;
  bool distance_between_fingers_loaded = false;
  float longest_gripper_dim = 0;
  bool longest_gripper_dim_loaded = false;
  int fingers = 0;
  bool fingers_loaded = false;
  bool attributes_present = false;
  float min_zero_angle = 0;
  float min_height_diff_to_grip = 0;
  float min_gdi_diff_for_comparison = 0;
  bool min_zero_angle_loaded = false;
  bool min_height_diff_to_grip_loaded = false;
  bool min_gdi_diff_for_comparison_loaded = false;
  try {
    YAML::Node config = load_config_yaml(yaml_filename, folder_name, package_name);
    for (YAML::iterator config_it = config.begin(); config_it != config.end(); ++config_it) {
      if (config_it->first.as<std::string>().compare("end_effector") == 0) {
        for (YAML::iterator ee_it = config_it->second.begin();
          ee_it != config_it->second.end(); ++ee_it)
        {
          if (ee_it->first.as<std::string>().compare("attributes") == 0) {
            attributes_present = true;
            for (YAML::iterator attribute_it = ee_it->second.begin();
              attribute_it != ee_it->second.end(); ++attribute_it)
            {
              if (attribute_it->first.as<std::string>().compare("distance_between_fingers") == 0) {
                distance_between_fingers = attribute_it->second.as<float>();
                if (distance_between_fingers < 0) {
                  throw("Error: Negative value for distance_between_fingers");
                } else {distance_between_fingers_loaded = true;}
              }
              if (attribute_it->first.as<std::string>().compare("longest_gripper_dim") == 0) {
                longest_gripper_dim = attribute_it->second.as<float>();
                if (longest_gripper_dim < 0) {
                  throw("Error: Negative value for longest_gripper_dim");
                } else {longest_gripper_dim_loaded = true;}
              }
              if (attribute_it->first.as<std::string>().compare("table_height") == 0) {
                table_height = attribute_it->second.as<float>();
                if (table_height < 0) {
                  throw("Error: Negative value for table_height");
                } else {table_height_loaded = true;}
              }
              if (attribute_it->first.as<std::string>().compare("fingers") == 0) {
                fingers = attribute_it->second.as<int>();
                if (fingers != 2) {
                  throw("Only 2 Finger Grippers supported "
                        "for current implementation");
                }
                fingers_loaded = true;
              }
            }
          }
        }
      }
      if (config_it->first.as<std::string>().compare("parameters") == 0) {
        for (YAML::iterator param_it = config_it->second.begin();
          param_it != config_it->second.end(); ++param_it)
        {
          if (param_it->first.as<std::string>().compare("min_zero_angle") == 0) {
            min_zero_angle = param_it->second.as<float>();
            min_zero_angle_loaded = true;
          }
          if (param_it->first.as<std::string>().compare("min_height_diff_to_grip") == 0) {
            min_height_diff_to_grip = param_it->second.as<float>();
            min_height_diff_to_grip_loaded = true;
          }
          if (param_it->first.as<std::string>().compare("min_gdi_diff_for_comparison") == 0) {
            min_gdi_diff_for_comparison = param_it->second.as<float>();
            min_gdi_diff_for_comparison_loaded = true;
          }
        }
      }
    }
    if (!attributes_present) {
      throw("No attribute field available in YAML File");
    }
    if (!table_height_loaded || !longest_gripper_dim_loaded ||
      !distance_between_fingers_loaded || !fingers_loaded ||
      !min_zero_angle_loaded || !min_gdi_diff_for_comparison_loaded ||
      !min_height_diff_to_grip_loaded)
    {
      if (!table_height_loaded) {
        throw("yaml file does not contain the table_height parameter");
      }
      if (!longest_gripper_dim_loaded) {
        throw("yaml file does not contain the longest_gripper_dim parameter");
      }
      if (!distance_between_fingers_loaded) {
        throw("yaml file does not contain the distance_between_fingers parameter");
      }
      if (!fingers_loaded) {
        throw("yaml file does not contain the fingers parameter");
      }
      if (!min_zero_angle_loaded) {
        throw("grasp planning parameter min_zero_angle not loaded!");
      }
      if (!min_gdi_diff_for_comparison_loaded) {
        throw("grasp planning parameter min_gdi_diff_for_comparison not loaded!");
      }
      if (!min_height_diff_to_grip_loaded) {
        throw("grasp planning parameter min_height_diff_to_grip not loaded!");
      }
    } else {
      TwoFinger grasp(min_zero_angle, min_height_diff_to_grip,
        min_gdi_diff_for_comparison, table_height,
        distance_between_fingers, longest_gripper_dim);
      return grasp;
    }
    TwoFinger empty_grasp;
    return empty_grasp;
  } catch (YAML::BadFile & error) {
    throw(error);
  } catch (const char * exp) {
    throw(exp);
  }
}

/**
 * Load suction array gripper attributes from config.yaml file.
 */
SuctionCupArray load_suction_attributes(
  std::string yaml_filename,
  std::string folder_name,
  std::string package_name)
{
  float table_height_ = 0;
  bool table_height_loaded = false;
  int length_cup_num_ = 0;
  bool length_cup_num_loaded = false;
  int breadth_cup_num_ = 0;
  bool breadth_cup_num_loaded = false;
  float radius_ = 0;
  bool radius_loaded = false;
  bool attribute_present = false;
  SuctionCupArray empty_suction;
  try {
    YAML::Node config = load_config_yaml(yaml_filename, folder_name, package_name);
    for (YAML::iterator config_it = config.begin(); config_it != config.end(); ++config_it) {
      if (config_it->first.as<std::string>().compare("end_effector") == 0) {
        for (YAML::iterator ee_it = config_it->second.begin();
          ee_it != config_it->second.end(); ++ee_it)
        {
          if (ee_it->first.as<std::string>().compare("attributes") == 0) {
            attribute_present = true;
            for (YAML::iterator attribute_it = ee_it->second.begin();
              attribute_it != ee_it->second.end(); ++attribute_it)
            {
              if (attribute_it->first.as<std::string>().compare("length_cups") == 0) {
                length_cup_num_ = attribute_it->second.as<int>();
                if (length_cup_num_ <= 0) {
                  throw("[ERROR] Length of suction array"
                        " must have at least 1 cup.");
                }
                if (length_cup_num_ > 2) {
                  throw("[ERROR] Current planner only supports 1x1 cup array");
                }
                length_cup_num_loaded = true;
              }
              if (attribute_it->first.as<std::string>().compare("breadth_cups") == 0) {
                breadth_cup_num_ = attribute_it->second.as<int>();
                if (breadth_cup_num_ <= 0) {
                  throw("[ERROR] Breadth of suction array "
                        "must have at least 1 cup.");
                }
                if (breadth_cup_num_ > 2) {
                  throw("[ERROR] Current planner only supports 1x1 cup array");
                }
                breadth_cup_num_loaded = true;
              }
              if (attribute_it->first.as<std::string>().compare("radius") == 0) {
                if (attribute_it->second.as<float>() <= 0) {
                  throw("[ERROR] radius needs to be a positive number ");
                }
                radius_ = attribute_it->second.as<float>();
                radius_loaded = true;
              }
              if (attribute_it->first.as<std::string>().compare("table_height") == 0) {
                if (attribute_it->second.as<float>() <= 0) {
                  throw("[ERROR] table height needs to be a positive number ");
                }
                table_height_ = attribute_it->second.as<float>();
                table_height_loaded = true;
              }
            }
          }
        }
      }
    }

    if (!attribute_present) {
      throw("No attribute field available in YAML File");
    }
    if (!table_height_loaded || !length_cup_num_loaded ||
      !breadth_cup_num_loaded || !radius_loaded)
    {
      if (!table_height_loaded) {
        throw("[ERROR] yaml file does not contain the table_height parameter");
      }
      if (!length_cup_num_loaded) {
        throw("[ERROR] yaml file does not contain the length_cup_num parameter");
      }
      if (!breadth_cup_num_loaded) {
        throw("[ERROR] yaml file does not contain the breadth_cup_num parameter");
      }
      if (!radius_loaded) {
        throw("[ERROR] yaml file does not contain the radius parameter");
      }
    } else {
      SuctionCupArray suction(length_cup_num_, breadth_cup_num_, radius_, table_height_);
      return suction;
    }
  } catch (YAML::BadFile & error) {
    throw(error);
  } catch (const char * exp) {
    throw(exp);
  } catch (const boost::filesystem::filesystem_error & error) {
    throw(error);
  } catch (std::exception & error) {
    throw(error);
  }
  return empty_suction;
}
#endif  // EE_CONFIG_HPP_
