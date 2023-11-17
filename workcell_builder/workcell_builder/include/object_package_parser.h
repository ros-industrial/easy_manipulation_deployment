// Copyright 2020 Advanced Remanufacturing and Technology Centre //NOLINT
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


#ifndef OBJECT_PACKAGE_PARSER_H_
#define OBJECT_PACKAGE_PARSER_H_


#include <boost/filesystem.hpp>
#include <iostream>
#include <string>

#include "attributes/object.h"
#include "include/file_functions.h"

bool package_exists(Object object);
void GenerateObjectPackageXML(
  boost::filesystem::path workcell_filepath, std::string package_name,
  int ros_ver)
{
  boost::filesystem::path package_filepath(
    workcell_filepath.string() + "/assets/environment/" + package_name);
  std::string example_path = workcell_filepath.string() +
    "/easy_manipulation_deployment/workcell_builder/examples";
  boost::filesystem::path example_file(
    example_path + "/ros" + std::to_string(ros_ver) + "/package_example.xml");
  boost::filesystem::path target_location(package_filepath.string() + "/package_example.xml");
  try {
    // std::cout << example_file.string() <<std::endl;
    // std::cout << target_location.string() <<std::endl;
    boost::filesystem::copy_file(
      example_file, target_location,
      boost::filesystem::copy_option::overwrite_if_exists);
  } catch(boost::filesystem::filesystem_error const & e) {
    std::cerr << e.what() << '\n';
  }
  boost::filesystem::current_path(package_filepath);
  find_replace("package_example.xml", "package.xml", "workcellexample", package_name);
}
void GenerateObjectCMakeLists(
  boost::filesystem::path workcell_filepath,
  boost::filesystem::path package_filepath, std::string package_name,
  int ros_ver)
{
  std::string example_path = workcell_filepath.string() +
    "/easy_manipulation_deployment/workcell_builder/examples";
  boost::filesystem::path example_file(
    example_path + "/ros" + std::to_string(ros_ver) + "/CMakeLists_object_example.txt");
  boost::filesystem::path target_location(
    package_filepath.string() + "/CMakeLists_object_example.txt");
  try {
    // std::cout << example_file.string() <<std::endl;
    // std::cout << target_location.string() <<std::endl;
    boost::filesystem::copy_file(
      example_file, target_location,
      boost::filesystem::copy_option::overwrite_if_exists);
  } catch(boost::filesystem::filesystem_error const & e) {
    std::cerr << e.what() << '\n';
  }
  boost::filesystem::current_path(package_filepath);
  find_replace("CMakeLists_object_example.txt", "CMakeLists.txt", "workcellexample", package_name);
}
void generate_object_package(boost::filesystem::path workcell_filepath, Object object, int ros_ver)
{
  boost::filesystem::current_path(workcell_filepath);
  boost::filesystem::current_path("assets/environment");
  if (!package_exists(object)) {
    std::cout << "generate_object_package: " << boost::filesystem::current_path() << std::endl;
    // create main _description folder
    boost::filesystem::current_path(object.name + "_description");
    boost::filesystem::path package_filepath(
      workcell_filepath.string() + "/assets/environment/" + object.name + "_description");
    GenerateObjectCMakeLists(
      workcell_filepath, package_filepath, object.name + "_description",
      ros_ver);
    GenerateObjectPackageXML(workcell_filepath, object.name + "_description", ros_ver);

    boost::filesystem::create_directory("urdf");
    boost::filesystem::create_directory("meshes");
    boost::filesystem::current_path("meshes");
    boost::filesystem::create_directory("visual");
    boost::filesystem::create_directory("collision");
    boost::filesystem::path collision_path(
      boost::filesystem::current_path().string() + "/collision");
    boost::filesystem::path visual_path(boost::filesystem::current_path().string() + "/visual");
    for (Link link : object.link_vector) {
      if (link.is_visual && link.visual_vector[0].geometry.is_stl) {
        // Get File name ____.stl
        // Make new filepath visual_path + /____.stl
        std::string stl_name;         // get stl file
        for (auto it = link.visual_vector[0].geometry.filepath.crbegin();
          it != link.visual_vector[0].geometry.filepath.crend(); ++it)
        {
          if (*it != '/') {
            stl_name = std::string(1, *it) + stl_name;
          } else {
            break;
          }
        }

        boost::filesystem::path link_visual_path(visual_path.string() + "/" + stl_name);
        boost::filesystem::current_path(visual_path);
        if (!boost::filesystem::exists(stl_name)) {
          boost::filesystem::copy_file(link.visual_vector[0].geometry.filepath, link_visual_path);
        }
      }
      if (link.is_collision && link.collision_vector[0].geometry.is_stl) {
        // Get File name ____.stl
        // Make new filepath collision_path + /____.stl
        std::string stl_name;         // get stl file
        for (auto it = link.collision_vector[0].geometry.filepath.crbegin();
          it != link.collision_vector[0].geometry.filepath.crend(); ++it)
        {
          if (*it != '/') {
            stl_name = std::string(1, *it) + stl_name;
          } else {
            break;
          }
        }

        boost::filesystem::path link_collision_path(collision_path.string() + "/" + stl_name);
        boost::filesystem::current_path(collision_path);
        if (!boost::filesystem::exists(stl_name)) {
          boost::filesystem::copy_file(
            link.collision_vector[0].geometry.filepath,
            link_collision_path);
        }
      }
    }
  }
}

bool package_exists(Object object)
{
  std::cout << "Package exists: " << boost::filesystem::current_path() << std::endl;
  boost::filesystem::path temp_path(boost::filesystem::current_path());
  std::string package_name = object.name + "_description";
  if (boost::filesystem::is_directory(package_name)) {  // folder exists
    boost::filesystem::current_path(package_name);
    boost::filesystem::path object_path(boost::filesystem::current_path());
    if (boost::filesystem::is_directory("urdf")) {
      boost::filesystem::current_path("urdf");
      std::string xacro_file = object.name + ".urdf.xacro";
    } else {  // no urdf folder
      std::cout << "No URDF folder" << std::endl;
      boost::filesystem::current_path(temp_path);
      return false;
    }
    if (!boost::filesystem::exists("CMakeLists.txt") ||
      !boost::filesystem::exists("package.xml") )
    {
      std::cout << "No CMakeLists or package.xml available" << std::endl;
      boost::filesystem::current_path(temp_path);
      return false;
    }
  } else {
    std::cout << "No description folders" << std::endl;
    boost::filesystem::current_path(temp_path);
    return false;
  }
  std::cout << "Object package ok. " << std::endl;
  boost::filesystem::current_path(temp_path);
  return true;
}
#endif  // OBJECT_PACKAGE_PARSER_H_
