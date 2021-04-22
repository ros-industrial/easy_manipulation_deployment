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


#ifndef FILE_FUNCTIONS_H_
#define FILE_FUNCTIONS_H_

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <string>

#include "attributes/workcell.h"

void find_replace(
  std::string example_text, std::string target_text, std::string current_text,
  std::string replaced_text)
{
  std::string file_contents;
  std::ifstream filein(example_text);
  std::ofstream fileout(target_text);
  for (char ch; filein.get(ch); file_contents.push_back(ch)) {
  }

  // This searches the file for the first occurence of the morn string.
  auto pos = file_contents.find(current_text);
  int counter = 0;
  // std::cout<<file_contents<<std::endl;
  while (pos != std::string::npos) {
    file_contents.replace(pos, current_text.length(), replaced_text);
    pos = file_contents.find(current_text);
    std::cout << "position: " << pos << std::endl;
    counter++;
    if (counter > 100) {
      break;
    }
  }
  fileout << file_contents;
  std::remove(example_text.c_str());
}

// void GenerateCMakeLists(boost::filesystem::path workcell_filepath,
// boost::filesystem::path package_filepath,
// std::string package_name,int ros_ver)
void generate_cmakelists(
  boost::filesystem::path workcell_filepath, std::string package_name,
  int ros_ver)
{
  boost::filesystem::path package_filepath(workcell_filepath.string() + "/scenes/" + package_name);
  std::string example_path = workcell_filepath.string() +
    "/easy_manipulation_deployment/workcell_builder/examples";
  boost::filesystem::path example_file(
    example_path + "/ros" + std::to_string(ros_ver) + "/CMakeLists_example.txt");
  boost::filesystem::path target_location(package_filepath.string() + "/CMakeLists_example.txt");
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
  find_replace("CMakeLists_example.txt", "CMakeLists.txt", "workcellexample", package_name);
}

void delete_folder(boost::filesystem::path scene_filepath, std::string scene_name)
{
  boost::filesystem::current_path(scene_filepath);
  if (boost::filesystem::exists(scene_name)) {
    boost::filesystem::remove_all(scene_name);
  }
}

void generate_package_xml(
  boost::filesystem::path workcell_filepath, std::string package_name,
  int ros_ver)
{
  boost::filesystem::path package_filepath(workcell_filepath.string() + "/scenes/" + package_name);
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
bool copyDir(boost::filesystem::path const & source, boost::filesystem::path const & destination)
{
  namespace fs = boost::filesystem;
  try {
    // Check whether the function call is valid
    if (!fs::exists(source) || !fs::is_directory(source)) {
      std::cerr << "Source directory " << source.string() <<
        " does not exist or is not a directory." << '\n';
      return false;
    }
    if (fs::exists(destination)) {
      std::cerr << "Destination directory " << destination.string() << " already exists." << '\n';
      // return false;
    }
    // Create the destination directory
    if (!fs::create_directory(destination)) {
      std::cerr << "Unable to create destination directory" << destination.string() << '\n';
      // return false;
    }
  } catch(fs::filesystem_error const & e) {
    std::cerr << e.what() << '\n';
    return false;
  }
  // Iterate through the source directory
  for (fs::directory_iterator file(source); file != fs::directory_iterator(); ++file) {
    try {
      fs::path current(file->path());
      if (fs::is_directory(current)) {
        // Found directory: Recursion
        if (!copyDir(current, destination / current.filename())) {
          return false;
        }
      } else {
        // Found file: Copy
        fs::copy_file(
          current,
          destination / current.filename(), fs::copy_option::overwrite_if_exists);
      }
    } catch(fs::filesystem_error const & e) {
      std::cerr << e.what() << '\n';
    }
  }
  return true;
}


#endif  // FILE_FUNCTIONS_H_
