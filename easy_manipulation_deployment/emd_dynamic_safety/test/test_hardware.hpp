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

#ifndef TEST_HARDWARE_HPP_
#define TEST_HARDWARE_HPP_

#include <fstream>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace test_dynamic_safety
{

class TestHardware
{
public:
  TestHardware() = default;
  virtual ~TestHardware() = default;
  TestHardware(const TestHardware &) = default;
  TestHardware(TestHardware &&) = default;
  TestHardware & operator=(const TestHardware &) = default;
  TestHardware & operator=(TestHardware &&) = default;

  void load_urdf(const std::string & package_name, const std::string & path_to_urdf);
  void load_srdf(const std::string & package_name, const std::string & path_to_srdf);

  void load_urdf(const std::string & path_to_urdf);
  void load_srdf(const std::string & path_to_srdf);

  const std::string & get_urdf() const
  {
    return urdf_;
  }

  const std::string & get_srdf() const
  {
    return srdf_;
  }

private:
  std::string urdf_;
  std::string srdf_;

  bool get_file_content(const std::string & filename, std::string & content)
  {
    std::string content_temp;
    std::fstream file(filename.c_str(), std::fstream::in);
    if (file.is_open()) {
      while (file.good()) {
        std::string line;
        std::getline(file, line);
        content_temp += (line + '\n');
      }
      file.close();
      content = content_temp;
      return true;
    } else {
      fprintf(stderr, "Could not open file [%s] for parsing.\n", filename.c_str());
      return false;
    }
  }
};

}  // namespace test_dynamic_safety

#endif  // TEST_HARDWARE_HPP_
