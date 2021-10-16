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

#include <experimental/filesystem>
#include <string>

#include "test_hardware.hpp"

namespace test_dynamic_safety
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_dynamic_safety.test_hardware");

void TestHardware::load_urdf(
  const std::string & path_to_urdf)
{
  // TODO(anyone): exception handling
  get_file_content(path_to_urdf, urdf_);
}

void TestHardware::load_urdf(
  const std::string & package_name,
  const std::string & path_to_urdf)
{
  std::experimental::filesystem::path res_path(
    ament_index_cpp::get_package_share_directory(package_name));

  // TODO(anyone): exception handling
  load_urdf((res_path / path_to_urdf).string());
}

void TestHardware::load_srdf(
  const std::string & path_to_srdf)
{
  // TODO(anyone): exception handling
  get_file_content(path_to_srdf, srdf_);
}

void TestHardware::load_srdf(
  const std::string & package_name,
  const std::string & path_to_srdf)
{
  std::experimental::filesystem::path res_path(
    ament_index_cpp::get_package_share_directory(package_name));

  // TODO(anyone): exception handling
  load_srdf((res_path / path_to_srdf).string());
}


}  // namespace test_dynamic_safety
