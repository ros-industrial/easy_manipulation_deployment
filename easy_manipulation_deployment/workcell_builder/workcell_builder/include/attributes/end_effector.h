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


#ifndef ATTRIBUTES__END_EFFECTOR_H_
#define ATTRIBUTES__END_EFFECTOR_H_

#include <string>
#include <vector>

#include "attributes/origin.h"

class EndEffector
{
public:
  // End effector should have urdf in package <name>_description
  // and filepath $(find <name>_description)/urdf/<name>.urdf.xacro
  EndEffector()
  {
      origin.is_origin = false;
  }
  std::string name;
  std::string brand;
  std::string filepath;
  std::string base_link;
  int robot_pos = 0;
  std::string robot_link;
  std::string ee_type;
  int attribute_1 = -1;
  int attribute_2 = -1;
  std::string ee_robot_joint_type = "fixed";
  Origin origin;
  std::vector < std::string > ee_links;
};


#endif  // ATTRIBUTES__END_EFFECTOR_H_
