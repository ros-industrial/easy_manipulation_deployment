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


#ifndef ATTRIBUTES__ENVIRONMENT_H_
#define ATTRIBUTES__ENVIRONMENT_H_

#include <string>
#include <vector>

#include "attributes/end_effector.h"
#include "attributes/external_joint.h"
#include "attributes/object.h"
#include "attributes/robot.h"

class Environment
{
public:
  std::vector < Object > object_vector;
  std::vector < ExternalJoint > ext_joint_vector;
  std::vector < Robot > robot_vector;
  std::vector < EndEffector > ee_vector;
  std::vector < std::vector < std::string >> robot_links;       // Loaded with moveit_file_config
  std::vector < std::vector < std::string >> ee_links;
};

#endif  // ATTRIBUTES__ENVIRONMENT_H_
