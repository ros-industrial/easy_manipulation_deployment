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


#ifndef ATTRIBUTES__EXTERNAL_JOINT_H_
#define ATTRIBUTES__EXTERNAL_JOINT_H_

#include <string>
#include "attributes/axis.h"
#include "attributes/link.h"
#include "attributes/origin.h"


class ExternalJoint
{
public:
  std::string name;
  Axis axis;
  Origin origin;
  std::string child_object;
  std::string type;
  int child_link_pos;
  int parent_obj_pos;
  int parent_link_pos;
  ExternalJoint()
  {
    parent_obj_pos = -1;
    parent_link_pos = -1;
    child_link_pos = -1;
  }
};


#endif  // ATTRIBUTES__EXTERNAL_JOINT_H_
