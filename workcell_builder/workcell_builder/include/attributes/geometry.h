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


#ifndef ATTRIBUTES__GEOMETRY_H_
#define ATTRIBUTES__GEOMETRY_H_

#include <string>

class Geometry
{
public:
  bool is_stl;
  std::string filepath;
  std::string filepath_new;
  float scale_x;
  float scale_y;
  float scale_z;

  std::string shape;

  float length;
  float breadth;
  float height;
  float radius;

  void disableSTL()
  {
    is_stl = false;
    filepath = "None";
    scale_x = -1;
    scale_y = -1;
    scale_z = -1;
  }

  void disableShape()
  {
    is_stl = true;
    length = -1;
    breadth = -1;
    height = -1;
    radius = -1;
    shape = "None";
  }
};

#endif  // ATTRIBUTES__GEOMETRY_H_
