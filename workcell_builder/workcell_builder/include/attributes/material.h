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


#ifndef ATTRIBUTES__MATERIAL_H_
#define ATTRIBUTES__MATERIAL_H_

#include <string>

class Material
{
public:
  bool is_texture;
  bool is_material;

  std::string filepath;
  std::string material_name;

  float r;
  float g;
  float b;
  float a;


  void disableColor()
  {
    r = -1;
    g = -1;
    b = -1;
    a = -1;
    is_texture = true;
  }

  void disableTexture()
  {
    is_texture = false;
    filepath = "None";
  }
};

#endif  // ATTRIBUTES__MATERIAL_H_
