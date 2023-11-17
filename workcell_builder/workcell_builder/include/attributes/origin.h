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


#ifndef ATTRIBUTES__ORIGIN_H_
#define ATTRIBUTES__ORIGIN_H_

class Origin
{
public:
  bool is_origin;
  float x;
  float y;
  float z;

  float roll;
  float pitch;
  float yaw;

  void disableOrigin()
  {
    this->is_origin = false;
    x = -1;
    y = -1;
    z = -1;
    roll = -1;
    pitch = -1;
    yaw = -1;
  }
};

#endif  // ATTRIBUTES__ORIGIN_H_
