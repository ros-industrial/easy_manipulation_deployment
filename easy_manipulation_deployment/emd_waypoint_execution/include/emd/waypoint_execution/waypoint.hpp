// Copyright 2022 ROS Industrial Consortium Asia Pacific
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

#include <string>
#include <vector>

#ifndef WAYPOINT__HPP_
#define WAYPOINT__HPP_
namespace emd
{
struct WayPoint
{

  /*! \brief Action type to execute. Move, Collision, Grasp or Release */
  std::string action_type;

  /*! \brief If true, Position and orientation is with respect to the previous waypoint.
    If false, it is with respect to the frame_id*/
  bool is_relative;

  /*! \brief New position offset from previous waypoint */
  std::vector<float> position;

  /*! \brief New orientation offset from previous waypoint */
  std::vector<float> rpy;

  /*! \brief Description of action */
  std::string action_description;

  /*! \brief Axis to consider in collision type environment */
  std::vector<int> collision_axis_direction;

  /*! \brief Constructor for move type actions */
  WayPoint(
    bool is_relative_,
    const std::string & action_type_,
    const std::string & action_description_)
  {
    is_relative = is_relative_;
    action_type = action_type_;
    action_description = action_description_;
  }

  /*! \brief Constructor for collision type actions */
  WayPoint(
    bool is_relative_,
    const std::string & action_type_,
    const std::string & action_description_,
    std::vector<int> collision_axis_direction_)
  {
    is_relative = is_relative_;
    action_type = action_type_;
    action_description = action_description_;
    collision_axis_direction = collision_axis_direction_;
  }

  /*! \brief Constructor for grasp/release type actions */
  WayPoint(
    const std::string & action_type_,
    const std::string & action_description_)
  {
    action_type = action_type_;
    action_description = action_description_;
  }
};
}


#endif // WAYPOINT__HPP_
