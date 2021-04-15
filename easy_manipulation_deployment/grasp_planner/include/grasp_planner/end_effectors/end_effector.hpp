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

#ifndef GRASP_PLANNER__END_EFFECTORS__END_EFFECTOR_HPP_
#define GRASP_PLANNER__END_EFFECTORS__END_EFFECTOR_HPP_

// For Visualization
#include <pcl/visualization/cloud_viewer.h>
#include <fcl/narrowphase/collision.h>

// General Libraries
#include <memory>
#include <string>
#include <vector>

#include "grasp_planner/grasp_object.hpp"


class EndEffector
{
public:
  virtual void planGrasps(
    std::shared_ptr < GraspObject > object,
    emd_msgs::msg::GraspMethod * grasp_method,
    std::shared_ptr < fcl::CollisionObject < float >> world_collision_object,
    pcl::visualization::PCLVisualizer::Ptr viewer) {}
  virtual void visualizeGrasps(pcl::visualization::PCLVisualizer::Ptr viewer) {}
  virtual std::string getID() {return id;}

protected:
  std::string id;
};
#endif  // GRASP_PLANNER__END_EFFECTORS__END_EFFECTOR_HPP_
