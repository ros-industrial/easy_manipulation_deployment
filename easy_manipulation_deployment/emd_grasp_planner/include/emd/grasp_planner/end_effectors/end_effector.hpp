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

#ifndef EMD__GRASP_PLANNER__END_EFFECTORS__END_EFFECTOR_HPP_
#define EMD__GRASP_PLANNER__END_EFFECTORS__END_EFFECTOR_HPP_

// For Visualization
#include <pcl/visualization/cloud_viewer.h>

// General Libraries
#include <memory>
#include <string>
#include <vector>

#include "emd/common/fcl_types.hpp"
#include "emd/grasp_planner/grasp_object.hpp"

#define UNUSED(expr) do {(void)(expr);} while (0)

/*! \brief Generic class for an end effector, to be inherited by other end effectors*/
class EndEffector
{
public:
  using CollisionObject = grasp_planner::collision::CollisionObject;

  /**
   * Generic method used to plan grasps
   *
   * \param[in] object Target grasp object
   * \param[in] grasp_method Grasp method message output defining the planned grasps
   * \param[in] world_collision_object FCL collision object
   * \param[in] camera_frame Camera tf frame for perception system used to plan grasps
   */
  virtual void plan_grasps(
    const GraspObject & object,
    emd_msgs::msg::GraspMethod & grasp_method,
    std::shared_ptr<CollisionObject> world_collision_object,
    std::string camera_frame)
  {
    UNUSED(object);
    UNUSED(grasp_method);
    UNUSED(world_collision_object);
    UNUSED(camera_frame);
  }

  /**
   * Generic method to visualize grasps. currently done using PCL visualizer
   *
   * \param[in] viewer PCL Visualizer
   * \param[in] object Target grasp object
   */
  virtual void visualize_grasps(
    pcl::visualization::PCLVisualizer::Ptr viewer,
    const GraspObject & object)
  {
    UNUSED(viewer);
    UNUSED(object);
  }

  /**
   * Generic method to return end effector ID
   */
  virtual std::string get_id() {return id;}

protected:
  std::string id;

};
#endif  // EMD__GRASP_PLANNER__END_EFFECTORS__END_EFFECTOR_HPP_
