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

#ifndef EMD__GRASP_PLANNER__COMMON__FCL_FUNCTIONS_HPP_
#define EMD__GRASP_PLANNER__COMMON__FCL_FUNCTIONS_HPP_

// Main PCL files
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <octomap/octomap.h>

// Other Libraries
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <memory>
// #include "grasp_object.h"

// FCL Libraries
#include "emd/common/fcl_types.hpp"


namespace FCLFunctions
{
/// Creates an FCL collision object from Point Cloud of type PointXYZRGB
/**
 * \param[in] pointcloud_ptr Target CLoud
 * \param[in] sensor_origin_wrt_world Octomap parameter for generation of collision object
 * \param[in] resolution Resolution of FCL Collision Object
 */
std::shared_ptr<grasp_planner::collision::CollisionObject>
create_collision_object_from_pointcloud_rgb(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_ptr,
  const octomap::point3d & sensor_origin_wrt_world,
  float resolution);

/// Creates an FCL collision object from Point Cloud of type PointXYZ
/**
 * \param[in] pointcloud_ptr Target CLoud
 * \param[in] sensor_origin_wrt_world Octomap parameter for generation of collision object
 * \param[in] resolution Resolution of FCL Collision Object
 */
std::shared_ptr<grasp_planner::collision::CollisionObject> create_collision_object_from_pointcloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pointcloud_ptr,
  const octomap::point3d & sensor_origin_wrt_world,
  float resolution);
}  // namespace FCLFunctions

#endif  // EMD__GRASP_PLANNER__COMMON__FCL_FUNCTIONS_HPP_
