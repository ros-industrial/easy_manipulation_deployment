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

#ifndef GRASP_PLANNER__COMMON__FCL_FUNCTIONS_HPP_
#define GRASP_PLANNER__COMMON__FCL_FUNCTIONS_HPP_

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
#include <fcl/narrowphase/collision_object.h>  // NOLINT
#include "fcl/geometry/octree/octree.h"


namespace FCLFunctions
{
  std::shared_ptr < fcl::CollisionObject < float >> createCollisionObjectFromPointCloudRGB(
    const pcl::PointCloud < pcl::PointXYZRGB > ::Ptr pointcloud_ptr,
    const octomap::point3d & sensor_origin_wrt_world,
    float resolution);

  std::shared_ptr < fcl::CollisionObject < float >> createCollisionObjectFromPointCloud(
    const pcl::PointCloud < pcl::PointXYZ > ::Ptr & pointcloud_ptr,
    const octomap::point3d & sensor_origin_wrt_world,
    float resolution);
}  // namespace FCLFunctions

#endif  // GRASP_PLANNER__COMMON__FCL_FUNCTIONS_HPP_
