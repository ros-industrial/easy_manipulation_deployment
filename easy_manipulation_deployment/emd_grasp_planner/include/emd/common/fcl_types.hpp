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

#ifndef EMD__GRASP_PLANNER__COMMON__FCL_TYPES_HPP_
#define EMD__GRASP_PLANNER__COMMON__FCL_TYPES_HPP_

#if FCL_VERSION_0_6_OR_HIGHER == 1

#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/geometry/shape/sphere.h"

#else

#include "fcl/collision_object.h"
#include "fcl/octree.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/collision.h"

#endif

namespace grasp_planner
{

namespace collision
{

#if FCL_VERSION_0_6_OR_HIGHER == 1

using fcl_data_type = float;

using CollisionObject = fcl::CollisionObject<fcl_data_type>;
using CollisionGeometry = fcl::CollisionGeometry<fcl_data_type>;
using OcTree = fcl::OcTree<fcl_data_type>;
using Sphere = fcl::Sphere<fcl_data_type>;
using Box = fcl::Box<fcl_data_type>;
using CollisionRequest = fcl::CollisionRequest<fcl_data_type>;
using CollisionResult = fcl::CollisionResult<fcl_data_type>;
using Transform = fcl::Transform3<float>;
using Vector = fcl::Vector3<float>;

#else

using CollisionObject = fcl::CollisionObject;
using CollisionGeometry = fcl::CollisionGeometry;
using OcTree = fcl::OcTree;
using Sphere = fcl::Sphere;
using Box = fcl::Box;
using CollisionRequest = fcl::CollisionRequest;
using CollisionResult = fcl::CollisionResult;
using Transform = fcl::Transform3f;
using Vector = fcl::Vec3f;

#endif

}  // namespace collision

}  // namespace grasp_planner

#endif // EMD__GRASP_PLANNER__COMMON__FCL_TYPES_HPP_
