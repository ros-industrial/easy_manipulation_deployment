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

#include "emd/common/fcl_functions.hpp"

using namespace grasp_planner::collision;

std::shared_ptr<CollisionObject> FCLFunctions::create_collision_object_from_pointcloud_rgb(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_ptr,
  const octomap::point3d & sensor_origin_wrt_world,
  float resolution)
// std::shared_ptr<octomap::OcTree> createOctomapFromPointCloud(
//  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_ptr,
//  const octomap::point3d& sensor_origin_wrt_world, float resolution)
{
  // octomap octree settings
  // const double resolution = 0.01;
  const double prob_hit = 0.9;
  const double prob_miss = 0.1;
  const double clamping_thres_min = 0.12;
  const double clamping_thres_max = 0.98;

  std::shared_ptr<octomap::OcTree> octomap_octree = std::make_shared<octomap::OcTree>(resolution);
  octomap_octree->setProbHit(prob_hit);
  octomap_octree->setProbMiss(prob_miss);
  octomap_octree->setClampingThresMin(clamping_thres_min);
  octomap_octree->setClampingThresMax(clamping_thres_max);

  octomap::KeySet free_cells;
  octomap::KeySet occupied_cells;

#if defined(_OPENMP)
#pragma omp parallel
#endif
  {
#if defined(_OPENMP)
    auto thread_id = omp_get_thread_num();
    auto thread_num = omp_get_num_threads();
#else
    int thread_id = 0;
    int thread_num = 1;
#endif
    int start_idx = static_cast<int>(pointcloud_ptr->size() / thread_num) * thread_id;
    int end_idx = static_cast<int>(pointcloud_ptr->size() / thread_num) * (thread_id + 1);
    if (thread_id == thread_num - 1) {
      end_idx = pointcloud_ptr->size();
    }
    octomap::KeySet local_free_cells;
    octomap::KeySet local_occupied_cells;

    for (auto i = start_idx; i < end_idx; i++) {
      octomap::point3d point(
        (*pointcloud_ptr)[i].x, (*pointcloud_ptr)[i].y,
        (*pointcloud_ptr)[i].z);
      octomap::KeyRay key_ray;
      if (octomap_octree->computeRayKeys(sensor_origin_wrt_world, point, key_ray)) {
        local_free_cells.insert(key_ray.begin(), key_ray.end());
      }

      octomap::OcTreeKey tree_key;
      if (octomap_octree->coordToKeyChecked(point, tree_key)) {
        local_occupied_cells.insert(tree_key);
      }
    }

#if defined(_OPENMP)
#pragma omp critical
#endif
    {
      free_cells.insert(local_free_cells.begin(), local_free_cells.end());
      occupied_cells.insert(local_occupied_cells.begin(), local_occupied_cells.end());
    }
  }

  // free cells only if not occupied in this cloud
  for (auto it = free_cells.begin(); it != free_cells.end(); ++it) {
    if (occupied_cells.find(*it) == occupied_cells.end()) {
      octomap_octree->updateNode(*it, false);
    }
  }

  // occupied cells
  for (auto it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
    octomap_octree->updateNode(*it, true);
  }

  // return octomap_octree;

  auto fcl_octree = std::make_shared<OcTree>(octomap_octree);
  std::shared_ptr<CollisionGeometry> fcl_geometry = fcl_octree;
  std::make_shared<CollisionObject>(fcl_geometry);
  return std::make_shared<CollisionObject>(fcl_geometry);
}

std::shared_ptr<CollisionObject> FCLFunctions::create_collision_object_from_pointcloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pointcloud_ptr,
  const octomap::point3d & sensor_origin_wrt_world,
  float resolution)
// std::shared_ptr<octomap::OcTree> createOctomapFromPointCloud(
// const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_ptr,
// const octomap::point3d& sensor_origin_wrt_world, float resolution)
{
  // octomap octree settings
  // const double resolution = 0.01;
  const double prob_hit = 0.9;
  const double prob_miss = 0.1;
  const double clamping_thres_min = 0.12;
  const double clamping_thres_max = 0.98;

  std::shared_ptr<octomap::OcTree> octomap_octree = std::make_shared<octomap::OcTree>(resolution);
  octomap_octree->setProbHit(prob_hit);
  octomap_octree->setProbMiss(prob_miss);
  octomap_octree->setClampingThresMin(clamping_thres_min);
  octomap_octree->setClampingThresMax(clamping_thres_max);

  octomap::KeySet free_cells;
  octomap::KeySet occupied_cells;

#if defined(_OPENMP)
#pragma omp parallel
#endif
  {
#if defined(_OPENMP)
    auto thread_id = omp_get_thread_num();
    auto thread_num = omp_get_num_threads();
#else
    int thread_id = 0;
    int thread_num = 1;
#endif
    int start_idx = static_cast<int>(pointcloud_ptr->size() / thread_num) * thread_id;
    int end_idx = static_cast<int>(pointcloud_ptr->size() / thread_num) * (thread_id + 1);
    if (thread_id == thread_num - 1) {
      end_idx = pointcloud_ptr->size();
    }
    octomap::KeySet local_free_cells;
    octomap::KeySet local_occupied_cells;

    for (auto i = start_idx; i < end_idx; i++) {
      octomap::point3d point(
        (*pointcloud_ptr)[i].x, (*pointcloud_ptr)[i].y,
        (*pointcloud_ptr)[i].z);
      octomap::KeyRay key_ray;
      if (octomap_octree->computeRayKeys(sensor_origin_wrt_world, point, key_ray)) {
        local_free_cells.insert(key_ray.begin(), key_ray.end());
      }

      octomap::OcTreeKey tree_key;
      if (octomap_octree->coordToKeyChecked(point, tree_key)) {
        local_occupied_cells.insert(tree_key);
      }
    }

#if defined(_OPENMP)
#pragma omp critical
#endif
    {
      free_cells.insert(local_free_cells.begin(), local_free_cells.end());
      occupied_cells.insert(local_occupied_cells.begin(), local_occupied_cells.end());
    }
  }

  // free cells only if not occupied in this cloud
  for (auto it = free_cells.begin(); it != free_cells.end(); ++it) {
    if (occupied_cells.find(*it) == occupied_cells.end()) {
      octomap_octree->updateNode(*it, false);
    }
  }

  // occupied cells
  for (auto it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
    octomap_octree->updateNode(*it, true);
  }

  // return octomap_octree;

  auto fcl_octree = std::make_shared<OcTree>(octomap_octree);
  std::shared_ptr<CollisionGeometry> fcl_geometry = fcl_octree;
  std::make_shared<CollisionObject>(fcl_geometry);
  return std::make_shared<CollisionObject>(fcl_geometry);
}
