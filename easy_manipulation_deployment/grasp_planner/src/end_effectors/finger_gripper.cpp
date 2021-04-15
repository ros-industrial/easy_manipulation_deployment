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


// Main PCL files
#include "grasp_planner/end_effectors/finger_gripper.hpp"

FingerGripper::FingerGripper(
  std::string id_,
  const int num_fingers_side_1_,
  const int num_fingers_side_2_,
  const float distance_between_fingers_1_,
  const float distance_between_fingers_2_,
  const float finger_thickness_,
  const float gripper_stroke_,
  const float voxel_size_,
  const float grasp_quality_weight1_,
  const float grasp_quality_weight2_,
  const float grasp_plane_dist_limit_,
  const float cloud_normal_radius_,
  const float worldXAngleThreshold_,
  const float worldYAngleThreshold_,
  const float worldZAngleThreshold_)
: id(id_),
  num_fingers_side_1(num_fingers_side_1_),
  num_fingers_side_2(num_fingers_side_2_),
  distance_between_fingers_1(distance_between_fingers_1_),
  distance_between_fingers_2(distance_between_fingers_2_),
  finger_thickness(finger_thickness_),
  gripper_stroke(gripper_stroke_),
  voxel_size(voxel_size_),
  grasp_quality_weight1(grasp_quality_weight1_),
  grasp_quality_weight2(grasp_quality_weight2_),
  grasp_plane_dist_limit(grasp_plane_dist_limit_),
  cloud_normal_radius(cloud_normal_radius_),
  worldXAngleThreshold(worldXAngleThreshold_),
  worldYAngleThreshold(worldYAngleThreshold_),
  worldZAngleThreshold(worldZAngleThreshold_)
{
  this->num_fingers_total = this->num_fingers_side_1 + this->num_fingers_side_2;
}

void FingerGripper::planGrasps(
  std::shared_ptr<GraspObject> object,
  emd_msgs::msg::GraspMethod * grasp_method,
  std::shared_ptr<fcl::CollisionObject<float>> world_collision_object,
  pcl::visualization::PCLVisualizer::Ptr viewer)
{
  std::cout << "getCuttingPlanes" << std::endl;
  getCuttingPlanes(object);
  std::cout << "getGraspCloud" << std::endl;
  getGraspCloud(object);

  std::cout << "getInitialSamplePoints" << std::endl;
  if (!this->getInitialSamplePoints(object)) {
    this->resetVariables();
    return;
  }

  std::cout << "getInitialSampleCloud" << std::endl;
  getInitialSampleCloud(object);
  std::cout << "voxelizeSampleCloud" << std::endl;
  voxelizeSampleCloud();
  std::cout << "getMaxMinValues" << std::endl;
  getMaxMinValues(object);
  std::cout << "getFingerSamples" << std::endl;
  getFingerSamples(object);
  std::cout << "getGripperClusters" << std::endl;
  getGripperClusters();
  std::cout << "getAllGripperConfigs" << std::endl;
  std::vector<std::shared_ptr<multiFingerGripper>> valid_open_gripper_configs =
    getAllGripperConfigs(object, world_collision_object);

  for (auto & gripper : valid_open_gripper_configs) {
    getGraspPose(gripper, object);
  }

  this->sorted_gripper_configs = getAllRanks(valid_open_gripper_configs, grasp_method);
  std::cout << "grasp_samples size: " << this->grasp_samples.size() << std::endl;
  for (auto grasp_sample : this->grasp_samples) {
    viewer->addPointCloud<pcl::PointNormal>(grasp_sample->grasp_plane_ncloud, "normals");
    // viewer->addPlane(*(grasp_sample->plane), "cenerplane", 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb_1(
      grasp_sample->sample_side_1->finger_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB>(
      grasp_sample->sample_side_1->finger_cloud, rgb_1,
      "finger1");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb_2(
      grasp_sample->sample_side_2->finger_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB>(
      grasp_sample->sample_side_2->finger_cloud, rgb_2,
      "finger2");
    viewer->spin();
    viewer->close();
  }

  std::cout << "outside getallranks functions" << std::endl;
}


/***************************************************************************//**
 * Function to create cutting planes along the object. each cutting plane represents
 * the planar contact area for two fingers. For multi-fingered gripper, multiple planes are created,
 * @param object grasp object
 ******************************************************************************/
void FingerGripper::getCuttingPlanes(const std::shared_ptr<GraspObject> object)
{
  /*! \brief First we find the vector representing the major axis of the object */
  pcl::PointXYZ gradient_vector = pcl::PointXYZ(
    object->axis(0) - object->centerpoint(0),
    object->axis(1) - object->centerpoint(1), object->axis(2) - object->centerpoint(2));

  /*! \brief The major axis of the object acts as the normal of the cutting plane.
  Create the cutting plane that goes through the center of the object. */
  float a = gradient_vector.x;
  float b = gradient_vector.y;
  float c = gradient_vector.z;
  float d =
    -((a * object->centerpoint(0)) + (b * object->centerpoint(1)) + (c * object->centerpoint(2)));
  Eigen::Vector4f grasp_plane_vector(a, b, c, d);
  Eigen::Vector3f grasp_plane_normal_(a, b, c);


  this->center_cutting_plane = grasp_plane_vector;
  this->center_cutting_plane_normal = grasp_plane_normal_;

  bool side_1_even = this->num_fingers_side_1 % 2 == 0;
  bool side_2_even = this->num_fingers_side_2 % 2 == 0;
  bool both_sides_even = (side_1_even && side_2_even);
  bool both_sides_odd = (!side_1_even && !side_2_even);

  /*! \brief If any of the sides have odd numbered fingers,
  we include the center plane in consideration for that side. */

  if (!side_1_even || !side_2_even) {
    pcl::ModelCoefficients::Ptr center_plane(new pcl::ModelCoefficients);

    graspPlaneSample grasp_sample;
    grasp_sample.plane_index = 0;
    grasp_sample.dist_to_center_plane = 0;
    grasp_sample.plane->values.resize(4);
    grasp_sample.plane_eigen(0) = grasp_sample.plane->values[0] = this->center_cutting_plane(0);
    grasp_sample.plane_eigen(1) = grasp_sample.plane->values[1] = this->center_cutting_plane(1);
    grasp_sample.plane_eigen(2) = grasp_sample.plane->values[2] = this->center_cutting_plane(2);
    grasp_sample.plane_eigen(3) = grasp_sample.plane->values[3] = this->center_cutting_plane(3);

    this->grasp_samples.push_back(std::make_shared<graspPlaneSample>(grasp_sample));
    this->cutting_plane_distances.push_back(0);


    if (!side_1_even) {
      this->plane_1_index.push_back(0);
    }
    if (!side_2_even) {
      this->plane_2_index.push_back(0);
    }
  }

  /*! \brief If both sides are odd or both even, the spacing for planes is consistent */
  if (both_sides_even || both_sides_odd) {
    // std::cout << "Both sides even or odd" << std::endl;
    if (this->distance_between_fingers_1 == this->distance_between_fingers_2) {
      addCuttingPlanesEqualAligned(object->centerpoint, grasp_plane_vector, both_sides_even);
    } else {
      float initial_gap_1 =
        (both_sides_even ? this->distance_between_fingers_1 / 2 : this->distance_between_fingers_1);
      float initial_gap_2 =
        (both_sides_even ? this->distance_between_fingers_2 / 2 : this->distance_between_fingers_2);

      int num_itr_1 = (this->num_fingers_side_1 == 1 ? 0 : floor(this->num_fingers_side_1 / 2));
      int num_itr_2 = (this->num_fingers_side_2 == 1 ? 0 : floor(this->num_fingers_side_2 / 2));
      addCuttingPlanes(
        object->centerpoint, grasp_plane_vector, num_itr_1, num_itr_2, initial_gap_1,
        initial_gap_2);
    }

  } else { /*! \brief If both sides are different, the spacing for planes is not consistent,
                      we need to create planes separately */
    // std::cout << "Both sides unequal" << std::endl;
    bool is_even_fingers_1 = this->num_fingers_side_1 % 2 == 0;
    bool is_even_fingers_2 = this->num_fingers_side_2 % 2 == 0;

    int num_itr_1 = (this->num_fingers_side_1 == 1 ? 0 : floor(this->num_fingers_side_1 / 2) + 1);
    int num_itr_2 = (this->num_fingers_side_2 == 1 ? 0 : floor(this->num_fingers_side_2 / 2) + 1);

    float initial_gap_1 = this->distance_between_fingers_1 / (1.0 + (is_even_fingers_1 ? 1 : 0));
    float initial_gap_2 = this->distance_between_fingers_2 / (1.0 + (is_even_fingers_2 ? 1 : 0));
    addCuttingPlanes(
      object->centerpoint, grasp_plane_vector, num_itr_1, num_itr_2, initial_gap_1,
      initial_gap_2);
  }
  /*! \brief Create cutting planes for side 1 */
  // PCLFunctions::computeCloudNormal(
  // object->cloud, object->cloud_normal, this->cloud_normal_radius);
}

/***************************************************************************//**
 * Function to add cutting planes if both sides of the end effector is odd or even.
 * This is because each plane contains finger points on both sides of the end effector
 * @param centerpoint centerpoint of gripper
 * @param plane_vector centerplane gripper
 * @param both_sides_even True if both sides of the end effector is even
 ******************************************************************************/
void FingerGripper::addCuttingPlanesEqualAligned(
  const Eigen::Vector4f centerpoint,
  const Eigen::Vector4f plane_vector,
  const bool both_sides_even)
{
  /*! \brief If both even, the initial gap is half the space between fingers, since middle plane is ignored
  If both odd, the initial gap is the space between fingers*/
  float initial_gap =
    (both_sides_even ? this->distance_between_fingers_1 / 2 : this->distance_between_fingers_1);
  bool side_1_max = this->num_fingers_side_1 > this->num_fingers_side_2;
  int max_fingers = (side_1_max ? this->num_fingers_side_1 : this->num_fingers_side_2);
  int min_fingers = (side_1_max ? this->num_fingers_side_2 : this->num_fingers_side_1);
  int num_itr = (max_fingers == 1 ? 0 : floor(max_fingers / 2) + 1);
  for (int row = 0, updown_toggle = 1; row < num_itr; row += updown_toggle ^= 1) {
    // for(int row = 0, updown_toggle = 0; row < num_itr; row += updown_toggle ^=1)
    float gap;
    gap =
      (updown_toggle ==
      0 ? 1 : -1) * (initial_gap + (row > 0 ? (row - 1) : 0) * this->distance_between_fingers_1);

    int plane_index = checkPlaneExists(gap);
    if (plane_index >= 0) {
      // std::cout << "plane exists" <<std::endl;
      // if(min_fingers > 0) // Plane still contains fingers on both side
      // {
      //   this->plane_1_index.push_back(plane_index);
      //   this->plane_2_index.push_back(plane_index);
      // }
      // else // Plane only contains fingers on the side with more fingers
      // {
      //   if(side_1_max)
      //   {
      //     this->plane_1_index.push_back(plane_index);
      //   }
      //   else{
      //     this->plane_2_index.push_back(plane_index);
      //   }
      // }

    } else {
      if (min_fingers > 0) {  // Plane still contains fingers on both side
        // std::cout << "Both sides have fingers on this plane" << std::endl;
        addPlane(gap, centerpoint, plane_vector, true, true);
      } else {  // Plane only contains fingers on the side with more fingers
        if (side_1_max) {
          // std::cout << "Side 1 have more fingers on this plane" << std::endl;
          addPlane(gap, centerpoint, plane_vector, true, false);
        } else {
          // std::cout << "Side 2 have more fingers on this plane" << std::endl;
          addPlane(gap, centerpoint, plane_vector, false, true);
        }
      }
      addPlane(gap, centerpoint, plane_vector, false, true);
    }
    // std::cout << "gap: " << gap << std::endl;

    min_fingers--;
    // std::cout << "cutting_plane_distances" << this->cutting_plane_distances.size() << std::endl;
    // std::cout << "grasp_samples" << this->grasp_samples.size() << std::endl;
    // std::cout << "plane_1_index" << this->plane_1_index.size() << std::endl;
    // std::cout << "plane_2_index" << this->plane_2_index.size() << std::endl;
  }
}

/***************************************************************************//**
 * Function to add cutting planes if both sides of the end effector is not both odd or even.
 * This is because each plane may or may not contain the finger on each side.
 * @param centerpoint centerpoint of gripper
 * @param plane_vector centerplane gripper
 * @param num_itr_1 Number of iterations to find the cutting planes for all fingers in side 1. Each iteration finds planes above and below the center plane with a given gap
 * @param num_itr_2 Number of iterations to find the cutting planes for all fingers in side 2. Each iteration finds planes above and below the center plane with a given gap
 * @param initial_gap_1 Initial Gap from the center plane to the next cutting plane for side 1
 * @param initial_gap_2 Initial Gap from the center plane to the next cutting plane for side 2
 ******************************************************************************/
void FingerGripper::addCuttingPlanes(
  const Eigen::Vector4f & centerpoint,
  const Eigen::Vector4f & plane_vector,
  int num_itr_1, int num_itr_2,
  float initial_gap_1, float initial_gap_2)
{
  for (int side_1 = 0, updown_toggle_1 = 1; side_1 < num_itr_1; side_1 += updown_toggle_1 ^= 1) {
    float gap;
    gap =
      (updown_toggle_1 == 0 ? 1 : -1) * (initial_gap_1 + side_1 * this->distance_between_fingers_1);
    int plane_index = checkPlaneExists(gap);
    if (plane_index >= 0) {
      this->plane_1_index.push_back(plane_index);
    } else {
      addPlane(gap, centerpoint, plane_vector, true, false);
    }
  }
  for (int side_2 = 0, updown_toggle_2 = 1; side_2 < num_itr_2; side_2 += updown_toggle_2 ^= 1) {
    float gap;
    gap =
      (updown_toggle_2 == 0 ? 1 : -1) * (initial_gap_2 + side_2 * this->distance_between_fingers_2);
    int plane_index = checkPlaneExists(gap);
    if (plane_index >= 0) {
      this->plane_2_index.push_back(plane_index);
    } else {
      addPlane(gap, centerpoint, plane_vector, false, true);
    }
  }
}

/***************************************************************************//**
 * Function check if the current plane created with a certain dist from the center plane
 * exists. This is to prevent multiple planes created if a finger on both sides share the same plane
 * Returns the index of the plane if it exists, if not, -1
 * @param dist Distance from the center plane to the current plane checked
 ******************************************************************************/

int FingerGripper::checkPlaneExists(float dist)
{
  // static std::mutex mutex;
  // std::lock_guard<std::mutex> lock(mutex);
  // std::cout << "dist: " << dist <<std::endl;
  std::vector<float>::iterator it = std::find(
    this->cutting_plane_distances.begin(), this->cutting_plane_distances.end(), dist);
  if (it != this->cutting_plane_distances.end()) {
    return it - this->cutting_plane_distances.begin();
  } else {
    return -1;
  }
}

/***************************************************************************//**
 * Function that creates a cutting plane and a graspPlaneSample instance that will
 * be added to the gripper instance's vector of plane samples
 * @param dist Distance from the center plane to the current plane checked
 * @param centerpoint Centerpoint of the plane
 * @param plane_vector Vector of the plane
 * @param inside_1 True if a finger in side 1 is on this plane
 * @param inside_2 True if a finger in side 2 is on this plane
 ******************************************************************************/

void FingerGripper::addPlane(
  float dist, Eigen::Vector4f centerpoint, Eigen::Vector4f plane_vector,
  bool inside_1, bool inside_2)
{
  // Get the vector representing the gripper direction
  Eigen::Vector3f plane_normal(plane_vector(0), plane_vector(1), plane_vector(2));
  // Get End effector finger clouds
  Eigen::Vector3f plane_normal_norm = plane_normal / plane_normal.norm();
  Eigen::Vector3f point_on_plane(centerpoint(0) + dist * plane_normal_norm(0), centerpoint(
      1) + dist * plane_normal_norm(1), centerpoint(2) + dist * plane_normal_norm(2));

  graspPlaneSample grasp_sample;
  grasp_sample.plane->values.resize(4);
  grasp_sample.plane_eigen(0) = grasp_sample.plane->values[0] = plane_vector(0);
  grasp_sample.plane_eigen(1) = grasp_sample.plane->values[1] = plane_vector(1);
  grasp_sample.plane_eigen(2) = grasp_sample.plane->values[2] = plane_vector(2);
  grasp_sample.plane_eigen(3) = grasp_sample.plane->values[3] =
    -((plane_vector(0) * point_on_plane(0)) + (plane_vector(1) * point_on_plane(1)) +
    (plane_vector(2) * point_on_plane(2)));

  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  int curr_index = this->grasp_samples.size();
  grasp_sample.plane_index = curr_index;
  this->cutting_plane_distances.push_back(dist);
  this->grasp_samples.push_back(std::make_shared<graspPlaneSample>(grasp_sample));
  if (inside_1) {
    this->plane_1_index.push_back(curr_index);
  }
  if (inside_2) {
    this->plane_2_index.push_back(curr_index);
  }
}

/***************************************************************************//**
 * Function to create a grasp cloud. Thisis done through SAC which finds points within the
 * grasp_plane_dist_limit. This create a strip of point cloud about the grasp plane.
 * @param object grasp object
 ******************************************************************************/
void FingerGripper::getGraspCloud(const std::shared_ptr<GraspObject> object)
{
  pcl::SampleConsensusModelPlane<pcl::PointNormal>::Ptr planeSAC(
    new pcl::SampleConsensusModelPlane<pcl::PointNormal>(
      object->cloud_normal));
  for (auto & sample : this->grasp_samples) {
    Eigen::Vector4f plane_vector(sample->plane->values[0], sample->plane->values[1],
      sample->plane->values[2], sample->plane->values[3]);
    pcl::PointIndices::Ptr grasp_plane_indices(new pcl::PointIndices);
    planeSAC->selectWithinDistance(
      plane_vector, this->grasp_plane_dist_limit,
      grasp_plane_indices->indices);
    PCLFunctions::extractInliersCloud<pcl::PointCloud<pcl::PointNormal>::Ptr,
      pcl::ExtractIndices<pcl::PointNormal>>(
      object->cloud_normal,
      grasp_plane_indices, sample->grasp_plane_ncloud);

    if (sample->grasp_plane_ncloud->points.size() > 0) {
      sample->plane_intersects_object = true;
    } else {
      sample->plane_intersects_object = false;
    }
  }
}

/***************************************************************************//**
 * Function that determines the start points on the center plane of the object depending
 * on the angle of the object with respect to the world. This start point will represent
 * the center of the finger cloud on that side of this plane.
 ******************************************************************************/
bool FingerGripper::getInitialSamplePoints(std::shared_ptr<GraspObject> object)
{
  // Object in the X axis
  if (object->objectWorldCosX > this->worldXAngleThreshold) {
    std::cout << "It is oriented with the X axis\n";
    for (auto & sample : this->grasp_samples) {
      // for(auto sample : this->grasp_samples)
      int first_point_index, second_point_index;
      float minY = std::numeric_limits<float>::max();
      float maxY = -std::numeric_limits<float>::min();
      if (sample->plane_intersects_object) {
        for (size_t i = 0; i < sample->grasp_plane_ncloud->points.size(); ++i) {
          if (sample->grasp_plane_ncloud->points[i].y < minY) {
            minY = sample->grasp_plane_ncloud->points[i].y;
            first_point_index = i;
          }
          if (sample->grasp_plane_ncloud->points[i].y > maxY) {
            maxY = sample->grasp_plane_ncloud->points[i].y;
            second_point_index = i;
          }
        }
        sample->sample_side_1->start_index = first_point_index;
        sample->sample_side_2->start_index = second_point_index;
      } else {
      }
    }
  } else {  // Object in the Y axis
    std::cout << "It is oriented with the Y axis\n";
    for (auto & sample : this->grasp_samples) {
      int first_point_index, second_point_index;
      float minX = std::numeric_limits<float>::max();
      float maxX = -std::numeric_limits<float>::min();
      for (size_t i = 0; i < sample->grasp_plane_ncloud->points.size(); ++i) {
        if (sample->grasp_plane_ncloud->points[i].x < minX) {
          minX = sample->grasp_plane_ncloud->points[i].x;
          first_point_index = i;
        }
        if (sample->grasp_plane_ncloud->points[i].x > maxX) {
          maxX = sample->grasp_plane_ncloud->points[i].x;
          second_point_index = i;
        }
      }

      fingerCloudSample sample_1;
      sample_1.start_index = first_point_index;
      fingerCloudSample sample_2;
      sample_2.start_index = second_point_index;
      sample->sample_side_1 = std::make_shared<fingerCloudSample>(sample_1);
      sample->sample_side_2 = std::make_shared<fingerCloudSample>(sample_2);
    }
  }
  return true;
}

/***************************************************************************//**
 * Function that gets the cluster of point clouds based on a certain radius from
 * a certain start index that has already been defined in another method. This cluster
 * of point clouds will repreesnt the gripper finger point cloud at that point on the plane
 ******************************************************************************/
void FingerGripper::getInitialSampleCloud(std::shared_ptr<GraspObject> object)
{
  std::vector<std::future<void>> futures;
  auto getAllRadiusPoints = [this](
    std::shared_ptr<graspPlaneSample> & sample,
    std::shared_ptr<GraspObject> & object) -> void
    {
      if (sample->plane_intersects_object) {
        PCLFunctions::getClosestPointsByRadius(
          sample->grasp_plane_ncloud->points[sample->sample_side_1->start_index],
          this->finger_thickness, object->cloud, object->cloud_normal,
          sample->sample_side_1->finger_cloud, sample->sample_side_1->finger_ncloud);

        PCLFunctions::getClosestPointsByRadius(
          sample->grasp_plane_ncloud->points[sample->sample_side_2->start_index],
          this->finger_thickness, object->cloud, object->cloud_normal,
          sample->sample_side_2->finger_cloud, sample->sample_side_2->finger_ncloud);
      } else {}
    };
  for (auto & sample : this->grasp_samples) {
    // if(sample->plane_intersects_object)
    // {
    //   PCLFunctions::getClosestPointsByRadius(sample->grasp_plane_ncloud->points[sample->sample_side_1->start_index],
    //     this->finger_thickness, object->cloud, object->cloud_normal,
    //     sample->sample_side_1->finger_cloud, sample->sample_side_1->finger_ncloud);

    //   PCLFunctions::getClosestPointsByRadius(sample->grasp_plane_ncloud->points[sample->sample_side_2->start_index],
    //     this->finger_thickness, object->cloud, object->cloud_normal,
    //     sample->sample_side_2->finger_cloud, sample->sample_side_2->finger_ncloud );
    // }
    // else{}
    futures.push_back(
      std::async(
        std::launch::async,
        getAllRadiusPoints,
        std::ref(sample),
        std::ref(object)));
  }
}

/***************************************************************************//**
 * Function to voxelize a sample cloud
 * This is used to voxelize finger sample cloud for easier traversal
 ******************************************************************************/
void FingerGripper::voxelizeSampleCloud()
{
  auto getAllRadiusPoints = [this](
    std::shared_ptr<graspPlaneSample> & sample) -> void
    {
      auto getEachRadiusPoints = [this](
        std::shared_ptr<fingerCloudSample> & sample_side) -> void
        {
          PCLFunctions::voxelizeCloud<pcl::PointCloud<pcl::PointNormal>::Ptr,
            pcl::VoxelGrid<pcl::PointNormal>>(
            sample_side->finger_ncloud,
            this->finger_thickness,
            sample_side->finger_nvoxel);
        };

      std::vector<std::future<void>> futures_inner;
      futures_inner.push_back(
        std::async(
          std::launch::async,
          getEachRadiusPoints,
          std::ref(sample->sample_side_1)));
      futures_inner.push_back(
        std::async(
          std::launch::async,
          getEachRadiusPoints,
          std::ref(sample->sample_side_2)));
    };
  std::vector<std::future<void>> futures;
  for (auto & sample : this->grasp_samples) {
    futures.push_back(
      std::async(
        std::launch::async,
        getAllRadiusPoints,
        std::ref(sample)));
  }
}


void FingerGripper::getFingerSamples(std::shared_ptr<GraspObject> const object)
{
  pcl::PointNormal centroid_point{
    object->centerpoint(0),
    object->centerpoint(1),
    object->centerpoint(2)};
  std::vector<std::future<void>> futures;
  auto getFingerSample = [this](
    std::shared_ptr<graspPlaneSample> & sample,
    pcl::PointNormal & centroid_point) -> void
    {
      auto getFingerSampleFromSide1 = [this](
        const pcl::PointNormal & point,
        pcl::PointNormal & centroid_point,
        std::shared_ptr<graspPlaneSample> & sample) -> void
        {
          Eigen::Vector3f curr1_vector(point.x, point.y, point.z);
          float centroid_dist = PCLFunctions::normalize(
            pcl::geometry::distance(point, centroid_point),
            sample->sample_side_1->centroid_dist_min, sample->sample_side_1->centroid_dist_max);
          float grasp_plane_dist = PCLFunctions::normalize(
            PCLFunctions::pointToPlane(sample->plane_eigen, point),
            sample->sample_side_1->grasp_plane_dist_min,
            sample->sample_side_1->grasp_plane_dist_max);
          float curvature = PCLFunctions::normalize(
            point.curvature,
            sample->sample_side_1->curvature_min, sample->sample_side_1->curvature_max);
          singleFinger finger(point, centroid_dist, grasp_plane_dist, curvature,
            sample->plane_index);
          static std::mutex mutex;
          std::lock_guard<std::mutex> lock(mutex);
          sample->sample_side_1->finger_samples.push_back(std::make_shared<singleFinger>(finger));
        };
      auto getFingerSampleFromSide2 = [this](
        const pcl::PointNormal & point,
        pcl::PointNormal & centroid_point,
        std::shared_ptr<graspPlaneSample> & sample) -> void
        {
          Eigen::Vector3f curr2_vector(point.x, point.y, point.z);
          float centroid_dist = PCLFunctions::normalize(
            pcl::geometry::distance(point, centroid_point),
            sample->sample_side_2->centroid_dist_min, sample->sample_side_2->centroid_dist_max);
          float grasp_plane_dist = PCLFunctions::normalize(
            PCLFunctions::pointToPlane(sample->plane_eigen, point),
            sample->sample_side_2->grasp_plane_dist_min,
            sample->sample_side_2->grasp_plane_dist_max);
          float curvature = PCLFunctions::normalize(
            point.curvature,
            sample->sample_side_2->curvature_min, sample->sample_side_2->curvature_max);
          singleFinger finger(point, centroid_dist, grasp_plane_dist, curvature,
            sample->plane_index);
          static std::mutex mutex;
          std::lock_guard<std::mutex> lock(mutex);
          sample->sample_side_2->finger_samples.push_back(std::make_shared<singleFinger>(finger));
        };
      std::vector<std::future<void>> futures_inner1;
      if (sample->plane_intersects_object) {
        for (auto const & curr1_point : sample->sample_side_1->finger_nvoxel->points) {
          futures_inner1.push_back(
            std::async(
              std::launch::async,
              getFingerSampleFromSide1,
              std::ref(curr1_point),
              std::ref(centroid_point),
              std::ref(sample)));
        }
        std::vector<std::future<void>> futures_inner2;
        for (auto const & curr2_point : sample->sample_side_2->finger_nvoxel->points) {
          futures_inner2.push_back(
            std::async(
              std::launch::async,
              getFingerSampleFromSide2,
              std::ref(curr2_point),
              std::ref(centroid_point),
              std::ref(sample)));
        }
      } else {
      }
    };
  for (auto & sample : this->grasp_samples) {
    futures.push_back(
      std::async(
        std::launch::async,
        getFingerSample,
        std::ref(sample),
        std::ref(centroid_point)));
  }
}

/***************************************************************************//**
 * Now that we have generated the various fingerCloudSamples for each plane
 * (2 per plane), we now choose the correct finger clouds that corresponds to
 * the multifinger gripper.
 ******************************************************************************/
void FingerGripper::getGripperClusters()
{
  for (size_t i = 0; i < this->grasp_samples.size(); i++) {
    // grasp planes that do not intersect the object would not contain grasp clouds.
    if (this->grasp_samples[i]->plane_intersects_object) {
      // Check if this grasp plane contains finger on side 1
      if (std::find(
          this->plane_1_index.begin(), this->plane_1_index.end(),
          i) != this->plane_1_index.end())
      {
        // Finger cloud on side 1 on this plane is part of the gripper
        this->gripper_clusters.push_back(this->grasp_samples[i]->sample_side_1->finger_samples);
      }
      // Check if this grasp plane contains finger on side 2
      if (std::find(
          this->plane_2_index.begin(), this->plane_2_index.end(),
          i) != this->plane_2_index.end())
      {
        // Finger cloud on side 2 on this plane is part of the gripper
        this->gripper_clusters.push_back(this->grasp_samples[i]->sample_side_2->finger_samples);
      }
    } else {}
  }
}

/***************************************************************************//**
 * Function to create all possible gripper configurations. Returns a vector
 * containing the configurations.

 * @param object Object to be grasped
 * @param world_collision_object Collision object representing the world
 ******************************************************************************/

std::vector<std::shared_ptr<multiFingerGripper>> FingerGripper::getAllGripperConfigs(
  const std::shared_ptr<GraspObject> object,
  const std::shared_ptr<fcl::CollisionObject<float>> world_collision_object)
{
  std::vector<std::shared_ptr<multiFingerGripper>> valid_open_gripper_configs;
  // Query the gripping points at the center cutting plane
  if (this->grasp_samples[0]->plane_intersects_object) {
    for (auto & finger_sample_1 : this->grasp_samples[0]->sample_side_1->finger_samples) {
      for (auto & finger_sample_2 : this->grasp_samples[0]->sample_side_2->finger_samples) {
        Eigen::Vector3f centerpoint_side1_vector(finger_sample_1->finger_point.x,
          finger_sample_1->finger_point.y,
          finger_sample_1->finger_point.z);
        Eigen::Vector3f centerpoint_side2_vector(finger_sample_2->finger_point.x,
          finger_sample_2->finger_point.y,
          finger_sample_2->finger_point.z);
        // Get the vector representing the grasping direction
        Eigen::Vector3f grasp_direction = Eigen::ParametrizedLine<float, 3>::Through(
          centerpoint_side1_vector, centerpoint_side2_vector).direction();
        float grasp_plane_angle_cos_ = 1e-8 + std::abs(
          (grasp_direction.dot(this->center_cutting_plane_normal)) /
          (grasp_direction.norm() * this->center_cutting_plane_normal.norm()));

        Eigen::Vector3f curr1_normal(finger_sample_1->finger_point.normal_x,
          finger_sample_1->finger_point.normal_y,
          finger_sample_1->finger_point.normal_z);
        finger_sample_1->angle_cos = 1e-8 +
          std::abs(
          (grasp_direction.dot(curr1_normal)) /
          (grasp_direction.norm() * curr1_normal.norm()));

        Eigen::Vector3f curr2_normal(finger_sample_2->finger_point.normal_x,
          finger_sample_2->finger_point.normal_y,
          finger_sample_2->finger_point.normal_z);
        finger_sample_2->angle_cos = 1e-8 +
          std::abs(
          (grasp_direction.dot(curr2_normal)) /
          (grasp_direction.norm() * curr1_normal.norm()));

        // TEST
        pcl::PointXYZ parallel_object_vector = pcl::PointXYZ(
          object->eigenvectors.col(0)(0) - object->centerpoint(0),
          object->eigenvectors.col(0)(1) - object->centerpoint(1), object->eigenvectors.col(0)(
            2) - object->centerpoint(2));
        Eigen::Vector3f point_on_plane;
        point_on_plane(0) = (finger_sample_1->finger_point.x + finger_sample_2->finger_point.x) / 2;
        point_on_plane(1) = (finger_sample_1->finger_point.y + finger_sample_2->finger_point.y) / 2;
        point_on_plane(2) = (finger_sample_1->finger_point.z + finger_sample_2->finger_point.z) / 2;
        pcl::ModelCoefficients::Ptr parallel_object_plane(new pcl::ModelCoefficients);
        parallel_object_plane->values.resize(4);
        parallel_object_plane->values[0] = parallel_object_vector.x;
        parallel_object_plane->values[1] = parallel_object_vector.y;
        parallel_object_plane->values[2] = parallel_object_vector.z;
        parallel_object_plane->values[3] = -((parallel_object_vector.x * point_on_plane(0)) +
          (parallel_object_vector.y * point_on_plane(1)) +
          (parallel_object_vector.z * point_on_plane(2)));
        Eigen::Vector3f perpendicular_grasp_direction = getPerpendicularVectorInPlane(
          grasp_direction, parallel_object_plane);
        Eigen::Vector3f perpendicular_grasp_direction_normalized = perpendicular_grasp_direction /
          perpendicular_grasp_direction.norm();
        // END TEST

        // Get End effector finger clouds
        Eigen::Vector3f grasp_direction_normalized = grasp_direction / grasp_direction.norm();
        // Get the centerpoint vector of the grasp
        Eigen::Vector3f side1_2_centerpoint_vector((centerpoint_side1_vector(
            0) + centerpoint_side2_vector(0)) / 2,
          (centerpoint_side1_vector(1) + centerpoint_side2_vector(1)) / 2,
          (centerpoint_side1_vector(2) + centerpoint_side2_vector(2)) / 2);

        // Get the initial finger positions in the open configuration from the center plane.
        // This may or may not be used depending on number of fingers
        Eigen::Vector3f open_center_finger_1 = side1_2_centerpoint_vector -
          (this->gripper_stroke / 2) * grasp_direction_normalized;
        Eigen::Vector3f open_center_finger_2 = side1_2_centerpoint_vector +
          (this->gripper_stroke / 2) * grasp_direction_normalized;
        std::shared_ptr<multiFingerGripper> gripper_sample = generateGripperOpenConfig(
          world_collision_object, finger_sample_1, finger_sample_2,
          open_center_finger_1, open_center_finger_2, perpendicular_grasp_direction_normalized,
          grasp_direction);
        gripper_sample->grasp_plane_angle_cos = grasp_plane_angle_cos_;
        if (!gripper_sample->collides_with_world) {
          valid_open_gripper_configs.push_back(gripper_sample);
        }
      }
    }
  }
  std::cout << "end getAllGripperConfigs: " << std::endl;
  return valid_open_gripper_configs;
}


/***************************************************************************//**
 * Function to create the finger gripper configuration.
 * Returns the created finger gripper sample
 * @param world_collision_object Collision object representing the world
 * @param closed_center_finger_1 Finger representation of the closed configuration for the center finger in side 1.
 * @param closed_center_finger_2 Finger representation of the closed configuration for the center finger in side 2.
 * @param open_center_finger_1 Finger representation of the open configuration for the center finger in side 1.
 * @param open_center_finger_2 Finger representation of the open configuration for the center finger in side 2.
 * @param plane_normal_normalized Normal vector of the center plane.
 ******************************************************************************/
std::shared_ptr<multiFingerGripper> FingerGripper::generateGripperOpenConfig(
  const std::shared_ptr<fcl::CollisionObject<float>> world_collision_object,
  const std::shared_ptr<singleFinger> closed_center_finger_1,
  const std::shared_ptr<singleFinger> closed_center_finger_2,
  const Eigen::Vector3f open_center_finger_1,
  const Eigen::Vector3f open_center_finger_2,
  Eigen::Vector3f plane_normal_normalized, Eigen::Vector3f grasp_direction)
{
  // Create an instance of the multifinger gripper.
  multiFingerGripper gripper(closed_center_finger_1, closed_center_finger_2);
  gripper.collides_with_world = false;
  bool is_even_1 = this->num_fingers_side_1 % 2 == 0;
  bool is_even_2 = this->num_fingers_side_2 % 2 == 0;

  /* Assuming the gripper is symmetrical, if a side as an odd number of fingers, the center finger
     of that side must correspond to the point on the center plane. Thus pushback the open finger
     position and closed finger position on the center plane */

  if (!is_even_1) {
    gripper.closed_fingers_1.push_back(closed_center_finger_1);
    gripper.open_fingers_1.push_back(open_center_finger_1);
  }
  if (!is_even_2) {
    gripper.closed_fingers_2.push_back(closed_center_finger_2);
    gripper.open_fingers_2.push_back(open_center_finger_2);
  }

  /*Get the initial gap between on each side with respect to the center plane.
    For an odd number of fingers the initial gap is half the distance between finger
    (since the point on the center plane is taken)
    For an even number of fingers the initial gap is the distance between fingers
    (Since the point on center plane is not taken.)*/

  float initial_gap_1 =
    (is_even_1 ? this->distance_between_fingers_1 / 2 : this->distance_between_fingers_1);
  float initial_gap_2 =
    (is_even_2 ? this->distance_between_fingers_2 / 2 : this->distance_between_fingers_2);

  /* Get the number of iterations to create the entire finger gripper. Each iteration creates a
     finger above and below the center plane with the given distance. thus for a three finger
     side, for example, it only requires one iteration. The center finger is taken, plus
     one iteration to create the top and bottom finger. */

  int num_itr_1 = (this->num_fingers_side_1 == 1 ? 0 : floor(this->num_fingers_side_1 / 2));
  int num_itr_2 = (this->num_fingers_side_2 == 1 ? 0 : floor(this->num_fingers_side_2 / 2));

  // Iterate through the points on side 1
  for (int side_1 = 0, updown_toggle_1 = 1; side_1 < num_itr_1; side_1 += updown_toggle_1 ^= 1) {
    // Define the distance the current finger is from the center point.
    float gap1;
    gap1 = (updown_toggle_1 == 0 ? 1 : -1) * (initial_gap_1 + side_1 *
      this->distance_between_fingers_1);

    // Get the coordinates of the current finger in the open configuration
    Eigen::Vector3f finger_1_temp = open_center_finger_1 + gap1 * plane_normal_normalized;
    gripper.open_fingers_1.push_back(finger_1_temp);

    /* Check if this current open configuration for the finger collides with the world,
       since the end effector will approach the object in its open state.*/

    if (checkFingerCollision(finger_1_temp, world_collision_object)) {
      gripper.collides_with_world = true;
    }

    /* Ranking of grasp quality involves the positions of the gripper fingers ON the object,
       so we next need to find the finger point on the object corresponding to the current
       open configuration of the finger */

    // First, find the plane in which we need to find this point
    int plane_index;
    float plane_dist_diff = std::numeric_limits<float>::max();

    /* We use the gap from the open configuration finger to compare with the cutting plane
       gaps to find the most identical distance between the center plane to that plane, and
       that will be the plane index that corresponds to the correct plane */

    for (size_t plane_index_ = 0; plane_index_ < this->cutting_plane_distances.size();
      plane_index_++)
    {
      if (std::abs(cutting_plane_distances[plane_index_] - gap1) < plane_dist_diff) {
        plane_dist_diff = std::abs(cutting_plane_distances[plane_index_] - gap1);
        plane_index = plane_index_;
      }
    }

    /* Now that we find the plane, we need to find the point near that plane.
     this is done using the kdtree search function */
    pcl::PointNormal finger_1_point(finger_1_temp(0), finger_1_temp(1), finger_1_temp(2));
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    // We only need to find 1 neighbour. can be changed later
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    // We already have a set of voxelized points on the correspoinding side, and correspoinding plane.
    kdtree.setInputCloud(this->grasp_samples[plane_index]->sample_side_1->finger_nvoxel);

    // std::cout << "Open Point at: " << finger_1_point.x <<
    // "," << finger_1_point.y << "," << finger_1_point.z << std::endl;

    if (kdtree.nearestKSearch(finger_1_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
      // pcl::PointNormal finger_1_contactpoint;
      // finger_1_contactpoint = this->
      // grasp_samples[plane_index].sample_side_1.finger_nvoxel->points[pointIdxNKNSearch[0]];
      // std::cout << "Real Point at: " << finger_1_contactpoint.x << "," <<
      //  finger_1_contactpoint.y << "," << finger_1_contactpoint.z << std::endl;
      gripper.closed_fingers_1.push_back(
        this->grasp_samples[plane_index]->sample_side_1->finger_samples[pointIdxNKNSearch[0]]);
      gripper.closed_fingers_1_index.push_back(pointIdxNKNSearch[0]);
    }
  }

  // Iterate through the points on side 2
  for (int side_2 = 0, updown_toggle_2 = 1; side_2 < num_itr_2; side_2 += updown_toggle_2 ^= 1) {
    float gap2;
    gap2 =
      (updown_toggle_2 == 0 ? 1 : -1) * (initial_gap_2 + side_2 * this->distance_between_fingers_2);
    Eigen::Vector3f finger_2_temp = open_center_finger_2 + gap2 * plane_normal_normalized;
    // std::cout << "Finger Side 2: " << side_2 << std::endl;
    // std::cout << "updown_toggle_2: " << updown_toggle_2 << std::endl;
    // std::cout << "gap2: " << gap2 << std::endl;
    if (checkFingerCollision(finger_2_temp, world_collision_object)) {
      gripper.collides_with_world = true;
    }
    gripper.open_fingers_2.push_back(finger_2_temp);

    int plane_index_2;
    float plane_dist_diff_2 = std::numeric_limits<float>::max();
    // std::cout << "Curr gap2: " << gap2 << std::endl;

    for (size_t plane_index_2_ = 0; plane_index_2_ < this->cutting_plane_distances.size();
      plane_index_2_++)
    {
      if (std::abs(cutting_plane_distances[plane_index_2_] - gap2) < plane_dist_diff_2) {
        plane_dist_diff_2 = std::abs(cutting_plane_distances[plane_index_2_] - gap2);
        plane_index_2 = plane_index_2_;
      }
    }
    pcl::PointNormal finger_2_point(finger_2_temp(0), finger_2_temp(1), finger_2_temp(2));
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.setInputCloud(this->grasp_samples[plane_index_2]->sample_side_2->finger_nvoxel);

    if (kdtree.nearestKSearch(finger_2_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
      // pcl::PointNormal finger_2_contactpoint;
      // finger_2_contactpoint = this->
      // grasp_samples[plane_index_2].sample_side_2.finger_nvoxel->points[pointIdxNKNSearch[0]];
      gripper.closed_fingers_2.push_back(
        this->grasp_samples[plane_index_2]->sample_side_2->finger_samples[pointIdxNKNSearch[0]]);
      gripper.closed_fingers_2_index.push_back(pointIdxNKNSearch[0]);
    }
  }
  for (auto & finger : gripper.closed_fingers_1) {
    Eigen::Vector3f finger_normal(finger->finger_point.normal_x,
      finger->finger_point.normal_y,
      finger->finger_point.normal_z);
    finger->angle_cos = 1e-8 +
      std::abs(
      (grasp_direction.dot(finger_normal)) /
      (grasp_direction.norm() * finger_normal.norm()));
  }

  for (auto & finger : gripper.closed_fingers_2) {
    Eigen::Vector3f finger_normal(finger->finger_point.normal_x,
      finger->finger_point.normal_y,
      finger->finger_point.normal_z);
    finger->angle_cos = 1e-8 +
      std::abs(
      (grasp_direction.dot(finger_normal)) /
      (grasp_direction.norm() * finger_normal.norm()));
  }

  return std::make_shared<multiFingerGripper>(gripper);
}

/***************************************************************************//**
 * Function to check the finger collision with the world
 * Returns true if the finger collides with the world.
 * @param finger_point Coordinates for the current finger
 * @param world_collision_object Collision object representing the world
 ******************************************************************************/
bool FingerGripper::checkFingerCollision(
  const Eigen::Vector3f finger_point,
  const std::shared_ptr<fcl::CollisionObject<float>> world_collision_object)
{
  auto finger_shape = std::make_shared<fcl::Sphere<float>>(this->finger_thickness / 2);
  fcl::Transform3<float> finger_transform = fcl::Transform3<float>::Identity();
  finger_transform.translation() << finger_point(0), finger_point(1), finger_point(2);
  fcl::CollisionObject<float> finger(finger_shape, finger_transform);
  std::shared_ptr<fcl::CollisionObject<float>> finger_ptr =
    std::make_shared<fcl::CollisionObject<float>>(finger);

  fcl::CollisionRequest<float> request;
  request.enable_contact = true;
  // num_max_contacts_ = 1, //The maximum number of contacts will return.
  // enable_contact_ = false, //whether the contact information (normal, penetration depth and
  // contact position) will return
  // num_max_cost_sources_ = 1, // The maximum number of cost sources will return.
  // enable_cost_ = false, //whether the cost sources will be computed
  // use_approximate_cost_ = true // whether the cost computation is approximated

  // result will be returned via the collision result structure
  fcl::CollisionResult<float> result;
  fcl::collide(world_collision_object.get(), finger_ptr.get(), request, result);
  return result.isCollision();
}


/***************************************************************************//**
 * Function that gets the maximum and minimum values of certain grasp ranking attributes within
 * a point cloud Each plane will have its own independant maximum and minimum values, we only
 * compare between the 2 voxelized clouds in each plane
 * @param object Grasp object.
 ******************************************************************************/
void FingerGripper::getMaxMinValues(std::shared_ptr<GraspObject> object)
{
  pcl::PointNormal centroid_point{
    object->centerpoint(0),
    object->centerpoint(1),
    object->centerpoint(2)};

  int counter = 0;

  for (auto & sample : this->grasp_samples) {
    if (sample->plane_intersects_object) {
      for (auto & point : sample->sample_side_1->finger_nvoxel->points) {
        float centroid_distance, grasp_plane_distance, curvature;
        centroid_distance = pcl::geometry::distance(point, centroid_point);
        grasp_plane_distance = PCLFunctions::pointToPlane(sample->plane_eigen, point);
        curvature = point.curvature;

        if (curvature < sample->sample_side_1->curvature_min) {
          sample->sample_side_1->curvature_min = curvature;
        }
        if (curvature > sample->sample_side_1->curvature_max) {
          sample->sample_side_1->curvature_max = curvature;
        }
        if (centroid_distance < sample->sample_side_1->centroid_dist_min) {
          sample->sample_side_1->centroid_dist_min = centroid_distance;
        }
        if (centroid_distance > sample->sample_side_1->centroid_dist_max) {
          sample->sample_side_1->centroid_dist_max = centroid_distance;
        }
        if (grasp_plane_distance < sample->sample_side_1->grasp_plane_dist_min) {
          sample->sample_side_1->grasp_plane_dist_min = grasp_plane_distance;
        }
        if (grasp_plane_distance > sample->sample_side_1->grasp_plane_dist_max) {
          sample->sample_side_1->grasp_plane_dist_max = grasp_plane_distance;
        }
      }

      for (auto & point : sample->sample_side_2->finger_nvoxel->points) {
        float centroid_distance, grasp_plane_distance, curvature;
        centroid_distance = pcl::geometry::distance(point, centroid_point);
        grasp_plane_distance = PCLFunctions::pointToPlane(sample->plane_eigen, point);
        curvature = point.curvature;

        if (curvature < sample->sample_side_2->curvature_min) {
          sample->sample_side_2->curvature_min = curvature;
        }
        if (curvature > sample->sample_side_2->curvature_max) {
          sample->sample_side_2->curvature_max = curvature;
        }
        if (centroid_distance < sample->sample_side_2->centroid_dist_min) {
          sample->sample_side_2->centroid_dist_min = centroid_distance;
        }
        if (centroid_distance > sample->sample_side_2->centroid_dist_max) {
          sample->sample_side_2->centroid_dist_max = centroid_distance;
        }
        if (grasp_plane_distance < sample->sample_side_2->grasp_plane_dist_min) {
          sample->sample_side_2->grasp_plane_dist_min = grasp_plane_distance;
        }
        if (grasp_plane_distance > sample->sample_side_2->grasp_plane_dist_max) {
          sample->sample_side_2->grasp_plane_dist_max = grasp_plane_distance;
        }
      }
    }
    counter++;
  }
}

/***************************************************************************//**
 * Function that gets the rank of each multiFingerGripper instance, and sort them into a vector
 * of decreasing rank. This function also populates the GraspMethod message which will be sent
 * To the grasp execution
 * @param input_vector Unsorted vector of gripper instances
 * @param grasp_method GraspMethod output for grasp execution
 ******************************************************************************/
std::vector<std::shared_ptr<multiFingerGripper>> FingerGripper::getAllRanks(
  std::vector<std::shared_ptr<multiFingerGripper>> input_vector,
  emd_msgs::msg::GraspMethod * grasp_method)
{
  std::vector<std::shared_ptr<multiFingerGripper>> sorted_gripper_ranks;
  for (auto gripper : input_vector) {
    getGripperRank(gripper);
    std::vector<geometry_msgs::msg::PoseStamped>::iterator grasps_it;
    std::vector<std::shared_ptr<multiFingerGripper>>::iterator contacts_it;
    size_t rank;
    for (rank = 0,
      grasps_it = grasp_method->grasp_poses.begin(), contacts_it = sorted_gripper_ranks.begin();
      rank < grasp_method->grasp_ranks.size(); ++rank, ++grasps_it, ++contacts_it)
    {
      if (gripper->rank > grasp_method->grasp_ranks[rank]) {
        grasp_method->grasp_ranks.insert(grasp_method->grasp_ranks.begin() + rank, gripper->rank);
        grasp_method->grasp_poses.insert(grasps_it, gripper->pose);
        sorted_gripper_ranks.insert(contacts_it, gripper);
        break;
      }
    }
  }
  return sorted_gripper_ranks;
}

/***************************************************************************//**
 * Get the current rank of a multifinger gripper. This is an extension of the implementation
 * by the research paper.
 * @param gripper Target gripper
 ******************************************************************************/
void FingerGripper::getGripperRank(std::shared_ptr<multiFingerGripper> gripper)
{
  float curvature_sum = 0;
  float grasp_plane_dist_sum = 0;

  for (auto & finger_1 : gripper->closed_fingers_1) {
    curvature_sum += finger_1->curvature;
    grasp_plane_dist_sum += finger_1->grasp_plane_dist;

  }

  for (auto & finger_2 : gripper->closed_fingers_2) {
    curvature_sum += finger_2->curvature;
    grasp_plane_dist_sum += finger_2->grasp_plane_dist;
  }

  float rank_1 = (gripper->closed_fingers_1.size() + gripper->closed_fingers_2.size()) -
    grasp_plane_dist_sum -
    (gripper->grasp_plane_angle_cos - 0.2) * 10.0;
  // More to add for rank 2
  float rank_2 = (gripper->closed_fingers_1.size() + gripper->closed_fingers_2.size()) -
    curvature_sum;
  gripper->rank = this->grasp_quality_weight1 * rank_1 + this->grasp_quality_weight2 * rank_2;
}

/***************************************************************************//**
 * Function that gets the grasp pose of a target gripper based on the object.
 * Current implementation of the gripper pose orientation is in relation to the
 * Object orientation, which should be changed in the future.
 * @param gripper Target gripper
 * @param object Target object
 ******************************************************************************/
void FingerGripper::getGraspPose(
  std::shared_ptr<multiFingerGripper> gripper,
  std::shared_ptr<GraspObject> object)
{
  geometry_msgs::msg::PoseStamped result_pose;
  result_pose.pose.position.x = gripper->gripper_palm_center.x;
  result_pose.pose.position.y = gripper->gripper_palm_center.y;
  result_pose.pose.position.z = gripper->gripper_palm_center.z;


  tf2::Matrix3x3 rotation_matrix(object->affine_matrix(0, 0), object->affine_matrix(
      1,
      0), object->affine_matrix(
      2, 0),
    object->affine_matrix(0, 1), object->affine_matrix(1, 1), object->affine_matrix(2, 1),
    object->affine_matrix(0, 2), object->affine_matrix(1, 2), object->affine_matrix(2, 2));

  double r, p, y;
  rotation_matrix.getRPY(r, p, y);
  tf2::Quaternion quaternion_;
  quaternion_.setRPY(0, 0, y);

  result_pose.pose.orientation.x = quaternion_.x();
  result_pose.pose.orientation.y = quaternion_.y();
  result_pose.pose.orientation.z = quaternion_.z();
  result_pose.pose.orientation.w = quaternion_.w();

  const auto clock = std::chrono::system_clock::now();
  result_pose.header.frame_id = object->object_frame;
  result_pose.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(
    clock.time_since_epoch()).count();
  gripper->pose = result_pose;
}

/***************************************************************************//**
 * Get a vector that is perpendicular to another vector that is on the same plane
 * Equations below is a culmination of simultaneous equations
 * @param target_vector Target vector to be perpendicular to
 * @param plane Target plane that both vectors need to be on.
 ******************************************************************************/
Eigen::Vector3f FingerGripper::getPerpendicularVectorInPlane(
  Eigen::Vector3f target_vector,
  pcl::ModelCoefficients::Ptr plane)
{
  Eigen::Vector4f plane_vector(plane->values[0], plane->values[1], plane->values[2],
    plane->values[3]);
  Eigen::Vector3f plane_normal_vector(plane->values[0], plane->values[1], plane->values[2]);
  float z_bar = plane_normal_vector(0) * target_vector(2) - target_vector(0) *
    plane_normal_vector(2);
  float y_bar = plane_normal_vector(0) * target_vector(1) - target_vector(0) *
    plane_normal_vector(1);
  float y_numerator = (-plane_vector(3) * plane_normal_vector(0) * z_bar);
  float y_denominator = (plane_vector(0) * plane_normal_vector(2) * y_bar) -
    (plane_normal_vector(1) * z_bar) +
    (plane_vector(1) * plane_normal_vector(0) * z_bar) -
    (plane_vector(2) * plane_normal_vector(0) * y_bar);
  float y_temp = y_numerator / y_denominator;
  float x_temp = y_temp *
    (((plane_normal_vector(2) * y_bar) / (plane_normal_vector(0) * z_bar)) -
    (plane_normal_vector(1) / plane_normal_vector(0)));
  float z_temp = y_temp * (-y_bar / z_bar);
  Eigen::Vector3f output_vector(x_temp, y_temp, z_temp);
  return output_vector;
}

void FingerGripper::resetVariables()
{
}


void FingerGripper::visualizeGrasps(pcl::visualization::PCLVisualizer::Ptr viewer)
{
  // viewer->addPointCloud(object->cloud, "object_cloud");
  // viewer->addPlane(*(this->grasp_samples[0]->plane), "cenerplane", 0);
  // viewer->spin();
  // viewer->close();
  // viewer->removeAllShapes();

  // for(size_t i = 0 ; i < this->plane_1_index.size(); i++)
  // {
  //   std::cout << "View plane index at: " << this->plane_1_index[i] << std::endl;
  //   pcl::PointXYZ check_vector = pcl::PointXYZ(
  //   object->eigenvectors.col(0)(0) - object->centerpoint(0),
  //   object->eigenvectors.col(0)(1) - object->centerpoint(1),
  //   object->eigenvectors.col(0)(2) - object->centerpoint(2));
  //   viewer->addPlane(
  //   *(this->grasp_samples[plane_1_index[i]]->plane), "plane_" + std::to_string(i), 0);
  // }
  // viewer->spin();
  // viewer->close();
  // viewer->removeAllShapes();
  // viewer->removeAllPointClouds();
  // viewer->addPointCloud(object->cloud, "object_cloud");

  // for(size_t i = 0 ; i < this->plane_2_index.size(); i++)
  // {

  //   pcl::PointXYZ check_vector = pcl::PointXYZ(
  //   object->eigenvectors.col(0)(0) - object->centerpoint(0),
  //   object->eigenvectors.col(0)(1) - object->centerpoint(1),
  //   object->eigenvectors.col(0)(2) - object->centerpoint(2));
  //   viewer->addPlane(
  //    *(this->grasp_samples[plane_2_index[i]]->plane), "plane_" + std::to_string(i), 0);
  // }
  // viewer->spin();
  // viewer->close();
  // viewer->removeAllShapes();
  // viewer->removeAllPointClouds();
  // viewer->addPointCloud(object->cloud, "object_cloud");

  // for(size_t i = 0 ; i < this->grasp_samples.size(); i++)
  // {
  //   viewer->addPointCloud<pcl::PointNormal>(
  //   this->grasp_samples[i]->grasp_plane_ncloud,"cloud_" + std::to_string(i));
  // }
  // viewer->spin();
  // viewer->close();
  // viewer->removeAllShapes();
  // viewer->removeAllPointClouds();
  // viewer->addPointCloud(object->cloud, "object_cloud");

  // for(size_t i = 0 ; i < this->grasp_samples.size(); i++)
  // {
  //   if(this->grasp_samples[i]->plane_intersects_object)
  //   {
  //     if(std::find(
  //     this->plane_1_index.begin(), this->plane_1_index.end(), i) != this->plane_1_index.end())
  //     {
  //       int index = this->grasp_samples[i]->sample_side_1->start_index;
  //       viewer->addSphere(
  //       this->grasp_samples[i]->grasp_plane_ncloud->points[index],
  //       0.005, "finger_point_1" + std::to_string(i));
  //     }
  //     if(std::find(this->plane_2_index.begin(),
  //     this->plane_2_index.end(), i) != this->plane_2_index.end())
  //     {
  //       int index = this->grasp_samples[i]->sample_side_2->start_index;
  //       viewer->addSphere(
  //       this->grasp_samples[i]->grasp_plane_ncloud->points[index],
  //       0.005, "finger_point_2" + std::to_string(i));
  //     }
  //   }

  // }
  // viewer->spin();
  // viewer->close();
  // viewer->removeAllShapes();
  // viewer->removeAllPointClouds();

  // for(size_t i = 0 ; i < this->grasp_samples.size(); i++)
  // {
  //   viewer->addPointCloud<pcl::PointXYZRGB>(
  //   this->grasp_samples[i].sample_side_1.finger_cloud,"cloud1_" + std::to_string(i));
  //   viewer->addPointCloud<pcl::PointXYZRGB>(
  //   this->grasp_samples[i].sample_side_2.finger_cloud,"cloud2_" + std::to_string(i));
  // }

  // viewer->spin();
  // viewer->close();
  // viewer->removeAllShapes();
  // viewer->removeAllPointClouds();

  // for(size_t i = 0 ; i < this->grasp_samples.size(); i++)
  // {
  //   viewer->addPointCloud<pcl::PointNormal>(
  //   this->grasp_samples[i].sample_side_1.finger_ncloud,"cloud1_" + std::to_string(i));
  //   viewer->addPointCloud<pcl::PointNormal>(
  //   this->grasp_samples[i].sample_side_2.finger_ncloud,"cloud2_" + std::to_string(i));
  // }
  // viewer->spin();
  // viewer->close();
  // viewer->removeAllShapes();
  // viewer->removeAllPointClouds();

  // for(size_t i = 0 ; i < this->grasp_samples.size(); i++)
  // {
  //   viewer->addPointCloud<pcl::PointNormal>(
  //   this->grasp_samples[i].sample_side_1.finger_nvoxel,"cloud1_" + std::to_string(i));
  //   viewer->addPointCloud<pcl::PointNormal>(
  //   this->grasp_samples[i].sample_side_2.finger_nvoxel,"cloud2_" + std::to_string(i));
  // }

  // viewer->spin();
  // viewer->close();
  // viewer->removeAllShapes();

  // viewer->addPointCloud(object->cloud, "object_cloud");

  // std::cout << "Side 1 num: " << this ->num_fingers_side_1 << std::endl;
  // std::cout << "Side 2 num: " << this ->num_fingers_side_2 << std::endl;

  // std::cout << "Valid open gripper configs size: " <<
  // valid_open_gripper_configs.size() << std::endl;

  // for(size_t i = 0 ; i < this->grasp_samples.size(); i++)
  // {
  //   viewer->addPointCloud<pcl::PointNormal>(
  //   this->grasp_samples[i]->sample_side_1->finger_ncloud,"cloud1_" + std::to_string(i));
  //   viewer->addPointCloud<pcl::PointNormal>(
  //   this->grasp_samples[i]->sample_side_2->finger_ncloud,"cloud2_" + std::to_string(i));
  // }
  std::cout << "Visualizing " << this->sorted_gripper_configs.size() << " grasps" << std::endl;
  for (auto const & multigripper : this->sorted_gripper_configs) {
    for (size_t cs_1 = 0; cs_1 < multigripper->closed_fingers_1.size(); cs_1++) {
      viewer->addSphere(
        multigripper->closed_fingers_1[cs_1]->
        finger_point, 0.01, 1.0, 0, 0, "finger_point_cs_1" + std::to_string(cs_1));
    }

    for (size_t os_1 = 0; os_1 < multigripper->open_fingers_1.size(); os_1++) {
      pcl::PointNormal os_point_1(multigripper->open_fingers_1[os_1](0),
        multigripper->open_fingers_1[os_1](1),
        multigripper->open_fingers_1[os_1](2));
      viewer->addSphere(os_point_1, 0.01, 0, 1.0, 0, "finger_point_os_1" + std::to_string(os_1));
    }

    for (size_t cs_2 = 0; cs_2 < multigripper->closed_fingers_2.size(); cs_2++) {
      viewer->addSphere(
        multigripper->closed_fingers_2[cs_2]->
        finger_point, 0.01, 1.0, 0, 0, "finger_point_cs_2" + std::to_string(cs_2));
    }
    for (size_t os_2 = 0; os_2 < multigripper->open_fingers_2.size(); os_2++) {
      pcl::PointNormal os_point_2(multigripper->open_fingers_2[os_2](0),
        multigripper->open_fingers_2[os_2](1),
        multigripper->open_fingers_2[os_2](2));
      viewer->addSphere(os_point_2, 0.01, 0, 1.0, 0, "finger_point_os_2" + std::to_string(os_2));
    }
    viewer->spin();
    viewer->close();
    viewer->removeAllShapes();
  }
  // std::cout << "getAllRanks" << std::endl;
}


// void FingerGripper::getBestGrasps(std::shared_ptr<GraspObject> object,
//   emd_msgs::msg::GraspMethod *grasp_method,
//   std::shared_ptr<fcl::CollisionObject<float>> world_collision_object)
// {

//   std::vector<std::future<void>> futures_1;
//   auto GetBestGrasps1 = [this](
//     int i,
//     pcl::PointNormal &centroid_point,
//     float &initial_1_2_dist,
//     float &initial_1_centroid_dist,
//     //float &initial_2_centroid_dist,
//     std::shared_ptr<GraspObject> &object,
//     emd_msgs::msg::GraspMethod *grasp_method,
//     //octomap::point3d sensor_origin,
//     std::shared_ptr<fcl::CollisionObject<float>> &world_collision_object)-> void
//   {
//     static std::mutex obj_2_mutex;

//     std::vector<std::future<void>> futures_2;
//     auto GetBestGrasps2 = [this](
//       int j,
//       pcl::PointNormal &centroid_point,
//       pcl::PointNormal &curr1_point,
//       Eigen::Vector3f &curr1_vector,
//       float initial_1_2_dist,
//       //float &curr1_centroid_dist,
//       float &curr1_grasp_plane_dist,
//       float &curr1_curvature,
//       std::shared_ptr<GraspObject> &object,
//       emd_msgs::msg::GraspMethod *grasp_method,
//       //octomap::point3d sensor_origin,
//       std::shared_ptr<fcl::CollisionObject<float>> &world_collision_object)-> void
//     {
//       float epsilon = 1e-8;
//       pcl::PointNormal curr2_point = this->grasp_2_nvoxel->points[j];
//       Eigen::Vector3f curr2_vector(curr2_point.x, curr2_point.y, curr2_point.z);
//       float curr1_curr2_dist = pcl::geometry::distance(curr1_point, curr2_point);
//       float initial1_curr2_dist = pcl::geometry::distance(this->sample_point_1, curr2_point);
//       float curr2_centroid_dist = PCLFunctions::normalize(
//         pcl::geometry::distance(curr2_point, centroid_point),
//         this->centroid_dist_2_min, this->centroid_dist_2_max);

//       // Points are closer to the centroid than initials: points on the surface!
//       if (curr2_centroid_dist < this->centroid_dist_2_min * 0.98 ||
//           curr1_curr2_dist < initial_1_2_dist * 0.98 ||
//           initial1_curr2_dist < initial_1_2_dist * 0.98)
//       {
//         //continue;
//       }
//       Eigen::Vector3f curr1_normal(curr1_point.normal_x,
//                                       curr1_point.normal_y,
//                                       curr1_point.normal_z);

//       Eigen::Vector3f curr2_normal(curr2_point.normal_x,
//                                   curr2_point.normal_y,
//                                   curr2_point.normal_z);

//       // Get the vector representing the gripper direction
//       Eigen::Vector3f grasp_direction =
//       Eigen::ParametrizedLine<float,3>::Through(curr1_vector, curr2_vector).direction();

//       // Get End effector finger clouds
//       Eigen::Vector3f grasp_direction_norm = grasp_direction/grasp_direction.norm();
//       Eigen::Vector3f curr1_2_centerpoint_vector((curr1_point.x + curr2_point.x)/2,
//       (curr1_point.y + curr2_point.y)/2, (curr1_point.z + curr2_point.z)/2);
//       Eigen::Vector3f finger1_point = curr1_2_centerpoint_vector +
//       (this->gripper_stroke/2) * grasp_direction_norm;
//       Eigen::Vector3f finger2_point = curr1_2_centerpoint_vector -
//       (this->gripper_stroke/2) * grasp_direction_norm;
//       // pcl::PointCloud<pcl::PointXYZ>::Ptr finger1_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//       // pcl::PointCloud<pcl::PointXYZ>::Ptr finger2_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//       // PCLFunctions::createSphereCloud(
//       // finger1_cloud, finger1_point, this->finger_thickness/2, 20);
//       // PCLFunctions::createSphereCloud(
//       // finger2_cloud, finger2_point, this->finger_thickness/2, 20);
//       // std::vector<std::shared_ptr<fcl::CollisionObject<float>>> fingers =
//       //   generateFingerModels(finger1_cloud,finger2_cloud,sensor_origin);

//       // // End Get End effector finger clouds//
//       // if (CheckFingerCollision(fingers[0], world_collision_object) ||
//       // CheckFingerCollision(fingers[1], world_collision_object))
//       // {
//       //   // continue; //Finger collides
//       //   return;
//       // }

//       auto finger_1_shape = std::make_shared<fcl::Sphere<float>>(this->finger_thickness/2);
//       fcl::Transform3<float> finger_1_transform = fcl::Transform3<float>::Identity();
//       finger_1_transform.translation() << finger1_point(0), finger1_point(1), finger1_point(2);
//       fcl::CollisionObject<float> finger_1(finger_1_shape, finger_1_transform);
//       std::shared_ptr<fcl::CollisionObject<float>> finger_1_ptr =
//         std::make_shared<fcl::CollisionObject<float>>(finger_1);

//       auto finger_2_shape = std::make_shared<fcl::Sphere<float>>(this->finger_thickness/2);
//       fcl::Transform3<float> finger_2_transform = fcl::Transform3<float>::Identity();
//       finger_2_transform.translation() << finger2_point(0), finger2_point(1), finger2_point(2);
//       fcl::CollisionObject<float> finger_2(finger_2_shape, finger_2_transform);
//       std::shared_ptr<fcl::CollisionObject<float>> finger_2_ptr =
//         std::make_shared<fcl::CollisionObject<float>>(finger_2);

//       // if (CheckFingerCollision(world_collision_object, finger_1_ptr) ||
//       //     CheckFingerCollision(world_collision_object, finger_2_ptr))
//       // {
//       //   // continue; //Finger collides
//       //   return;
//       // }
//       float grasp_plane_angle_cos = epsilon +
//       std::abs((grasp_direction.dot(this->center_cutting_plane_normal)) /
//         (grasp_direction.norm() * this->center_cutting_plane_normal.norm()));

//       // float grasp_plane_angle_cos = epsilon +
//       //  std::abs((grasp_direction.dot(object->grasp_plane_normal)) /
//       //   (grasp_direction.norm() * object->grasp_plane_normal.norm()));

//       float curr1_angle_cos = epsilon +
//         std::abs((grasp_direction.dot(curr1_normal)) /
//           (grasp_direction.norm() * curr1_normal.norm()));

//       float curr2_angle_cos = epsilon +
//         std::abs((grasp_direction.dot(curr2_normal)) /
//           (grasp_direction.norm() * curr2_normal.norm()));

//       float curr2_grasp_plane_dist = PCLFunctions::normalize(
//       PCLFunctions::pointToPlane(this->center_cutting_plane, curr2_point),
//       this->grasp_plane_dist_2_min, this->grasp_plane_dist_2_max);

//       // float curr2_grasp_plane_dist = PCLFunctions::normalize(
//       // PCLFunctions::pointToPlane(object->grasp_plane, curr2_point),
//       // object->grasp_plane_dist_2_min, object->grasp_plane_dist_2_max);

//       float curr2_curvature = PCLFunctions::normalize(curr2_point.curvature,
//         this->curvature_2_min, this->curvature_2_max);

//       float grasp_rank = getGraspQuality(grasp_plane_angle_cos,
//         curr1_curvature, curr1_grasp_plane_dist, curr1_angle_cos,
//         curr2_curvature, curr2_grasp_plane_dist, curr2_angle_cos);

//       //Insert the rank and grasp into the vectors in object
//       //(Might remove, takes time to iterate)
//       std::lock_guard<std::mutex> lock(obj_2_mutex);
//       int rank;
//       std::vector<geometry_msgs::msg::PoseStamped>::iterator grasps_it;
//       std::vector<GraspContacts>::iterator contacts_it;
//       for (rank = 0, grasps_it = grasp_method->grasp_poses.begin(),
//             contacts_it = grasp_contacts.begin();
//             rank < static_cast<int>(grasp_method->grasp_ranks.size());
//         ++rank, ++grasps_it, ++contacts_it)
//       {

//         if (grasp_rank > grasp_method->grasp_ranks[rank]) {
//           GraspContacts new_grasp;
//           new_grasp.point_1.x = finger1_point(0);
//           new_grasp.point_1.y = finger1_point(1);
//           new_grasp.point_1.z = finger1_point(2);
//           new_grasp.point_2.x = finger2_point(0);
//           new_grasp.point_2.y = finger2_point(1);
//           new_grasp.point_2.z = finger2_point(2);
//           grasp_method->grasp_ranks.insert(grasp_method->grasp_ranks.begin() + rank, grasp_rank);
//           //object->ranks.insert(object->ranks.begin() + rank, grasp_rank);
//           grasp_method->grasp_poses.insert(grasps_it, getGraspPose(new_grasp,
//           object->object_frame, object));
//           this->grasp_contacts.insert(contacts_it, new_grasp);
//           break;
//         }
//       }
//     };

//     pcl::PointNormal curr1_point = this->grasp_1_nvoxel->points[i];
//     Eigen::Vector3f curr1_vector(curr1_point.x, curr1_point.y, curr1_point.z);
//     float curr1_initial2_dist = pcl::geometry::distance(curr1_point, this->sample_point_2);

//     float curr1_centroid_dist = PCLFunctions::normalize(
//       pcl::geometry::distance(curr1_point, centroid_point),
//       this->centroid_dist_1_min, this->centroid_dist_1_max);

//     // If the first point centroid distance is lesser than the initial first point
//     // centroid distance, OR
//     // if the initial distance between grasp points is wider
//     // than the current distance between grasp points

//     // Point is then on the surface, cause collision

//     if (curr1_centroid_dist  < initial_1_centroid_dist ||
//         curr1_initial2_dist < initial_1_2_dist)
//     {
//       //continue;
//     }

//     float curr1_grasp_plane_dist = PCLFunctions::normalize(
//       PCLFunctions::pointToPlane(this->center_cutting_plane, curr1_point),
//       this->grasp_plane_dist_1_min, this->grasp_plane_dist_1_max);

//     // float curr1_grasp_plane_dist = PCLFunctions::normalize(
//     //   PCLFunctions::pointToPlane(object->grasp_plane, curr1_point),
//     //   object->grasp_plane_dist_1_min, object->grasp_plane_dist_1_max);

//     float curr1_curvature = PCLFunctions::normalize(curr1_point.curvature,
//       this->curvature_1_min, this->curvature_1_max);

//     for(int j = 0 ; j < static_cast<int>(this->grasp_2_nvoxel->points.size()); j++)
//     {
//       futures_2.push_back(
//         std::async(std::launch::async,
//           GetBestGrasps2,
//           j,
//           std::ref(centroid_point),
//           std::ref(curr1_point),
//           std::ref(curr1_vector),
//           std::ref(initial_1_2_dist),
//           //std::ref(curr1_centroid_dist),
//           std::ref(curr1_grasp_plane_dist),
//           std::ref(curr1_curvature),
//           std::ref(object),
//           grasp_method,
//           //std::ref(sensor_origin),
//           std::ref(world_collision_object)));
//     }
//   };

//   // viewer->addPointCloud<pcl::PointNormal>(this->grasp_cloud, "grasp cloud");
//   // viewer->addPointCloud<pcl::PointNormal>(this->grasp_1_nvoxel, "grasp_1_nvoxel");
//   // viewer->addPointCloud<pcl::PointNormal>(this->grasp_2_nvoxel, "grasp_2_nvoxel");

//   pcl::PointNormal centroid_point{
//     object->centerpoint(0),
//     object->centerpoint(1),
//     object->centerpoint(2)};

//   float initial_1_2_dist = pcl::geometry::distance(this->sample_point_1,
//     this->sample_point_2);
//   float initial_1_centroid_dist = pcl::geometry::distance(this->sample_point_1,
//     centroid_point);
//   float initial_2_centroid_dist = pcl::geometry::distance(this->sample_point_2,
//     centroid_point);

//   initial_1_centroid_dist = PCLFunctions::normalize(initial_1_centroid_dist,
//     this->centroid_dist_1_min, this->centroid_dist_1_max);
//   initial_2_centroid_dist = PCLFunctions::normalize(initial_2_centroid_dist,
//     this->centroid_dist_2_min, this->centroid_dist_2_max);
//   for(int i = 0 ; i < static_cast<int>(this->grasp_1_nvoxel->points.size()); i++)
//   {
//     futures_1.push_back(
//       std::async(std::launch::async,
//         GetBestGrasps1,
//         i,
//         std::ref(centroid_point),
//         std::ref(initial_1_2_dist),
//         std::ref(initial_1_centroid_dist),
//         //std::ref(initial_2_centroid_dist),
//         std::ref(object),
//         grasp_method,
//         //std::ref(sensor_origin),
//         std::ref(world_collision_object)));
//   }

// }

// float FingerGripper::getGraspQuality(
//   const float &grasp_plane_angle_cos, const float &point1_curvature,
//   const float &point1_grasp_plane_dist, const float &point1_grasp_angle_cos,
//   const float &point2_curvature, const float &point2_grasp_plane_dist,
//   const float &point2_grasp_angle_cos)
// {

//   float rank_1 = 2.0 - point1_grasp_plane_dist - point2_grasp_plane_dist -
//   (grasp_plane_angle_cos - 0.20) * 10.0;
//   float rank_2 = 2.0 - point1_curvature - point2_curvature +
//         point1_grasp_angle_cos + point2_grasp_angle_cos -
//         std::abs(point1_grasp_angle_cos - point2_grasp_angle_cos);
//   // Minimum quality criteria
//   if (rank_1 < 1.0 || rank_2 < 1.0)
//       return 0;
//   float rank = grasp_quality_weight1 * rank_1 + grasp_quality_weight2 * rank_2;
//   return rank;
// }
