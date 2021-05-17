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
static const rclcpp::Logger & LOGGER = rclcpp::get_logger("FingerGripper");

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
  if (num_fingers_side_1_ <= 0 || num_fingers_side_2_ <= 0) {
    RCLCPP_ERROR(LOGGER, "Each side needs to have a minimum of 1 finger");
    throw std::invalid_argument("Invalid value for field.");
  }
  if (num_fingers_side_1 == 1 || num_fingers_side_2 == 1) {
    if (num_fingers_side_1 == 1 && num_fingers_side_2 == 1) {
      if (distance_between_fingers_1 > 0.0 || distance_between_fingers_2 > 0.0) {
        RCLCPP_ERROR(LOGGER, "Distance_between_fingers should be 0 for both sides");
        throw std::invalid_argument("Invalid value for field.");
      }
    }
    if (num_fingers_side_1 == 1 && num_fingers_side_2 > 1) {
      if (distance_between_fingers_1 > 0.0) {
        RCLCPP_ERROR(LOGGER, "Distance_between_fingers should be 0 for Side 1");
        throw std::invalid_argument("Invalid value for field.");
      }
      if (finger_thickness > distance_between_fingers_2) {
        RCLCPP_ERROR(
          LOGGER,
          "Finger thickness is too large, will collide with adjacent fingers on Side 2");
        throw std::invalid_argument("Invalid value for field.");
      }
    }
    if (num_fingers_side_2 == 1 && num_fingers_side_1 > 1) {
      if (distance_between_fingers_2 > 0.0) {
        RCLCPP_ERROR(LOGGER, "Distance_between_fingers should be 0 for Side 2");
        throw std::invalid_argument("Invalid value for field.");
      }
      if (finger_thickness > distance_between_fingers_1) {
        RCLCPP_ERROR(
          LOGGER,
          "Finger thickness is too large, will collide with adjacent fingers on Side 1");
        throw std::invalid_argument("Invalid value for field.");
      }
    }
  } else if (finger_thickness > distance_between_fingers_1 ||
    finger_thickness > distance_between_fingers_2)
  {
    RCLCPP_ERROR(LOGGER, "Finger thickness is too large, will collide with adjacent fingers");
    throw std::invalid_argument("Invalid value for field.");
  }

  if (finger_thickness <= 0) {
    RCLCPP_ERROR(LOGGER, "Finger thickness needs to be positive and non-zero");
    throw std::invalid_argument("Invalid value for field.");
  }
  if (gripper_stroke <= 0) {
    RCLCPP_ERROR(LOGGER, "Gripper stroke needs to be positive and non-zero");
    throw std::invalid_argument("Invalid value for field.");
  }
  if (gripper_stroke < finger_thickness) {
    RCLCPP_ERROR(LOGGER, "Gripper stroke needs larger than finger_thickness");
    throw std::invalid_argument("Invalid value for field.");
  }
  this->num_fingers_total = this->num_fingers_side_1 + this->num_fingers_side_2;
}

/***************************************************************************//**
 * Get the derived gripper attributes that will be used in the grasp samples
 * generation
 ******************************************************************************/
void FingerGripper::generateGripperAttributes()
{
  this->is_even_1 = this->num_fingers_side_1 % 2 == 0;
  this->is_even_2 = this->num_fingers_side_2 % 2 == 0;

  /*Get the initial gap between on each side with respect to the center plane.
  For an odd number of fingers the initial gap is half the distance between finger
  (since the point on the center plane is taken)
  For an even number of fingers the initial gap is the distance between fingers
  (Since the point on center plane is not taken.)*/

  this->initial_gap_1 = this->distance_between_fingers_1 / (1.0 + (is_even_1 ? 1 : 0));
  this->initial_gap_2 = this->distance_between_fingers_2 / (1.0 + (is_even_2 ? 1 : 0));

  this->num_itr_1 = (this->num_fingers_side_1 == 1 ? 0 : floor(this->num_fingers_side_1 / 2));
  this->num_itr_2 = (this->num_fingers_side_2 == 1 ? 0 : floor(this->num_fingers_side_2 / 2));
}

void FingerGripper::planGrasps(
  std::shared_ptr<GraspObject> object,
  emd_msgs::msg::GraspMethod * grasp_method,
  std::shared_ptr<CollisionObject> world_collision_object)
{
  generateGripperAttributes();
  getCenterCuttingPlane(object);
  std::cout << "getCuttingPlanes" << std::endl;
  getCuttingPlanes(object);
  std::cout << "getGraspCloud" << std::endl;
  if (!this->getGraspCloud(object)) {
    RCLCPP_ERROR(
      LOGGER,
      "Grasping Planes do not intersect with object. Off center grasp is needed. Please change gripper");
    this->resetVariables();
    return;
  }

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
}

/***************************************************************************//**
 * Function find the plane that cuts the center of the Grasp Object
 * @param object grasp object
 ******************************************************************************/

void FingerGripper::getCenterCuttingPlane(const std::shared_ptr<GraspObject> object)
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

  /*! \brief If any of the sides have odd numbered fingers,
  we include the center plane in consideration for that side. */
  if (!side_1_even || !side_2_even) {
    Eigen::Vector3f centerpoint3f(
      object->centerpoint(0),
      object->centerpoint(1),
      object->centerpoint(2));

    this->grasp_samples.push_back(
      generateGraspSamples(
        this->center_cutting_plane,
        centerpoint3f,
        0,
        0));

    this->cutting_plane_distances.push_back(0);
    if (!side_1_even) {
      this->plane_1_index.push_back(0);
    }
    if (!side_2_even) {
      this->plane_2_index.push_back(0);
    }
  }
}

/***************************************************************************//**
 * Function to create cutting planes along the object. each cutting plane represents
 * the planar contact area for two fingers. For multi-fingered gripper, multiple planes are created,
 * @param object grasp object
 ******************************************************************************/
void FingerGripper::getCuttingPlanes(const std::shared_ptr<GraspObject> object)
{
  bool side_1_even = this->num_fingers_side_1 % 2 == 0;
  bool side_2_even = this->num_fingers_side_2 % 2 == 0;
  bool both_sides_even = (side_1_even && side_2_even);
  bool both_sides_odd = (!side_1_even && !side_2_even);

  /*! \brief If both sides are odd or both even, the spacing for planes is consistent */
  if (both_sides_even || both_sides_odd) {
    // std::cout << "Both sides even or odd" << std::endl;
    if (this->distance_between_fingers_1 == this->distance_between_fingers_2) {
      addCuttingPlanesEqualAligned(
        object->centerpoint, this->center_cutting_plane,
        both_sides_even);
    } else {
      addCuttingPlanes(
        object->centerpoint, this->center_cutting_plane, this->num_itr_1, this->num_itr_2,
        this->initial_gap_1,
        this->initial_gap_2);
    }

  } else { /*! \brief If both sides are different, the spacing for planes is not consistent,
                      we need to create planes separately */
    addCuttingPlanes(
      object->centerpoint, this->center_cutting_plane, this->num_itr_1, this->num_itr_2,
      this->initial_gap_1,
      this->initial_gap_2);
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
  int curr_min_size = (side_1_max ? this->plane_2_index.size() : this->plane_1_index.size());
  for (int row = 0, updown_toggle = 1; row < num_itr; row += updown_toggle ^= 1) {
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
      if (curr_min_size < min_fingers) {  // Plane still contains fingers on both side
        std::cout << "Add to both" << std::endl;
        addPlane(gap, centerpoint, plane_vector, true, true);
        curr_min_size++;
      } else {  // Plane only contains fingers on the side with more fingers
        if (side_1_max) {
          std::cout << "Add to side 1" << std::endl;
          addPlane(gap, centerpoint, plane_vector, true, false);
        } else {
          std::cout << "Add to side 2" << std::endl;
          addPlane(gap, centerpoint, plane_vector, false, true);
        }
      }
    }
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

  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  int curr_index = this->grasp_samples.size();
  this->cutting_plane_distances.push_back(dist);

  this->grasp_samples.push_back(
    generateGraspSamples(
      plane_vector,
      point_on_plane,
      dist,
      curr_index));

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
bool FingerGripper::getGraspCloud(const std::shared_ptr<GraspObject> object)
{
  bool at_least_one_plane_intersect = false;
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
      at_least_one_plane_intersect = true;
    } else {
      sample->plane_intersects_object = false;
    }
  }
  return at_least_one_plane_intersect;
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
      if (sample->plane_intersects_object) {
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
        sample->sample_side_1->start_index = first_point_index;
        sample->sample_side_2->start_index = second_point_index;
      }
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
  pcl::PointNormal centroid_point;
  centroid_point.x = object->centerpoint(0);
  centroid_point.y = object->centerpoint(1);
  centroid_point.z = object->centerpoint(2);

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
          // Eigen::Vector3f curr1_vector(point.x, point.y, point.z);
          float centroid_dist = MathFunctions::normalize(
            pcl::geometry::distance(point, centroid_point),
            sample->sample_side_1->centroid_dist_min, sample->sample_side_1->centroid_dist_max);
          float grasp_plane_dist = MathFunctions::normalize(
            PCLFunctions::pointToPlane(sample->plane_eigen, point),
            sample->sample_side_1->grasp_plane_dist_min,
            sample->sample_side_1->grasp_plane_dist_max);
          float curvature = MathFunctions::normalize(
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
          // Eigen::Vector3f curr2_vector(point.x, point.y, point.z);
          float centroid_dist = MathFunctions::normalize(
            pcl::geometry::distance(point, centroid_point),
            sample->sample_side_2->centroid_dist_min, sample->sample_side_2->centroid_dist_max);
          float grasp_plane_dist = MathFunctions::normalize(
            PCLFunctions::pointToPlane(sample->plane_eigen, point),
            sample->sample_side_2->grasp_plane_dist_min,
            sample->sample_side_2->grasp_plane_dist_max);
          float curvature = MathFunctions::normalize(
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
  const std::shared_ptr<CollisionObject> world_collision_object)
{
  std::vector<std::shared_ptr<multiFingerGripper>> valid_open_gripper_configs;
  // Query the gripping points at the center cutting plane
  if (this->grasp_samples[0]->plane_intersects_object) {
    for (auto & finger_sample_1 : this->grasp_samples[0]->sample_side_1->finger_samples) {
      for (auto & finger_sample_2 : this->grasp_samples[0]->sample_side_2->finger_samples) {
        // Eigen::Vector3f centerpoint_side1_vector(finger_sample_1->finger_point.x,
        //   finger_sample_1->finger_point.y,
        //   finger_sample_1->finger_point.z);
        // Eigen::Vector3f centerpoint_side2_vector(finger_sample_2->finger_point.x,
        //   finger_sample_2->finger_point.y,
        //   finger_sample_2->finger_point.z);
        Eigen::Vector3f centerpoint_side1_vector =
          PCLFunctions::convertPCLtoEigen(finger_sample_1->finger_point);
        Eigen::Vector3f centerpoint_side2_vector =
          PCLFunctions::convertPCLtoEigen(finger_sample_2->finger_point);

        // Get the vector representing the grasping direction
        Eigen::Vector3f grasp_direction = Eigen::ParametrizedLine<float, 3>::Through(
          centerpoint_side1_vector, centerpoint_side2_vector).direction();

        // Find the angle of the normal vector of each point with the grasp direction vector.
        finger_sample_1->angle_cos = MathFunctions::getAngleBetweenVectors(
          grasp_direction,
          {finger_sample_1->finger_point.normal_x,
            finger_sample_1->finger_point.normal_y,
            finger_sample_1->finger_point.normal_z});

        finger_sample_2->angle_cos = MathFunctions::getAngleBetweenVectors(
          grasp_direction,
          {finger_sample_2->finger_point.normal_x,
            finger_sample_2->finger_point.normal_y,
            finger_sample_2->finger_point.normal_z});

        /* Get the vector perpendicular to the grasp direction. This vector will be used to generate
         the other fingers with reference to the center finger. */

        Eigen::Vector3f perpendicular_grasp_direction = getGripperPlane(
          finger_sample_1,
          finger_sample_2,
          grasp_direction, object);

        /* Get the coordinates of the open finger configuration of the center fingers of the gripper */
        std::vector<Eigen::Vector3f> open_coords = getOpenFingerCoordinates(
          grasp_direction,
          centerpoint_side1_vector,
          centerpoint_side2_vector);

        /* With the open configuration of the center fingers, generate the rest of the open configuration grippers */
        std::shared_ptr<multiFingerGripper> gripper_sample = generateGripperOpenConfig(
          world_collision_object, finger_sample_1, finger_sample_2,
          open_coords[0], open_coords[1], perpendicular_grasp_direction,
          grasp_direction);

        if (!gripper_sample->collides_with_world) {
          valid_open_gripper_configs.push_back(gripper_sample);
        }
      }
    }
  }
  return valid_open_gripper_configs;
}


/***************************************************************************//**
 * Function to create the finger gripper configuration.
 * Returns the created finger gripper sample. Finger sample is created using the center part of the
 * gripper and then based on the grasp direction the finger will be expanded outwards.
 * @param world_collision_object Collision object representing the world
 * @param closed_center_finger_1 Finger representation of the closed configuration for the center finger in side 1.
 * @param closed_center_finger_2 Finger representation of the closed configuration for the center finger in side 2.
 * @param open_center_finger_1 Finger representation of the open configuration for the center finger in side 1.
 * @param open_center_finger_2 Finger representation of the open configuration for the center finger in side 2.
 * @param plane_normal_normalized Normal vector of the center plane.
 ******************************************************************************/
std::shared_ptr<multiFingerGripper> FingerGripper::generateGripperOpenConfig(
  const std::shared_ptr<CollisionObject> world_collision_object,
  const std::shared_ptr<singleFinger> closed_center_finger_1,
  const std::shared_ptr<singleFinger> closed_center_finger_2,
  const Eigen::Vector3f open_center_finger_1,
  const Eigen::Vector3f open_center_finger_2,
  Eigen::Vector3f plane_normal, Eigen::Vector3f grasp_direction)
{
  // Create an instance of the multifinger gripper.
  multiFingerGripper gripper(closed_center_finger_1, closed_center_finger_2);
  gripper.collides_with_world = false;
  bool is_even_1 = this->num_fingers_side_1 % 2 == 0;
  bool is_even_2 = this->num_fingers_side_2 % 2 == 0;

  /* Get the angle between the vector between the grasp direction and the normal of the cutting
     plane */

  float grasp_plane_angle_cos_ = MathFunctions::getAngleBetweenVectors(
    grasp_direction, this->center_cutting_plane_normal);
  gripper.grasp_plane_angle_cos = grasp_plane_angle_cos_;

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

  // Check Initial middle fingers if they collide with world
  if (checkFingerCollision(open_center_finger_1, world_collision_object)) {
    gripper.collides_with_world = true;
  }
  if (checkFingerCollision(open_center_finger_2, world_collision_object)) {
    gripper.collides_with_world = true;
  }

  // Iterate through the points on side 1
  for (int side_1 = 0, updown_toggle_1 = 1; side_1 < this->num_itr_1;
    side_1 += updown_toggle_1 ^= 1)
  {
    // Define the distance the current finger is from the center point.
    float gap1;
    gap1 = (updown_toggle_1 == 0 ? 1 : -1) * (this->initial_gap_1 + side_1 *
      this->distance_between_fingers_1);

    // Get the coordinates of the current finger in the open configuration
    Eigen::Vector3f finger_1_open_temp = MathFunctions::getPointInDirection(
      open_center_finger_1,
      plane_normal, gap1);
    // Eigen::Vector3f finger_1_open_temp = open_center_finger_1 + gap1 * plane_normal_normalized;
    gripper.open_fingers_1.push_back(finger_1_open_temp);

    /* Check if this current open configuration for the finger collides with the world,
       since the end effector will approach the object in its open state.*/

    if (checkFingerCollision(finger_1_open_temp, world_collision_object)) {
      gripper.collides_with_world = true;
    }

    /* Ranking of grasp quality involves the positions of the gripper fingers ON the object,
       so we next need to find the finger point on the object corresponding to the current
       open configuration of the finger */

    // Find the index of the closest cutting plane with the given gap from the center finger
    int plane_index = getNearestPlaneIndex(gap1);

    // Now that we find the plane, we need to find the closest point to the corresponding
    // normal cloud of the object at the particular side
    pcl::PointNormal finger_1_point;
    finger_1_point.x = finger_1_open_temp(0);
    finger_1_point.y = finger_1_open_temp(1);
    finger_1_point.z = finger_1_open_temp(2);

    int point_index = getNearestPointIndex(
      finger_1_point,
      this->grasp_samples[plane_index]->sample_side_1->finger_nvoxel);

    // Add the finger sample to the gripper configuration
    gripper.closed_fingers_1.push_back(
      this->grasp_samples[plane_index]->sample_side_1->finger_samples[point_index]);
    gripper.closed_fingers_1_index.push_back(point_index);

    // Eigen::Vector3f finger_normal(
    //   gripper.closed_fingers_1[gripper.closed_fingers_1.size() - 1]->finger_point.normal_x,
    //   gripper.closed_fingers_1[gripper.closed_fingers_1.size() - 1]->finger_point.normal_y,
    //   gripper.closed_fingers_1[gripper.closed_fingers_1.size() - 1]->finger_point.normal_z);
    Eigen::Vector3f finger_normal =
      PCLFunctions::convertPCLNormaltoEigen(
        gripper.closed_fingers_1[gripper.closed_fingers_1.size() - 1]->finger_point);

    gripper.closed_fingers_1[gripper.closed_fingers_1.size() - 1]->angle_cos =
      MathFunctions::getAngleBetweenVectors(grasp_direction, finger_normal);
  }

  // Iterate through the points on side 2
  for (int side_2 = 0, updown_toggle_2 = 1; side_2 < this->num_itr_2;
    side_2 += updown_toggle_2 ^= 1)
  {
    float gap2;
    gap2 =
      (updown_toggle_2 == 0 ? 1 : -1) * (initial_gap_2 + side_2 * this->distance_between_fingers_2);
    Eigen::Vector3f finger_2_open_temp = MathFunctions::getPointInDirection(
      open_center_finger_2,
      plane_normal, gap2);
    if (checkFingerCollision(finger_2_open_temp, world_collision_object)) {
      gripper.collides_with_world = true;
    }
    gripper.open_fingers_2.push_back(finger_2_open_temp);

    int plane_index_2 = getNearestPlaneIndex(gap2);

    pcl::PointNormal finger_2_point;
    finger_2_point.x = finger_2_open_temp(0);
    finger_2_point.y = finger_2_open_temp(1);
    finger_2_point.z = finger_2_open_temp(2);

    int point_index_2 = getNearestPointIndex(
      finger_2_point,
      this->grasp_samples[plane_index_2]->sample_side_2->finger_nvoxel);
    gripper.closed_fingers_2.push_back(
      this->grasp_samples[plane_index_2]->sample_side_2->finger_samples[point_index_2]);
    gripper.closed_fingers_2_index.push_back(point_index_2);

    // Eigen::Vector3f finger_normal_2(
    //   gripper.closed_fingers_2[gripper.closed_fingers_2.size() - 1]->finger_point.normal_x,
    //   gripper.closed_fingers_2[gripper.closed_fingers_2.size() - 1]->finger_point.normal_y,
    //   gripper.closed_fingers_2[gripper.closed_fingers_2.size() - 1]->finger_point.normal_z);
    
    Eigen::Vector3f finger_normal_2 =
      PCLFunctions::convertPCLNormaltoEigen(
        gripper.closed_fingers_2[gripper.closed_fingers_2.size() - 1]->finger_point);

    gripper.closed_fingers_2[gripper.closed_fingers_2.size() - 1]->angle_cos =
      MathFunctions::getAngleBetweenVectors(grasp_direction, finger_normal_2);
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
  const std::shared_ptr<CollisionObject> world_collision_object)
{
  grasp_planner::collision::Sphere * finger_shape =
    new grasp_planner::collision::Sphere(this->finger_thickness / 2);

  grasp_planner::collision::Transform finger_transform;
  finger_transform.setIdentity();

#if FCL_VERSION_0_6_OR_HIGHER == 1
  finger_transform.translation() << finger_point(0), finger_point(1), finger_point(2);
#else
  finger_transform.setTranslation(
    grasp_planner::collision::Vector(finger_point(0), finger_point(1), finger_point(2)));
#endif
  CollisionObject finger(
    std::shared_ptr<grasp_planner::collision::CollisionGeometry>(finger_shape), finger_transform);
  std::shared_ptr<CollisionObject> finger_ptr =
    std::make_shared<CollisionObject>(finger);

  grasp_planner::collision::CollisionRequest request;
  request.enable_contact = true;
  // num_max_contacts_ = 1, //The maximum number of contacts will return.
  // enable_contact_ = false, //whether the contact information (normal, penetration depth and
  // contact position) will return
  // num_max_cost_sources_ = 1, // The maximum number of cost sources will return.
  // enable_cost_ = false, //whether the cost sources will be computed
  // use_approximate_cost_ = true // whether the cost computation is approximated

  // result will be returned via the collision result structure
  grasp_planner::collision::CollisionResult result;
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
  pcl::PointNormal centroid_point;
  centroid_point.x = object->centerpoint(0);
  centroid_point.y = object->centerpoint(1);
  centroid_point.z = object->centerpoint(2);

  for (auto & sample : this->grasp_samples) {
    if (sample->plane_intersects_object) {
      for (auto & point : sample->sample_side_1->finger_nvoxel->points) {
        float centroid_distance, grasp_plane_distance, curvature;
        centroid_distance = pcl::geometry::distance(point, centroid_point);
        grasp_plane_distance = PCLFunctions::pointToPlane(sample->plane_eigen, point);
        curvature = point.curvature;
        updateMaxMinAttributes(
          sample->sample_side_1, centroid_distance, grasp_plane_distance,
          curvature);
      }

      for (auto & point : sample->sample_side_2->finger_nvoxel->points) {
        float centroid_distance, grasp_plane_distance, curvature;
        centroid_distance = pcl::geometry::distance(point, centroid_point);
        grasp_plane_distance = PCLFunctions::pointToPlane(sample->plane_eigen, point);
        curvature = point.curvature;
        updateMaxMinAttributes(
          sample->sample_side_2, centroid_distance, grasp_plane_distance,
          curvature);
      }
    }
  }
}

/***************************************************************************//**
 * Function that updates the max and min values of ranking attributes of
 * all finger samples at the particular section of the gripper. Used for
 * normalization later.

 * @param sample FingerCloudSample for the current finger
 * @param centroid_distance Distance from finger to object centroid
 * @param grasp_plane_distance Distance from finger sample to grasp plane
 * @param curvature Curvature of surface at finger point
 ******************************************************************************/

void FingerGripper::updateMaxMinAttributes(
  std::shared_ptr<fingerCloudSample> & sample,
  float centroid_distance,
  float grasp_plane_distance,
  float curvature)
{
  if (curvature < sample->curvature_min) {
    sample->curvature_min = curvature;
  }
  if (curvature > sample->curvature_max) {
    sample->curvature_max = curvature;
  }
  if (centroid_distance < sample->centroid_dist_min) {
    sample->centroid_dist_min = centroid_distance;
  }
  if (centroid_distance > sample->centroid_dist_max) {
    sample->centroid_dist_max = centroid_distance;
  }
  if (grasp_plane_distance < sample->grasp_plane_dist_min) {
    sample->grasp_plane_dist_min = grasp_plane_distance;
  }
  if (grasp_plane_distance > sample->grasp_plane_dist_max) {
    sample->grasp_plane_dist_max = grasp_plane_distance;
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
  Eigen::Vector3f plane_normal_vector(plane->values[0], plane->values[1], plane->values[2]);
  return target_vector.cross(plane_normal_vector);
}

/***************************************************************************//**
 * Given the contact points of 2 fingers on the surface of an object, find the
 * coordinates of the initial open configuration of the fingers depending on the
 * stroke
 * @param grasp_direction Vector representing the grasp direction
 * @param finger_1 Coordinates of finger at side 1
 * @param finger_2 Coordinates of finger at side 2
 ******************************************************************************/

std::vector<Eigen::Vector3f> FingerGripper::getOpenFingerCoordinates(
  Eigen::Vector3f grasp_direction,
  Eigen::Vector3f finger_1,
  Eigen::Vector3f finger_2)
{
  // Get the centerpoint vector of the grasp
  Eigen::Vector3f side1_2_centerpoint_vector(
    (finger_1(0) + finger_2(0)) / 2,
    (finger_1(1) + finger_2(1)) / 2,
    (finger_1(2) + finger_2(2)) / 2);

  // Get the initial finger positions in the open configuration from the center plane.
  // This may or may not be used depending on number of fingers

  Eigen::Vector3f open_center_finger_1 = MathFunctions::getPointInDirection(
    side1_2_centerpoint_vector, grasp_direction, -(this->gripper_stroke / 2));
  Eigen::Vector3f open_center_finger_2 = MathFunctions::getPointInDirection(
    side1_2_centerpoint_vector, grasp_direction, (this->gripper_stroke / 2));

  std::vector<Eigen::Vector3f> result{open_center_finger_1, open_center_finger_2};
  return result;
}

void FingerGripper::resetVariables()
{
}


void FingerGripper::visualizeGrasps(pcl::visualization::PCLVisualizer::Ptr viewer)
{
  std::cout << "Visualizing " << this->sorted_gripper_configs.size() << " grasps" << std::endl;
  for (auto const & multigripper : this->sorted_gripper_configs) {
    for (size_t cs_1 = 0; cs_1 < multigripper->closed_fingers_1.size(); cs_1++) {
      viewer->addSphere(
        multigripper->closed_fingers_1[cs_1]->
        finger_point, 0.01, 1.0, 0, 0, "finger_point_cs_1" + std::to_string(cs_1));
    }

    for (size_t os_1 = 0; os_1 < multigripper->open_fingers_1.size(); os_1++) {
      pcl::PointNormal os_point_1;
      os_point_1.x = multigripper->open_fingers_1[os_1](0);
      os_point_1.y = multigripper->open_fingers_1[os_1](1);
      os_point_1.z = multigripper->open_fingers_1[os_1](2);

      viewer->addSphere(os_point_1, 0.01, 0, 1.0, 0, "finger_point_os_1" + std::to_string(os_1));
    }

    for (size_t cs_2 = 0; cs_2 < multigripper->closed_fingers_2.size(); cs_2++) {
      viewer->addSphere(
        multigripper->closed_fingers_2[cs_2]->
        finger_point, 0.01, 1.0, 0, 0, "finger_point_cs_2" + std::to_string(cs_2));
    }
    for (size_t os_2 = 0; os_2 < multigripper->open_fingers_2.size(); os_2++) {
      pcl::PointNormal os_point_2;
      os_point_2.x = multigripper->open_fingers_2[os_2](0);
      os_point_2.y = multigripper->open_fingers_2[os_2](1);
      os_point_2.z = multigripper->open_fingers_2[os_2](2);

      viewer->addSphere(os_point_2, 0.01, 0, 1.0, 0, "finger_point_os_2" + std::to_string(os_2));
    }
    viewer->spin();
    viewer->close();
    viewer->removeAllShapes();
  }
}

/***************************************************************************//**
 * Method that generates the grasp sample for a particular plane vector
 * @param plane_vector vector of plane cutting the object
 * @param point_on_plane 3d point on plane
 * @param dist_to_center_plane perpendicular distance from center cutting plane
 * @param plane_index Index of plane with respect to the plane vector
 ******************************************************************************/

std::shared_ptr<graspPlaneSample> FingerGripper::generateGraspSamples(
  Eigen::Vector4f plane_vector,
  Eigen::Vector3f point_on_plane,
  float dist_to_center_plane,
  int plane_index)
{
  graspPlaneSample grasp_sample;
  grasp_sample.plane_index = plane_index;
  grasp_sample.dist_to_center_plane = dist_to_center_plane;
  grasp_sample.plane->values.resize(4);
  grasp_sample.plane_eigen(0) = grasp_sample.plane->values[0] = plane_vector(0);
  grasp_sample.plane_eigen(1) = grasp_sample.plane->values[1] = plane_vector(1);
  grasp_sample.plane_eigen(2) = grasp_sample.plane->values[2] = plane_vector(2);
  grasp_sample.plane_eigen(3) = grasp_sample.plane->values[3] =
    -((plane_vector(0) * point_on_plane(0)) + (plane_vector(1) * point_on_plane(1)) +
    (plane_vector(2) * point_on_plane(2)));
  return std::make_shared<graspPlaneSample>(grasp_sample);
}

/***************************************************************************//**
 * Given a vector of planes, find the index of the plane that represents a certain
 * distance from the center plane.
 * @param target_distance Distance target plane is from the center plane
 ******************************************************************************/

int FingerGripper::getNearestPlaneIndex(float target_distance)
{
  float plane_dist_diff = std::numeric_limits<float>::max();
  int plane_index;

  /* We use the gap from the open configuration finger to compare with the cutting plane
      gaps to find the most identical distance between the center plane to that plane, and
      that will be the plane index that corresponds to the correct plane */

  for (size_t plane_index_ = 0; plane_index_ < this->cutting_plane_distances.size();
    plane_index_++)
  {
    if (std::abs(this->cutting_plane_distances[plane_index_] - target_distance) < plane_dist_diff) {
      plane_dist_diff = std::abs(cutting_plane_distances[plane_index_] - target_distance);
      plane_index = plane_index_;
    }
  }
  return plane_index;
}

/***************************************************************************//**
 * Given a vector of planes, find the index of the plane that represents a certain
 * distance from the center plane.
 * @param target_point target point of reference
 * @param cloud cloud to search
 ******************************************************************************/
int FingerGripper::getNearestPointIndex(
  const pcl::PointNormal & target_point,
  const pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
  pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
  // We only need to find 1 neighbour. can be changed later
  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  // We already have a set of voxelized points on the correspoinding side, and correspoinding plane.
  kdtree.setInputCloud(cloud);

  if (kdtree.nearestKSearch(target_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
    return pointIdxNKNSearch[0];
  } else {
    return -1;
  }
}

/***************************************************************************//**
 * Method that generates the plane in which a particular gripper lies on
 * @param finger_sample_1 Finger sample on side 1
 * @param finger_sample_2 Finger sample on side 2
 * @param grasp_direction Vector representing grasp direction
 * @param object Grasp object
 ******************************************************************************/
Eigen::Vector3f FingerGripper::getGripperPlane(
  std::shared_ptr<singleFinger> & finger_sample_1,
  std::shared_ptr<singleFinger> & finger_sample_2,
  const Eigen::Vector3f & grasp_direction,
  const std::shared_ptr<GraspObject> & object)
{
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
  return perpendicular_grasp_direction;
}
