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

/***************************************************************************//**
 * Finger Gripper Constructor
 *
 * @param id_ gripper id
 * @param num_fingers_side_1_ Number of fingers on side 1
 * @param num_fingers_side_2_ Number of fingers on side 2
 * @param distance_between_fingers_1_ Distance between fingers at side 1
 * @param distance_between_fingers_2_ Distance between fingers at side 2
 * @param finger_thickness_ largest dimension of the finger
 * @param gripper_stroke_ distance between opposing gripper fingers
 * @param voxel_size_ Voxel size for downsampling
 * @param grasp_quality_weight1_ weight 1 of grasp ranking
 * @param grasp_quality_weight2_ weight 2 of grasp ranking
 * @param grasp_plane_dist_limit_ Parameter for SAC search for points on plane
 * @param cloud_normal_radius_ Radius of which to determine cloud normals
 * @param worldXAngleThreshold_ Threshold after which the object is angled to the world X axis
 * @param worldYAngleThreshold_ Threshold after which the object is angled to the world Y axis
 * @param worldZAngleThreshold_ Threshold after which the object is angled to the world Z axis
 * @param grasp_stroke_direction_ Axis in the same direction as the gripper stroke
 * @param grasp_stroke_normal_direction_ Axis normal to the direction of the gripper stroke
 * @param grasp_approach_direction_ Axis in which the gripper approaches the object
 ******************************************************************************/

FingerGripper::FingerGripper(
  std::string id_,
  const int & num_fingers_side_1_,
  const int & num_fingers_side_2_,
  const float & distance_between_fingers_1_,
  const float & distance_between_fingers_2_,
  const float & finger_thickness_,
  const float & gripper_stroke_,
  const float & voxel_size_,
  const float & grasp_quality_weight1_,
  const float & grasp_quality_weight2_,
  const float & grasp_plane_dist_limit_,
  const float & cloud_normal_radius_,
  const float & worldXAngleThreshold_,
  const float & worldYAngleThreshold_,
  const float & worldZAngleThreshold_,
  std::string grasp_stroke_direction_,
  std::string grasp_stroke_normal_direction_,
  std::string grasp_approach_direction_)
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
  worldZAngleThreshold(worldZAngleThreshold_),
  grasp_stroke_direction(grasp_stroke_direction_[0]),
  grasp_stroke_normal_direction(grasp_stroke_normal_direction_[0]),
  grasp_approach_direction(grasp_approach_direction_[0])
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

  generateGripperAttributes();
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

/***************************************************************************//**
 * Inherited method that visualizes the required grasps
 *
 * @param object Grasp Object
 * @param grasp_method Grasp method output for all possible grasps
 * @param world_collision_object FCL collision object of the world
 ******************************************************************************/

void FingerGripper::planGrasps(
  std::shared_ptr<GraspObject> object,
  emd_msgs::msg::GraspMethod * grasp_method,
  std::shared_ptr<CollisionObject> world_collision_object,
  std::string camera_frame)
{
  getCenterCuttingPlane(object);
  getCuttingPlanes(object);
  if (!this->getGraspCloud(object)) {
    RCLCPP_ERROR(
      LOGGER,
      "Grasping Planes do not intersect with object. Off center grasp is needed. Please change gripper");
    this->resetVariables();
    return;
  }

  if (!this->getInitialSamplePoints(object)) {
    this->resetVariables();
    return;
  }

  getInitialSampleCloud(object);
  voxelizeSampleCloud();
  getMaxMinValues(object);
  getFingerSamples(object);
  getGripperClusters();
  std::vector<std::shared_ptr<multiFingerGripper>> valid_open_gripper_configs =
    getAllGripperConfigs(object, world_collision_object, camera_frame);
  for (auto & gripper : valid_open_gripper_configs) {
    getGraspPose(gripper, object);
  }

  this->sorted_gripper_configs = getAllRanks(valid_open_gripper_configs, grasp_method);
}

/***************************************************************************//**
 * Function find the plane that cuts the center of the Grasp Object
 * @param object grasp object
 ******************************************************************************/

void FingerGripper::getCenterCuttingPlane(const std::shared_ptr<GraspObject> & object)
{
  /*! \brief First we find the normal of the cutting plane */
  Eigen::Vector3f centerpoint(
    object->centerpoint(0),
    object->centerpoint(1),
    object->centerpoint(2));
  Eigen::Vector3f center_minor = object->minor_axis - centerpoint;
  Eigen::Vector3f center_grasp = object->grasp_axis - centerpoint;
  Eigen::Vector3f gradient_vector = center_minor.cross(center_grasp);

  /*! \brief Create the cutting plane that goes through the center of the object. */
  float a = gradient_vector(0);
  float b = gradient_vector(1);
  float c = gradient_vector(2);
  float d =
    -((a * object->centerpoint(0)) +
    (b * object->centerpoint(1)) +
    (c * object->centerpoint(2)));
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
void FingerGripper::getCuttingPlanes(const std::shared_ptr<GraspObject> & object)
{
  bool side_1_even = this->num_fingers_side_1 % 2 == 0;
  bool side_2_even = this->num_fingers_side_2 % 2 == 0;
  bool both_sides_even = (side_1_even && side_2_even);
  bool both_sides_odd = (!side_1_even && !side_2_even);

  /*! \brief If both sides are odd or both even, the spacing for planes is consistent */
  if (both_sides_even || both_sides_odd) {
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
  const Eigen::Vector4f & centerpoint,
  const Eigen::Vector4f & plane_vector,
  const bool & both_sides_even)
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
        addPlane(gap, centerpoint, plane_vector, true, true);
        curr_min_size++;
      } else {  // Plane only contains fingers on the side with more fingers
        if (side_1_max) {
          addPlane(gap, centerpoint, plane_vector, true, false);
        } else {
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
  const int & num_itr_1,
  const int & num_itr_2,
  const float & initial_gap_1,
  const float & initial_gap_2)
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

int FingerGripper::checkPlaneExists(const float & dist)
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
  const float & dist,
  const Eigen::Vector4f & centerpoint,
  const Eigen::Vector4f & plane_vector,
  const bool & inside_1,
  const bool & inside_2)
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
bool FingerGripper::getGraspCloud(const std::shared_ptr<GraspObject> & object)
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
bool FingerGripper::getInitialSamplePoints(const std::shared_ptr<GraspObject> & object)
{
  pcl::PointNormal centerpoint;
  centerpoint.x = object->centerpoint(0);
  centerpoint.y = object->centerpoint(1);
  centerpoint.z = object->centerpoint(2);

  Eigen::Vector3f centerpoint_eigen(
    object->centerpoint(0),
    object->centerpoint(1),
    object->centerpoint(2));

  for (auto & sample : this->grasp_samples) {
    int first_point_index, second_point_index;
    float min_dist = std::numeric_limits<float>::max();
    // float min_linepoint_factor = std::numeric_limits<float>::max();
    float max_dist = 0;
    // float max_linepoint_factor = std::numeric_limits<float>::max();
    std::vector<MathFunctions::Point> point_vector;
    if (sample->plane_intersects_object) {
      for (size_t i = 0; i < sample->grasp_plane_ncloud->points.size(); ++i) {

        Eigen::Vector3f point(
          sample->grasp_plane_ncloud->points[i].x,
          sample->grasp_plane_ncloud->points[i].y,
          sample->grasp_plane_ncloud->points[i].z);

        MathFunctions::Point point_temp = MathFunctions::getPointInfo(
          point,
          object->grasp_axis,
          centerpoint_eigen);
        point_temp.index = i;
        point_temp.pcl_npoint = sample->grasp_plane_ncloud->points[i];
        point_vector.push_back(point_temp);
      }

      // Sort point vector according to its perpendicular distance from the direction vector
      std::sort(
        point_vector.begin(), point_vector.end(),
        [](MathFunctions::Point a, MathFunctions::Point b)
        {return a.perpendicular_distance < b.perpendicular_distance;});

      // Get the first 1/3 of points in the sorted vector (1/3 pts closest to the vector)
      int resize_size = static_cast<int>(point_vector.size() / 3);

      for (int i = 0; i < resize_size; i++) {
        if (point_vector[i].projection_distance < min_dist) {
          min_dist = point_vector[i].projection_distance;
          first_point_index = point_vector[i].index;
        }

        if (point_vector[i].projection_distance > max_dist) {
          max_dist = point_vector[i].projection_distance;
          second_point_index = point_vector[i].index;
        }
      }
      sample->sample_side_1->start_index = first_point_index;
      sample->sample_side_2->start_index = second_point_index;
    }
  }
  return true;
}

/***************************************************************************//**
 * Function that gets the cluster of point clouds based on a certain radius from
 * a certain start index that has already been defined in another method. This cluster
 * of point clouds will repreesnt the gripper finger point cloud at that point on the plane
 ******************************************************************************/
void FingerGripper::getInitialSampleCloud(const std::shared_ptr<GraspObject> & object)
{
  std::vector<std::future<void>> futures;
  auto getAllRadiusPoints = [this](
    std::shared_ptr<graspPlaneSample> & sample,
    const std::shared_ptr<GraspObject> & object) -> void
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
      }
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

/***************************************************************************//**
 * Method that generates all possible finger grasp samples
 *
 * @param object Grasp Object
 ******************************************************************************/
void FingerGripper::getFingerSamples(const std::shared_ptr<GraspObject> & object)
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
 * the multifinger gripper. TODO:
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
    }
  }
}

/***************************************************************************//**
 * Function to create all possible gripper configurations. Returns a vector
 * containing the configurations.

 * @param object Object to be grasped
 * @param world_collision_object Collision object representing the world
 ******************************************************************************/

std::vector<std::shared_ptr<multiFingerGripper>> FingerGripper::getAllGripperConfigs(
  const std::shared_ptr<GraspObject> & object,
  const std::shared_ptr<CollisionObject> & world_collision_object,
  std::string camera_frame)
{
  std::vector<std::shared_ptr<multiFingerGripper>> valid_open_gripper_configs;
  // Query the gripping points at the center cutting plane
  if (this->grasp_samples[0]->plane_intersects_object) {
    for (auto & finger_sample_1 : this->grasp_samples[0]->sample_side_1->finger_samples) {
      for (auto & finger_sample_2 : this->grasp_samples[0]->sample_side_2->finger_samples) {
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
          grasp_direction, camera_frame);
        if (!gripper_sample->collides_with_world) {
          valid_open_gripper_configs.push_back(gripper_sample);
        }
      }
    }
    if (valid_open_gripper_configs.empty()) {
      RCLCPP_ERROR(
        LOGGER,
        "All Possible Grasp sample collides with something in the scene! No grasps available");
    }
  } else {
    RCLCPP_ERROR(
      LOGGER,
      "Center Cutting plane does not intersect object, no grasp plans can be made.");
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
  const std::shared_ptr<CollisionObject> & world_collision_object,
  const std::shared_ptr<singleFinger> & closed_center_finger_1,
  const std::shared_ptr<singleFinger> & closed_center_finger_2,
  const Eigen::Vector3f & open_center_finger_1,
  const Eigen::Vector3f & open_center_finger_2,
  const Eigen::Vector3f & plane_normal,
  const Eigen::Vector3f & grasp_direction,
  std::string camera_frame)
{
  // Create an instance of the multifinger gripper.
  multiFingerGripper gripper(
    closed_center_finger_1,
    closed_center_finger_2,
    grasp_direction,
    plane_normal);
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

  // Setting up of marker members
  gripper.marker.header.frame_id = camera_frame;
  gripper.marker.ns = "";
  gripper.marker.type = gripper.marker.SPHERE_LIST;
  gripper.marker.action = gripper.marker.ADD;
  gripper.marker.lifetime = rclcpp::Duration::from_seconds(20);
  gripper.marker.scale.x = 0.02;
  gripper.marker.scale.y = 0.02;
  gripper.marker.scale.z = 0.02;
  gripper.marker.color.r = 1.0f;
  gripper.marker.color.a = 1.0;

  // Setting coordinates of open fingers on side 1 configuration into marker
  for (auto finger_1 : gripper.open_fingers_1) {
    geometry_msgs::msg::Point cup_marker_point;
    cup_marker_point.x = finger_1(0);
    cup_marker_point.y = finger_1(1);
    cup_marker_point.z = finger_1(2);
    gripper.marker.points.push_back(cup_marker_point);
  }

  // Setting coordinates of open fingers on side 2 configuration into marker
  for (auto finger_2 : gripper.open_fingers_2) {
    geometry_msgs::msg::Point cup_marker_point;
    cup_marker_point.x = finger_2(0);
    cup_marker_point.y = finger_2(1);
    cup_marker_point.z = finger_2(2);
    gripper.marker.points.push_back(cup_marker_point);
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
  const Eigen::Vector3f & finger_point,
  const std::shared_ptr<CollisionObject> & world_collision_object)
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
void FingerGripper::getMaxMinValues(const std::shared_ptr<GraspObject> & object)
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
  const float & centroid_distance,
  const float & grasp_plane_distance,
  const float & curvature)
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
    emd_msgs::msg::OptionArray grasp_option;
    grasp_option.options.push_back(addClosedGraspDistanceOption(gripper));
    getGripperRank(gripper);
    std::vector<geometry_msgs::msg::PoseStamped>::iterator grasps_it;
    std::vector<std::shared_ptr<multiFingerGripper>>::iterator contacts_it;
    std::vector<visualization_msgs::msg::Marker>::iterator markers_it;
    std::vector<emd_msgs::msg::OptionArray>::iterator options_it;
    size_t rank;
    for (rank = 0,
      grasps_it = grasp_method->grasp_poses.begin(), contacts_it = sorted_gripper_ranks.begin(),
      markers_it = grasp_method->grasp_markers.begin(),
      options_it = grasp_method->grasp_options.begin();
      rank < grasp_method->grasp_ranks.size();
      ++rank, ++grasps_it, ++contacts_it, ++markers_it, ++options_it)
    {
      if (gripper->rank > grasp_method->grasp_ranks[rank]) {
        grasp_method->grasp_ranks.insert(grasp_method->grasp_ranks.begin() + rank, gripper->rank);
        grasp_method->grasp_poses.insert(grasps_it, gripper->pose);
        grasp_method->grasp_markers.insert(markers_it, gripper->marker);
        grasp_method->grasp_options.insert(options_it, grasp_option);
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
 * Function that returns the roll, pitch and yaw from coplanar points. This method
 * accounts for coodinate systems where the z axis may not be the "downward direction" for
 * grasp approach
 *
 * @param grasp_direction Vector representing the direction of gripper stroke
 * @param grasp_direction_normal Vector representing the direction perpendicular to the stroke
 ******************************************************************************/

std::vector<double> FingerGripper::getPlanarRPY(
  const Eigen::Vector3f & grasp_direction,
  const Eigen::Vector3f & grasp_direction_normal)
{
  std::vector<double> output_vec;
  Eigen::Vector3f x_norm;
  Eigen::Vector3f y_norm;
  Eigen::Vector3f z_norm;
  bool x_filled = false;
  bool y_filled = false;
  bool z_filled = false;

  Eigen::Vector3f grasp_direction_normal_temp;
  auto setDirectionVector = [&](
    const char axis,
    const Eigen::Vector3f grasp_direction,
    bool & x_filled,
    bool & y_filled,
    bool & z_filled,
    Eigen::Vector3f & x_norm,
    Eigen::Vector3f & y_norm,
    Eigen::Vector3f & z_norm) -> void
    {
      auto getDirectionVector = [&](
        Eigen::Vector3f global_vector,
        Eigen::Vector3f direction_vector) -> Eigen::Vector3f
        {
          Eigen::Vector3f direction_vector_temp;
          if (direction_vector.dot(global_vector) > 0) {
            direction_vector_temp = grasp_direction;
          } else {
            direction_vector_temp(0) = -grasp_direction(0);
            direction_vector_temp(1) = -grasp_direction(1);
            direction_vector_temp(2) = -grasp_direction(2);
          }
          return direction_vector_temp.normalized();
        };
      Eigen::Vector3f grasp_direction_temp;
      if (axis == 'x') {
        x_filled = true;
        x_norm = getDirectionVector({1, 0, 0}, grasp_direction);
      } else if (axis == 'y') {
        y_filled = true;
        y_norm = getDirectionVector({0, 1, 0}, grasp_direction);
      } else if (axis == 'z') {
        z_filled = true;
        z_norm = getDirectionVector({0, 0, 1}, grasp_direction);
      }
    };
  //End Lambda

  setDirectionVector(
    this->grasp_stroke_direction, grasp_direction,
    x_filled, y_filled, z_filled,
    x_norm, y_norm, z_norm);

  setDirectionVector(
    this->grasp_stroke_normal_direction, grasp_direction_normal,
    x_filled, y_filled, z_filled,
    x_norm, y_norm, z_norm);

  if (x_filled && y_filled) {
    z_norm = x_norm.cross(y_norm);
  } else if (z_filled && y_filled) {
    x_norm = y_norm.cross(z_norm);
  } else if (x_filled && z_filled) {
    y_norm = z_norm.cross(x_norm);
  } else {
    RCLCPP_ERROR(LOGGER, "RPY estimation error.");
    throw std::runtime_error("RPY estimation error.");
    return {0, 0, 0};
  }

  double roll = std::atan2(-z_norm(1), z_norm(2));
  double pitch = std::asin(z_norm(0));
  double yaw = std::atan2(-y_norm(0), x_norm(0));

  output_vec.push_back(roll);
  output_vec.push_back(pitch);
  output_vec.push_back(yaw);
  return output_vec;
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
  const std::shared_ptr<GraspObject> & object)
{
  geometry_msgs::msg::PoseStamped result_pose;
  result_pose.pose.position.x = gripper->gripper_palm_center.x;
  result_pose.pose.position.y = gripper->gripper_palm_center.y;
  result_pose.pose.position.z = gripper->gripper_palm_center.z;

  tf2::Quaternion quaternion_;

  std::vector<double> rpy = getPlanarRPY(
    gripper->grasping_direction,
    gripper->grasping_normal_direction);
  quaternion_.setRPY(0, 0, rpy[2]);

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
  const Eigen::Vector3f & grasp_direction,
  const Eigen::Vector3f & finger_1,
  const Eigen::Vector3f & finger_2)
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
/***************************************************************************//**
 * Method to reset gripper variables (Not used)
 ******************************************************************************/
void FingerGripper::resetVariables()
{
}

// LCOV_EXCL_START
/***************************************************************************//**
 * Inherited method that visualizes the required grasps
 *
 * @param viewer Projected Cloud Visualizer
 ******************************************************************************/
void FingerGripper::visualizeGrasps(
  pcl::visualization::PCLVisualizer::Ptr viewer,
  std::shared_ptr<GraspObject> object)
{
  PCLVisualizer::centerCamera(object->cloud, viewer);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb2(object->cloud, 255, 0,
    0);

  viewer->addPointCloud<pcl::PointXYZRGB>(object->cloud, rgb2, "cloud_" + object->object_name);

  for (auto const & multigripper : this->sorted_gripper_configs) {
    //Testing
    pcl::PointXYZ grasp_direction;
    grasp_direction.x = multigripper->grasping_direction(0) + multigripper->gripper_palm_center.x;
    grasp_direction.y = multigripper->grasping_direction(1) + multigripper->gripper_palm_center.y;
    grasp_direction.z = multigripper->grasping_direction(2) + multigripper->gripper_palm_center.z;
    pcl::PointXYZ grasp_direction_normal;
    grasp_direction_normal.x = multigripper->grasping_normal_direction(0) +
      multigripper->gripper_palm_center.x;
    grasp_direction_normal.y = multigripper->grasping_normal_direction(1) +
      multigripper->gripper_palm_center.y;
    grasp_direction_normal.z = multigripper->grasping_normal_direction(2) +
      multigripper->gripper_palm_center.z;
    pcl::PointXYZ grasp_approach_direction;
    grasp_approach_direction.x = multigripper->grasp_approach_direction(0) +
      multigripper->gripper_palm_center.x;
    grasp_approach_direction.y = multigripper->grasp_approach_direction(1) +
      multigripper->gripper_palm_center.y;
    grasp_approach_direction.z = multigripper->grasp_approach_direction(2) +
      multigripper->gripper_palm_center.z;
    viewer->addLine(multigripper->gripper_palm_center, grasp_direction, 0, 1, 0, "grasp_direction");
    viewer->addLine(
      multigripper->gripper_palm_center, grasp_direction_normal, 1, 0, 0,
      "grasp_direction_normal");
    viewer->addLine(
      multigripper->gripper_palm_center, grasp_approach_direction, 0, 0, 1,
      "grasp_approach_direction");

    //End Test
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

    viewer->addCoordinateSystem(0.1);
    viewer->addCube(
      object->bboxTransform, object->bboxQuaternion,
      object->maxPoint.x - object->minPoint.x,
      object->maxPoint.y - object->minPoint.y,
      object->maxPoint.z - object->minPoint.z, "bbox_" + object->object_name);


    for (size_t i = 0; i < this->grasp_samples.size(); i++) {
      viewer->addSphere(
        this->grasp_samples[i]->grasp_plane_ncloud->points[
          this->grasp_samples[i]->sample_side_1->start_index],
        0.01, 1.0, 0, 1.0, "sample_side_1 " + std::to_string(i));

      viewer->addSphere(
        this->grasp_samples[i]->grasp_plane_ncloud->points[
          this->grasp_samples[i]->sample_side_2->start_index],
        0.01, 1.0, 0, 1.0, "sample_side_2 " + std::to_string(i));
    }

    viewer->spin();
    viewer->close();
    viewer->removeAllShapes();
  }
  viewer->removeAllPointClouds();
  viewer->removeAllCoordinateSystems();
}
// LCOV_EXCL_STOP

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
  Eigen::Vector3f centerpoint(
    object->centerpoint(0),
    object->centerpoint(1),
    object->centerpoint(2));

  Eigen::Vector3f gradient_vector = object->minor_axis - centerpoint;

  pcl::PointXYZ parallel_object_vector;
  parallel_object_vector.x = gradient_vector(0);
  parallel_object_vector.y = gradient_vector(1);
  parallel_object_vector.z = gradient_vector(2);

  pcl::ModelCoefficients::Ptr parallel_object_plane(new pcl::ModelCoefficients);
  parallel_object_plane->values.resize(4);
  parallel_object_plane->values[0] = gradient_vector(0);
  parallel_object_plane->values[1] = gradient_vector(1);
  parallel_object_plane->values[2] = gradient_vector(2);
  parallel_object_plane->values[3] =
    -((gradient_vector(0) * centerpoint(0)) +
    (gradient_vector(1) * centerpoint(1)) +
    (gradient_vector(2) * centerpoint(2)));
  Eigen::Vector3f perpendicular_grasp_direction = getPerpendicularVectorInPlane(
    grasp_direction, parallel_object_plane);

  return perpendicular_grasp_direction;
}

/***************************************************************************//**
 * Method that generates the grasp option for closed finger distance for
 * finger grippers
 * @param gripper Finger gripper sample
 ******************************************************************************/
emd_msgs::msg::Option FingerGripper::addClosedGraspDistanceOption(
  std::shared_ptr<multiFingerGripper> gripper)
{
  emd_msgs::msg::Option closed_grasp_option;
  closed_grasp_option.header = "closed finger distance";
  closed_grasp_option.value = gripper->closed_finger_stroke;
  return closed_grasp_option;
}
