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
#include "emd/grasp_planner/end_effectors/suction_gripper.hpp"
static const rclcpp::Logger & LOGGER = rclcpp::get_logger("EMD::SuctionGripper");
#define PI 3.14159265

SuctionGripper::SuctionGripper(
  std::string id_,
  const int & num_cups_length_,
  const int & num_cups_breadth_,
  const float & dist_between_cups_length_,
  const float & dist_between_cups_breadth_,
  const float & cup_radius_,
  const float & cup_height_,
  const int & num_sample_along_axis_,
  const float & search_resolution_,
  const int & search_angle_resolution_,
  const float & cloud_normal_radius_,
  const float & curvature_weight_,
  const float & grasp_center_distance_weight_,
  const float & num_contact_points_weight_,
  std::string length_direction_,
  std::string breadth_direction_,
  std::string grasp_approach_direction_)
: id(id_),
  num_cups_length(num_cups_length_),
  num_cups_breadth(num_cups_breadth_),
  dist_between_cups_length(dist_between_cups_length_),
  dist_between_cups_breadth(dist_between_cups_breadth_),
  cup_radius(cup_radius_),
  cup_height(cup_height_),
  num_sample_along_axis(num_sample_along_axis_),
  search_resolution(search_resolution_),
  search_angle_resolution(search_angle_resolution_),
  cloud_normal_radius(cloud_normal_radius_),
  curvature_weight(curvature_weight_),
  grasp_center_distance_weight(grasp_center_distance_weight_),
  num_contact_points_weight(num_contact_points_weight_),
  length_direction(length_direction_[0]),
  breadth_direction(breadth_direction_[0]),
  grasp_approach_direction(grasp_approach_direction_[0])
{
  if (num_cups_length_ <= 0 || num_cups_breadth_ <= 0) {
    RCLCPP_ERROR(LOGGER, "Length and breadth values need to be positive numbers");
    throw std::invalid_argument("Invalid value for field.");
  }

  if (dist_between_cups_length_ <= 0 || dist_between_cups_breadth_ <= 0) {
    RCLCPP_ERROR(LOGGER, "Distance values between length and breadth should be a positive number");
    throw std::invalid_argument("Invalid value for field.");
  }

  if (cup_radius * 2 > dist_between_cups_length ||
    cup_radius * 2 > dist_between_cups_breadth)
  {
    RCLCPP_ERROR(LOGGER, "Suction Cup radius too large, will collide with adjacent fingers");
    throw std::invalid_argument("Invalid value for field.");
  }

  if (cup_radius <= 0 || cup_height_ <= 0) {
    RCLCPP_ERROR(LOGGER, "cup_radius and height needs to be positive and non-zero");
    throw std::invalid_argument("Invalid value for field.");
  }
  if (num_sample_along_axis_ <= 0) {
    RCLCPP_ERROR(LOGGER, "num_sample_along_axis_ variable needs to be positive and non-zero");
    throw std::invalid_argument("Invalid value for field.");
  }

  if (search_resolution <= 0) {
    RCLCPP_ERROR(LOGGER, "search_resolution variable needs to be positive and non-zero");
    throw std::invalid_argument("Invalid value for field.");
  }

  if (search_angle_resolution_ <= 0) {
    RCLCPP_ERROR(LOGGER, "search_angle_resolution_ variable needs to be positive and non-zero");
    throw std::invalid_argument("Invalid value for field.");
  }
  if (
    curvature_weight_ > 1.0 || curvature_weight_ < 0.0 ||
    grasp_center_distance_weight_ > 1.0 || grasp_center_distance_weight_ < 0.0 ||
    num_contact_points_weight_ > 1.0 || num_contact_points_weight_ < 0.0)
  {
    RCLCPP_ERROR(LOGGER, "All Weights need to be a positive value less than 1");
    throw std::invalid_argument("Invalid value for field.");
  }

  this->max_curvature = std::numeric_limits<float>::min();
  this->min_curvature = std::numeric_limits<float>::max();

  this->max_center_dist = std::numeric_limits<float>::min();
  this->min_center_dist = std::numeric_limits<float>::max();

  this->max_contact_points = std::numeric_limits<int>::min();
  this->min_contact_points = std::numeric_limits<int>::max();

  this->length_dim = (this->num_cups_length - 1) * this->dist_between_cups_length;
  this->breadth_dim = (this->num_cups_breadth - 1) * this->dist_between_cups_breadth;
}


void SuctionGripper::generate_gripper_attributes()
{
  bool is_even_breadth = (this->num_cups_breadth % 2 == 0);
  bool is_even_length = (this->num_cups_length % 2 == 0);
  float num_itr_breadth = floor(this->num_cups_breadth / 2) + 1;
  float num_itr_length = floor(this->num_cups_length / 2) + 1;
  float initial_gap_breadth = this->dist_between_cups_breadth /
    (1.0 + (this->num_cups_breadth % 2 == 0 ? 1 : 0));
  float initial_gap_length = this->dist_between_cups_length /
    (1.0 + (this->num_cups_length % 2 == 0 ? 1 : 0));

  if (this->breadth_dim < this->length_dim) {
    this->col_itr = num_itr_length;
    this->col_is_even = is_even_length;
    this->col_dist_between_cups = this->dist_between_cups_length;
    this->col_initial_gap = initial_gap_length;
    this->col_direction = length_direction;

    this->row_itr = num_itr_breadth;
    this->row_is_even = is_even_breadth;
    this->row_dist_between_cups = this->dist_between_cups_breadth;
    this->row_initial_gap = initial_gap_breadth;
    this->row_direction = breadth_direction;
  } else {
    this->col_itr = num_itr_breadth;
    this->col_is_even = is_even_breadth;
    this->col_dist_between_cups = this->dist_between_cups_breadth;
    this->col_initial_gap = initial_gap_breadth;
    this->col_direction = breadth_direction;

    this->row_itr = num_itr_length;
    this->row_is_even = is_even_length;
    this->row_dist_between_cups = this->dist_between_cups_length;
    this->row_initial_gap = initial_gap_length;
    this->row_direction = length_direction;
  }
}

void SuctionGripper::plan_grasps(
  const GraspObject & object,
  emd_msgs::msg::GraspMethod & grasp_method,
  std::shared_ptr<CollisionObject> world_collision_object,
  std::string camera_frame)
{
  // RCLCPP_INFO(LOGGER, "Generate Gripper Attributes");
  generate_gripper_attributes();
  UNUSED(world_collision_object);
  pcl::PointXYZ object_center;
  object_center.x = object.centerpoint(0);
  object_center.y = object.centerpoint(1);
  object_center.z = object.centerpoint(2);
  // Get highest point of the object to begin grasp search
  // RCLCPP_INFO(LOGGER, "Find highest point to initialize grasp search.");

  // For now we say that the height vector is the negative world Z Axis. will change in future.
  pcl::PointXYZRGB object_top_point = find_highest_point(object.cloud, 'z', false);

  // RCLCPP_INFO(LOGGER, "Initializing Grasp sample generation");
  get_all_possible_grasps(object, object_center, object_top_point, camera_frame);
  get_all_grasp_ranks(grasp_method, object);
}

void SuctionGripper::get_all_possible_grasps(
  const GraspObject & object,
  const pcl::PointXYZ & object_center,
  const pcl::PointXYZRGB & top_point,
  std::string camera_frame)
{
  auto get_best_grasps1 = [this](
    int i,
    float slice_limit,
    const GraspObject & object,
    const pcl::PointXYZ & object_center,
    std::string camera_frame,
    pcl::ModelCoefficients::Ptr & plane
    ) -> void
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal(
        new pcl::PointCloud<pcl::PointNormal>);
      std::vector<std::future<void>> futures_2;

      slice_limit = slice_limit + 0.001 * i;  // How far down to slice the object cloud

      /*! \brief A sliced cloud is created to account for noise,
      so we take a range of z values and assume them to be in the same height*/
      // RCLCPP_INFO(LOGGER, "Slice pointcloud");
      get_sliced_cloud(
        object.cloud, object.cloud_normal, slice_limit, 0, sliced_cloud,
        sliced_cloud_normal, 'z');
      /*! \brief We then make them part of the same plane throguh projection*/
      // RCLCPP_INFO(LOGGER, "Project Sliced pointcloud to a plane");
      project_cloud_to_plane(sliced_cloud, plane, projected_cloud);
      /*! \brief Get the center index of the sliced cloud,
      which may not necessarily be the center of the object cloud*/
      // RCLCPP_INFO(LOGGER, "Get the centroid of the projected point cloud");
      int centroid_index = get_centroid_index(projected_cloud);
      // RCLCPP_INFO(LOGGER, "Generate grasp samples");

      // Iterate at different angles to search for best possible grasp
      for (int i = 1, angle = 2, toggle = 1; i < this->search_angle_resolution; i += toggle ^= 1) {
        angle = (toggle == 1 ? i : -i);

        for (int j = 0, k = 0; j < this->num_sample_along_axis; j++, k ^= 1) {

          // Get the maximum direction in the three axes.
          float object_max_dim =
            *std::max_element(std::begin(object.dimensions), std::end(object.dimensions));
          // Iterate between the positive and negative sides
          float offset = this->search_resolution * (k == 0 ? j : -j);

          pcl::PointXYZ sample_gripper_center = get_gripper_center(
            object.axis,
            offset,
            projected_cloud->points[centroid_index]);

          Eigen::Vector3f grasp_direction = MathFunctions::get_rotated_vector(
            object.grasp_axis,
            PI / angle, 'z');

          Eigen::Vector3f object_direction = MathFunctions::get_rotated_vector(
            object.axis,
            PI / angle, 'z');

          static std::mutex mutex_2;
          std::lock_guard<std::mutex> lock(mutex_2);
          // RCLCPP_INFO(LOGGER, "Generate grasp samples");
          SuctionCupArray grasp_sample = generate_grasp_sample(
            projected_cloud,
            sliced_cloud_normal,
            sample_gripper_center,
            object_center,
            grasp_direction,
            object_direction,
            object_max_dim,
            camera_frame);

          update_max_min_values(
            grasp_sample.total_contact_points, grasp_sample.average_curvature,
            grasp_sample.center_dist);

          this->cup_array_samples.push_back(std::make_shared<SuctionCupArray>(grasp_sample));
        }
      }
    };

  // End lambda
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  get_starting_plane(plane, object.minor_axis, object.centerpoint, top_point, 'z');
  float slice_limit = top_point.z;
  // if(object.alignments[2] == 'z')
  // {
  //   slice_limit = top_point.z;
  // }
  // else if(object.alignments[2] == 'y')
  // {
  //   slice_limit = top_point.y;
  // }
  // else if(object.alignments[2] == 'x')
  // {
  //   slice_limit = top_point.x;
  // }

  std::vector<std::future<void>> futures_1;

  for (int i = 0; i < static_cast<int>(round(this->cup_height / 0.001)); i++) {

    // Start async
    futures_1.push_back(
      std::async(
        std::launch::async,
        get_best_grasps1,
        i,
        slice_limit,
        std::ref(object),
        std::ref(object_center),
        std::ref(camera_frame),
        std::ref(plane)
    ));

    // Non Async version
    // get_best_grasps1(i, slice_limit, std::ref(object), std::ref(object_center), std::ref(plane));
  }
}

// LCOV_EXCL_START


void SuctionGripper::visualize_grasps(
  pcl::visualization::PCLVisualizer::Ptr viewer,
  const GraspObject & object)
{
  PCLVisualizer::center_camera(object.cloud, viewer);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb2(object.cloud, 255, 0,
    0);
  viewer->addPointCloud<pcl::PointXYZRGB>(object.cloud, rgb2, "cloud_" + object.object_name);
  if (this->cup_array_samples.size() > 0) {
    for (auto suction_cup_array : this->cup_array_samples) {
      int counter = 0;
      for (auto row : suction_cup_array->cup_array) {
        for (auto cup : row) {
          viewer->addSphere(
            cup->cup_center, this->cup_radius, "test_point" + std::to_string(
              counter));
          counter++;
        }
      }
      viewer->addCoordinateSystem(0.5);
      viewer->addCube(
        object.bboxTransform, object.bboxQuaternion,
        object.maxPoint.x - object.minPoint.x,
        object.maxPoint.y - object.minPoint.y,
        object.maxPoint.z - object.minPoint.z, "bbox_" + object.object_name);
      viewer->spin();
      viewer->close();
      viewer->removeAllShapes();
    }
  } else {
    std::cout << "No grasps found. Nothing to Visualize" << std::endl;
  }
  viewer->removeAllPointClouds();
  viewer->removeAllCoordinateSystems();
}

// LCOV_EXCL_STOP

int SuctionGripper::get_centroid_index(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
{
  Eigen::Vector4f centroid;
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  std::vector<int> kd_radius_search;
  std::vector<float> kd_sq_dist;
  pcl::compute3DCentroid(*(cloud), centroid);
  kdtree.setInputCloud(cloud);
  pcl::PointXYZRGB centroid_point;
  centroid_point.x = centroid(0);
  centroid_point.y = centroid(1);
  centroid_point.z = centroid(2);

  if (kdtree.nearestKSearch(centroid_point, 1, kd_radius_search, kd_sq_dist) > 0) {
    return kd_radius_search[0];
  } else {
    return -1;
  }
}

pcl::PointXYZRGB SuctionGripper::find_highest_point(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  const char & height_axis,
  const bool & is_positive)
{
  std::vector<std::future<void>> futures;
  auto checkTopPoint = [this](
    pcl::PointXYZRGB & point,
    pcl::PointXYZRGB & top_point,
    char height_axis,
    bool is_positive) -> void
    {
      if (height_axis == 'x') {
        if (is_positive) {
          if (point.x > top_point.x) {
            top_point = point;
          }
        } else {
          if (point.x < top_point.x) {
            top_point = point;
          }
        }
      } else if (height_axis == 'y') {
        if (is_positive) {
          if (point.y > top_point.y) {
            top_point = point;
          }
        } else {
          if (point.y < top_point.y) {
            top_point = point;
          }
        }
      } else if (height_axis == 'z') {
        if (is_positive) {
          if (point.z > top_point.z) {
            top_point = point;
          }
        } else {
          if (point.z < top_point.z) {
            top_point = point;
          }
        }
      }
    };
  pcl::PointXYZRGB top_point;
  if (is_positive) {
    top_point.x = std::numeric_limits<float>::min();
    top_point.y = std::numeric_limits<float>::min();
    top_point.z = std::numeric_limits<float>::min();
  } else {
    top_point.x = std::numeric_limits<float>::max();
    top_point.y = std::numeric_limits<float>::max();
    top_point.z = std::numeric_limits<float>::max();
  }


  for (pcl::PointXYZRGB point : cloud->points) {
    futures.push_back(
      std::async(
        std::launch::async,
        checkTopPoint,
        std::ref(point),
        std::ref(top_point),
        height_axis,
        is_positive));
  }
  return top_point;
}

void SuctionGripper::get_starting_plane(
  pcl::ModelCoefficients::Ptr plane_coefficients,
  const Eigen::Vector3f & axis,
  const Eigen::Vector4f & object_centerpoint,
  const pcl::PointXYZRGB & top_point,
  const char & height_axis)
{
  float a = axis(0);
  float b = axis(1);
  float c = axis(2);
  float d = 0;
  if (height_axis == 'x') {
    d = -((a * top_point.x) + (b * object_centerpoint(1)) + (c * object_centerpoint(2)));
  } else if (height_axis == 'y') {
    d = -((a * object_centerpoint(0)) + (b * top_point.y) + (c * object_centerpoint(2)));
  } else if (height_axis == 'z') {
    d = -((a * object_centerpoint(0)) + (b * object_centerpoint(1)) + (c * (top_point.z)));
  }
  // float d = -((a * object_centerpoint(0)) + (b * object_centerpoint(1)) + (c * (top_point.z)));
  plane_coefficients->values.resize(4);
  plane_coefficients->values[0] = a;
  plane_coefficients->values[1] = b;
  plane_coefficients->values[2] = c;
  plane_coefficients->values[3] = d;
}

void SuctionGripper::get_sliced_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
  const pcl::PointCloud<pcl::PointNormal>::Ptr & input_cloud_normal,
  const float & top_limit,
  const float & bottom_limit,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal,
  const char & height_axis)
{
  pcl::PassThrough<pcl::PointXYZRGB> ptFilter;
  ptFilter.setInputCloud(input_cloud);

  if (height_axis == 'x') {
    ptFilter.setFilterFieldName("x");
  } else if (height_axis == 'y') {
    ptFilter.setFilterFieldName("y");
  } else if (height_axis == 'z') {
    ptFilter.setFilterFieldName("z");
  }
  ptFilter.setFilterLimits(bottom_limit, top_limit);
  ptFilter.filter(*sliced_cloud);

  for (auto point : input_cloud_normal->points) {
    if (height_axis == 'x') {
      if (point.x <= top_limit && point.x >= bottom_limit) {
        sliced_cloud_normal->points.push_back(point);
      }
    } else if (height_axis == 'y') {
      if (point.y <= top_limit && point.y >= bottom_limit) {
        sliced_cloud_normal->points.push_back(point);
      }

    } else if (height_axis == 'z') {
      if (point.z <= top_limit && point.z >= bottom_limit) {
        sliced_cloud_normal->points.push_back(point);
      }
    }
  }
}

void SuctionGripper::project_cloud_to_plane(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
  const pcl::ModelCoefficients::Ptr & plane_coefficients,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud)
{
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(input_cloud);
  proj.setModelCoefficients(plane_coefficients);
  proj.filter(*projected_cloud);
}

int SuctionGripper::get_contact_points(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
  const pcl::PointCloud<pcl::PointNormal>::Ptr & sliced_cloud_normal,
  const pcl::PointXYZ & centerpoint,
  float & curvature_sum)
{
  //pcl::PointCloud<pcl::PointXYZ>::Ptr disk_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // std::vector<int> points_inside;
  int num_contact_points = 0;
  for (int i = 0; i < static_cast<int>(input_cloud->points.size()); i++) {
    // Using circle equation, determine if current cloud point is in contact with the suction cup
    float inside_cup = pow((input_cloud->points[i].x - centerpoint.x), 2) +
      pow((input_cloud->points[i].y - centerpoint.y), 2) - pow(this->cup_radius, 2);
    if (inside_cup <= 0) {
      num_contact_points++;
      curvature_sum += sliced_cloud_normal->points[i].curvature;
    }
  }
  return num_contact_points;
}

void SuctionGripper::update_max_min_values(
  const int & num_contact_points,
  const float & average_curvature,
  const float & center_dist)
{
  if (center_dist < this->min_center_dist) {this->min_center_dist = center_dist;}
  if (center_dist > this->max_center_dist) {this->max_center_dist = center_dist;}
  if (num_contact_points < this->min_contact_points) {
    this->min_contact_points = num_contact_points;
  }
  if (num_contact_points > this->max_contact_points) {
    this->max_contact_points = num_contact_points;
  }
  if (average_curvature < this->min_curvature) {this->min_curvature = average_curvature;}
  if (average_curvature > this->max_curvature) {
    // std::cout << "average curvature: " << average_curvature << std::endl;
    this->max_curvature = average_curvature;
  }
}

SuctionCupArray SuctionGripper::generate_grasp_sample(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & projected_cloud,
  const pcl::PointCloud<pcl::PointNormal>::Ptr & sliced_cloud_normal,
  const pcl::PointXYZ & sample_gripper_center,
  const pcl::PointXYZ & object_center,
  const Eigen::Vector3f & row_direction,
  const Eigen::Vector3f & col_direction,
  const float & object_max_dim,
  std::string camera_frame)
{

  SuctionCupArray grasp_sample(sample_gripper_center, row_direction, col_direction);
  Eigen::Vector3f sample_gripper_center_eigen = PCLFunctions::convert_pcl_to_eigen(
    sample_gripper_center);
  float total_contact_points = 0;
  float total_curvature = 0;

  grasp_sample.marker.header.frame_id = camera_frame;
  grasp_sample.marker.ns = "";
  grasp_sample.marker.type = grasp_sample.marker.SPHERE_LIST;
  grasp_sample.marker.action = grasp_sample.marker.ADD;
  grasp_sample.marker.lifetime = rclcpp::Duration::from_seconds(20);
  grasp_sample.marker.scale.x = 0.02;
  grasp_sample.marker.scale.y = 0.02;
  grasp_sample.marker.scale.z = 0.02;
  grasp_sample.marker.color.r = 1.0f;
  grasp_sample.marker.color.a = 1.0;

  for (int row = 0, row_updown_toggle = 0; row < this->row_itr; row += row_updown_toggle ^= 1) {

    // Get the constant gap between rows in each column
    float row_gap = get_gap(
      row, this->row_is_even, this->row_initial_gap, this->row_dist_between_cups);

    if (row_gap < 0) { // For even grippers, the first iteration has no cup generation
      continue;
    }
    (row_updown_toggle == 0 ? row_gap = row_gap : row_gap = -row_gap);
    Eigen::Vector3f row_cen_point = MathFunctions::get_point_in_direction(
      sample_gripper_center_eigen, row_direction,
      row_gap);

    std::vector<std::shared_ptr<SingleSuctionCup>> temp_row_array;

    for (int col = 0, col_updown_toggle = 0; col < this->col_itr; col += col_updown_toggle ^= 1) {

      float col_gap = get_gap(
        col, this->col_is_even, this->col_initial_gap, this->col_dist_between_cups);

      if (col_gap < 0) { // For even grippers, the first iteration has no cup generation
        continue;
      }
      (col_updown_toggle == 0 ? col_gap = col_gap : col_gap = -col_gap);

      Eigen::Vector3f cup_vector = MathFunctions::get_point_in_direction(
        row_cen_point, col_direction,
        col_gap);

      // Setting coordinates of individual cups into marker variable
      geometry_msgs::msg::Point cup_marker_point;
      cup_marker_point.x = cup_vector(0);
      cup_marker_point.y = cup_vector(1);
      cup_marker_point.z = cup_vector(2);

      SingleSuctionCup cup = generate_suction_cup(
        projected_cloud, sliced_cloud_normal,
        cup_vector, object_center, object_max_dim);
      total_contact_points += cup.weighted_contact_points;

      total_curvature += cup.curvature_sum;

      grasp_sample.marker.points.push_back(cup_marker_point);
      temp_row_array.push_back(std::make_shared<SingleSuctionCup>(cup));
    }
    grasp_sample.cup_array.push_back(temp_row_array);
  }
  grasp_sample.total_contact_points = total_contact_points;
  grasp_sample.total_curvature = total_curvature;
  grasp_sample.center_dist = pcl::geometry::distance(sample_gripper_center, object_center);
  grasp_sample.average_curvature = grasp_sample.total_curvature / grasp_sample.total_contact_points;

  return grasp_sample;
}

SingleSuctionCup SuctionGripper::generate_suction_cup(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & projected_cloud,
  const pcl::PointCloud<pcl::PointNormal>::Ptr & sliced_cloud_normal,
  const Eigen::Vector3f & suction_cup_center,
  const pcl::PointXYZ & object_center,
  const float & object_max_dim)
{
  pcl::PointXYZ cup_point;
  cup_point.x = suction_cup_center(0);
  cup_point.y = suction_cup_center(1);
  cup_point.z = suction_cup_center(2);
  float cup_object_center_dist = pcl::geometry::distance(cup_point, object_center);

  // Check how many points of the projected pointcloud land on a suction cup.
  float curvature_sum = 0;

  int contact_points = get_contact_points(
    projected_cloud, sliced_cloud_normal, cup_point, curvature_sum);

  int weighted_contact_points = generate_weighted_contact_points(
    contact_points,
    cup_object_center_dist,
    object_max_dim);

  return SingleSuctionCup(cup_point, curvature_sum, contact_points, weighted_contact_points);
}

float SuctionGripper::get_gap(
  const int & itr,
  const bool & is_even,
  const float & initial_gap,
  const float & dist_between_cups)
{
  if (is_even && itr == 0) {return -1;} else {
    return (itr == 0 ? 0 : 1) * initial_gap +
           (itr > 0 ? (itr - 1) : 0) * dist_between_cups;
  }
}

int SuctionGripper::generate_weighted_contact_points(
  const int & contact_points,
  const float & cup_to_center_dist,
  const float & object_max_dim)
{
  return std::round(
    contact_points *
    (this->num_contact_points_weight *
    (1 - cup_to_center_dist / object_max_dim)));
}

pcl::PointXYZ SuctionGripper::get_gripper_center(
  const Eigen::Vector3f & object_axis,
  const float & offset,
  const pcl::PointXYZRGB & slice_centroid)
{
  Eigen::Vector3f slice_centroid_eigen = PCLFunctions::convert_pcl_to_eigen(slice_centroid);
  Eigen::Vector3f result_vector = MathFunctions::get_point_in_direction(
    slice_centroid_eigen,
    object_axis, offset);
  pcl::PointXYZ sample_gripper_center;
  sample_gripper_center.x = result_vector(0);
  sample_gripper_center.y = result_vector(1);
  sample_gripper_center.z = result_vector(2);

  // if(height_axis == 'x'){
  //   std::cout << "x!" << std::endl;
  //   sample_gripper_center.x = slice_centroid.x + (object_axis(0)) * offset;
  //   sample_gripper_center.y = slice_centroid.y;
  //   sample_gripper_center.z = slice_centroid.z;
  // }
  // else if(height_axis == 'y'){
  //   std::cout << "y!" << std::endl;
  //   sample_gripper_center.x = slice_centroid.x;
  //   sample_gripper_center.y = slice_centroid.y + (object_axis(1)) * offset;
  //   sample_gripper_center.z = slice_centroid.z;
  // }
  // else if(height_axis == 'z'){
  //   std::cout << "z!" << std::endl;
  //   sample_gripper_center.x = slice_centroid.x;
  //   sample_gripper_center.y = slice_centroid.y;
  //   sample_gripper_center.z = slice_centroid.z + (object_axis(2)) * offset;
  // }
  return sample_gripper_center;
}

void SuctionGripper::get_all_grasp_ranks(
  emd_msgs::msg::GraspMethod & grasp_method,
  const GraspObject & object)
{
  std::vector<std::shared_ptr<SuctionCupArray>> sorted_grasps;
  for (auto grasp : this->cup_array_samples) {
    float contact_points_norm = MathFunctions::normalize_int(
      grasp->total_contact_points,
      this->min_contact_points, this->max_contact_points);

    float curvature_norm = MathFunctions::normalize(
      grasp->average_curvature,
      this->min_curvature, this->max_curvature);

    float grasp_center_norm = MathFunctions::normalize(
      grasp->center_dist,
      this->min_center_dist, this->max_center_dist);

    grasp->rank = 2.0 - curvature_norm * this->curvature_weight -
      grasp_center_norm * this->grasp_center_distance_weight +
      contact_points_norm * this->num_contact_points_weight;
    geometry_msgs::msg::PoseStamped grasp_pose = get_grasp_pose(grasp, object);

    if (sorted_grasps.size() == 0 && grasp_method.grasp_ranks.empty() &&
      grasp_method.grasp_poses.empty())
    {
      sorted_grasps.push_back(grasp);
      grasp_method.grasp_ranks.push_back(grasp->rank);
      grasp_method.grasp_poses.push_back(grasp_pose);
      grasp_method.grasp_markers.push_back(grasp->marker);
    } else {
      std::vector<std::shared_ptr<SuctionCupArray>>::iterator grasp_it;
      std::vector<geometry_msgs::msg::PoseStamped>::iterator pose_it;
      std::vector<visualization_msgs::msg::Marker>::iterator markers_it;

      int index;
      for (index = 0, pose_it = grasp_method.grasp_poses.begin(), grasp_it = sorted_grasps.begin(),
        markers_it = grasp_method.grasp_markers.begin();
        index < static_cast<int>(grasp_method.grasp_ranks.size());
        ++index, ++pose_it, ++grasp_it, ++markers_it)
      {
        if (grasp->rank > grasp_method.grasp_ranks[index]) {
          sorted_grasps.insert(grasp_it, grasp);
          grasp_method.grasp_ranks.insert(grasp_method.grasp_ranks.begin() + index, grasp->rank);
          grasp_method.grasp_poses.insert(pose_it, grasp_pose);
          grasp_method.grasp_markers.insert(markers_it, grasp->marker);
          break;
        }
      }
    }
  }
  this->cup_array_samples = sorted_grasps;
}

std::vector<double> SuctionGripper::get_planar_rpy(
  const Eigen::Vector3f & col_vector,
  const Eigen::Vector3f & row_vector)
{
  std::vector<double> output_vec;
  Eigen::Vector3f x_norm;
  Eigen::Vector3f y_norm;
  Eigen::Vector3f z_norm;
  bool x_filled = false;
  bool y_filled = false;
  bool z_filled = false;

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

  // if (this->col_direction == 'x') {
  //   x_norm = col_vector.normalized();
  //   x_filled = true;
  // } else if (this->col_direction == 'y') {
  //   y_norm = col_vector.normalized();
  //   y_filled = true;
  // } else if (this->col_direction == 'z') {
  //   z_norm = col_vector.normalized();
  //   z_filled = true;
  // }

  // if (this->row_direction == 'x') {
  //   x_norm = row_vector.normalized();
  //   x_filled = true;
  // } else if (this->row_direction == 'y') {
  //   y_norm = row_vector.normalized();
  //   y_filled = true;
  // } else if (this->row_direction == 'z') {
  //   z_norm = row_vector.normalized();
  //   z_filled = true;
  // }

  setDirectionVector(
    this->col_direction, col_vector,
    x_filled, y_filled, z_filled,
    x_norm, y_norm, z_norm);

  setDirectionVector(
    this->row_direction, row_vector,
    x_filled, y_filled, z_filled,
    x_norm, y_norm, z_norm);

  if (x_filled && y_filled) {
    z_norm = x_norm.cross(y_norm);
  } else if (z_filled && y_filled) {
    x_norm = y_norm.cross(z_norm);
  } else if (x_filled && z_filled) {
    y_norm = z_norm.cross(x_norm);
  } else {
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

geometry_msgs::msg::PoseStamped SuctionGripper::get_grasp_pose(
  const std::shared_ptr<SuctionCupArray> & grasp,
  const GraspObject & object)
{
  geometry_msgs::msg::PoseStamped result_pose;
  result_pose.pose.position.x = grasp->gripper_center.x;
  result_pose.pose.position.y = grasp->gripper_center.y;
  result_pose.pose.position.z = grasp->gripper_center.z;

  std::vector<double> rpy = get_planar_rpy(grasp->col_direction, grasp->row_direction);
  tf2::Quaternion quaternion_test_;
  quaternion_test_.setRPY(0, 0, rpy[2]);

  result_pose.pose.orientation.x = quaternion_test_.x();
  result_pose.pose.orientation.y = quaternion_test_.y();
  result_pose.pose.orientation.z = quaternion_test_.z();
  result_pose.pose.orientation.w = quaternion_test_.w();

  const auto clock = std::chrono::system_clock::now();
  result_pose.header.frame_id = object.object_frame;
  result_pose.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(
    clock.time_since_epoch()).count();
  return result_pose;
}

// LCOV_EXCL_START

bool SuctionGripper::get_cup_contact_cloud(
  pcl::PointXYZRGB contact_point,
  float radius,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output)
{
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  std::vector<int> kd_radius_search;
  std::vector<float> kd_sq_dist;
  kdtree.setInputCloud(cloud_input);
  if (kdtree.radiusSearch(contact_point, radius, kd_radius_search, kd_sq_dist) > 0) {
    for (std::size_t m = 0; m < kd_radius_search.size(); ++m) {
      pcl::PointXYZ temp_p;
      temp_p.x = cloud_input->points[kd_radius_search[m]].x;
      temp_p.y = cloud_input->points[kd_radius_search[m]].y;
      temp_p.z = cloud_input->points[kd_radius_search[m]].z;
      cloud_output->points.push_back(temp_p);
    }
    return true;
  } else {
    return false;
  }
}

std::vector<int> SuctionGripper::get_cup_contact_indices(
  pcl::PointXYZRGB contact_point,
  float radius,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input)
{
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  std::vector<int> kd_radius_search;
  std::vector<float> kd_sq_dist;
  kdtree.setInputCloud(cloud_input);
  kdtree.radiusSearch(contact_point, radius, kd_radius_search, kd_sq_dist);
  return kd_radius_search;
}

// LCOV_EXCL_STOP
