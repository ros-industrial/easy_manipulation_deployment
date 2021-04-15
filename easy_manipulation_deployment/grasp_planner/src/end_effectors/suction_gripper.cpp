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
#include "grasp_planner/end_effectors/suction_gripper.hpp"
#define PI 3.14159265

SuctionGripper::SuctionGripper(
  std::string id_,
  const int & num_cups_length_,
  const int & num_cups_breadth_,
  const float & dist_between_cups_length_,
  const float & dist_between_cups_breadth_,
  const float & cup_radius_,
  const float & cup_height_,
  const int num_sample_along_axis_,
  const float search_resolution_,
  const int search_angle_resolution_,
  const float cloud_normal_radius_,
  const float curvature_weight_,
  const float grasp_center_distance_weight_,
  const float num_contact_points_weight_)
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
  num_contact_points_weight(num_contact_points_weight_)
{
  this->max_curvature = std::numeric_limits<float>::min();
  this->min_curvature = std::numeric_limits<float>::max();

  this->max_center_dist = std::numeric_limits<float>::min();
  this->min_center_dist = std::numeric_limits<float>::max();

  this->max_contact_points = std::numeric_limits<int>::min();
  this->min_contact_points = std::numeric_limits<int>::max();

  this->length_dim = (this->num_cups_length - 1) * this->dist_between_cups_length;
  this->breadth_dim = (this->num_cups_breadth - 1) * this->dist_between_cups_breadth;
  std::cout << "length dim: " << this->length_dim << std::endl;
  std::cout << "breadth dim: " << this->breadth_dim << std::endl;
}

SuctionGripper::SuctionGripper(
  std::string id_,
  const int & num_cups_length_,
  const int & num_cups_breadth_,
  const float & dist_between_cups_length_,
  const float & dist_between_cups_breadth_,
  const float & cup_radius_,
  const float & cup_height_,
  const int num_sample_along_axis_,
  const float search_resolution_,
  const int search_angle_resolution_,
  const float cloud_normal_radius_)
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
  cloud_normal_radius(cloud_normal_radius_)
{
  // Intialize  maximum and minimum values of ranking variables
  this->max_curvature = std::numeric_limits<float>::min();
  this->min_curvature = std::numeric_limits<float>::max();

  this->max_center_dist = std::numeric_limits<float>::min();
  this->min_center_dist = std::numeric_limits<float>::max();

  this->max_contact_points = std::numeric_limits<int>::min();
  this->min_contact_points = std::numeric_limits<int>::max();

  // Calculate the dimensions of the gripper
  this->length_dim = (this->num_cups_length - 1) * this->dist_between_cups_length;
  this->breadth_dim = (this->num_cups_breadth - 1) * this->dist_between_cups_breadth;

  // Initialize default weights
  this->curvature_weight = 1.0;
  this->grasp_center_distance_weight = 1.0;
  this->num_contact_points_weight = 1.0;
}
void SuctionGripper::planGrasps(
  std::shared_ptr<GraspObject> object,
  emd_msgs::msg::GraspMethod * grasp_method,
  std::shared_ptr<fcl::CollisionObject<float>> world_collision_object,
  pcl::visualization::PCLVisualizer::Ptr viewer)
{
  UNUSED(world_collision_object);
  pcl::PointXYZ object_center;
  object_center.x = object->centerpoint(0);
  object_center.y = object->centerpoint(1);
  object_center.z = object->centerpoint(2);

  // Get highest point of the object to begin grasp search
  pcl::PointXYZRGB object_top_point = findHighestPoint(object->cloud);

  std::cout << "Object center: x " << object->centerpoint(0) << std::endl;
  std::cout << "Object center: y " << object->centerpoint(1) << std::endl;
  std::cout << "Object center: z " << object->centerpoint(2) << std::endl;

  std::cout << "object_top_point: x " << object_top_point.x << std::endl;
  std::cout << "object_top_point: y " << object_top_point.y << std::endl;
  std::cout << "object_top_point: z " << object_top_point.z << std::endl;

  // std::chrono::steady_clock::time_point getAllPossibleGraspsStart =
  // std::chrono::steady_clock::now();
  getAllPossibleGrasps(object, object_center, object_top_point, viewer);
  // std::chrono::steady_clock::time_point getAllPossibleGraspsEnd =
  //  std::chrono::steady_clock::now();
  // std::cout << "Grasp planning time for getAllPossibleGrasps "
  // << std::chrono::duration_cast<std::chrono::milliseconds> (getAllPossibleGraspsEnd -
  //  getAllPossibleGraspsStart).count() << "[ms]" << std::endl;

  // std::chrono::steady_clock::time_point getAllGraspRanksStart = std::chrono::steady_clock::now();
  getAllGraspRanks(grasp_method, object);
  // std::chrono::steady_clock::time_point getAllGraspRanksEnd = std::chrono::steady_clock::now();
  // std::cout << "Grasp planning time for getAllGraspRanks "
  // << std::chrono::duration_cast<std::chrono::milliseconds> (getAllGraspRanksEnd -
  //  getAllGraspRanksStart).count() << "[ms]" << std::endl;
  // std::cout << "Grasp plan size: " << this->cup_array_samples.size() <<std::endl;
}

void SuctionGripper::getAllPossibleGrasps(
  std::shared_ptr<GraspObject> object,
  pcl::PointXYZ object_center,
  pcl::PointXYZRGB top_point,
  pcl::visualization::PCLVisualizer::Ptr viewer)
{
  auto GetBestGrasps1 = [this](
    int i,
    float slice_limit,
    std::shared_ptr<GraspObject> & object,
    pcl::PointXYZ & object_center,
    pcl::ModelCoefficients::Ptr & plane,
    pcl::visualization::PCLVisualizer::Ptr viewer
    ) -> void
    {
      UNUSED(viewer);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal(
        new pcl::PointCloud<pcl::PointNormal>);
      std::vector<std::future<void>> futures_2;


      slice_limit = slice_limit + 0.001 * i;  // How far down to slice the object cloud
      // getSlicedCloud(object->cloud, slice_limit, 0, sliced_cloud);

      /*! \brief A sliced cloud is created to account for noise,
      so we take a range of z values and assume them to be in the same height*/
      getSlicedCloud(
        object->cloud, object->cloud_normal, slice_limit, 0, sliced_cloud,
        sliced_cloud_normal);
      /*! \brief We then make them part of the same plane throguh projection*/
      projectCloudToPlane(sliced_cloud, plane, projected_cloud);
      /*! \brief Get the center index of the sliced cloud,
      which may not necessarily be the center of the object cloud*/
      int centroid_index = getCentroidIndex(projected_cloud);

      // PCLFunctions::computeCloudNormal(
      // sliced_cloud, sliced_cloud_normal, this->cloud_normal_radius);

      // Iterate at different angles to search for best possible grasp
      for (int i = 1, angle = 2, toggle = 1; i < this->search_angle_resolution; i += toggle ^= 1) {
        angle = (toggle == 1 ? i : -i);

        for (int j = 0, k = 0; j < this->num_sample_along_axis; j++, k ^= 1) {
          float object_max_dim = std::max(
            std::max(
              object->maxPoint.x - object->minPoint.x,
              object->maxPoint.y - object->minPoint.y),
            object->maxPoint.z - object->minPoint.z);

          static std::mutex mutex_2;
          // Iterate between the positive and negative sides
          float offset = this->search_resolution * (k == 0 ? j : -j);
          pcl::PointXYZ sample_gripper_center;
          sample_gripper_center.x = projected_cloud->points[centroid_index].x +
            (object->eigenvectors.col(2)(0)) * offset;
          sample_gripper_center.y = projected_cloud->points[centroid_index].y +
            (object->eigenvectors.col(2)(1)) * offset;
          sample_gripper_center.z = slice_limit + (object->eigenvectors.col(2)(2)) * offset;

          // Eigen::Vector3f grasp_direction;
          // grasp_direction(0) = object->eigenvectors.col(1)(0);
          // grasp_direction(1) = object->eigenvectors.col(1)(1);
          // grasp_direction(2) = object->eigenvectors.col(1)(2);
          // Eigen::Vector3f grasp_direction_normalized = grasp_direction/grasp_direction.norm();

          // Eigen::Vector3f object_direction;
          // object_direction(0) = object->eigenvectors.col(2)(0);
          // object_direction(1) = object->eigenvectors.col(2)(1);
          // object_direction(2) = object->eigenvectors.col(2)(2);
          // Eigen::Vector3f object_direction_normalized = object_direction/object_direction.norm();

          Eigen::Vector3f grasp_direction;
          grasp_direction(0) = object->eigenvectors.col(1)(0) * cos(PI / angle) -
            object->eigenvectors.col(1)(1) * sin(PI / angle);
          grasp_direction(1) = object->eigenvectors.col(1)(0) * sin(PI / angle) +
            object->eigenvectors.col(1)(1) * cos(PI / angle);
          grasp_direction(2) = object->eigenvectors.col(1)(2);
          Eigen::Vector3f grasp_direction_normalized = grasp_direction / grasp_direction.norm();

          Eigen::Vector3f object_direction;
          object_direction(0) = object->eigenvectors.col(2)(0) * cos(PI / angle) -
            object->eigenvectors.col(2)(1) * sin(PI / angle);
          object_direction(1) = object->eigenvectors.col(2)(0) * sin(PI / angle) +
            object->eigenvectors.col(2)(1) * cos(PI / angle);
          object_direction(2) = object->eigenvectors.col(2)(2);
          Eigen::Vector3f object_direction_normalized = object_direction / object_direction.norm();

          Eigen::Vector3f grasp_centerpoint;
          grasp_centerpoint(0) = sample_gripper_center.x;
          grasp_centerpoint(1) = sample_gripper_center.y;
          grasp_centerpoint(2) = sample_gripper_center.z;

          bool is_even_breadth = (this->num_cups_breadth % 2 == 0);
          bool is_even_length = (this->num_cups_length % 2 == 0);

          int num_itr_breadth = floor(this->num_cups_breadth / 2) + 1;
          int num_itr_length = floor(this->num_cups_length / 2) + 1;
          float initial_gap_breadth = this->dist_between_cups_breadth /
            (1.0 + (this->num_cups_breadth % 2 == 0 ? 1 : 0));
          float initial_gap_length = this->dist_between_cups_length /
            (1.0 + (this->num_cups_length % 2 == 0 ? 1 : 0));

          suctionCupArray temp_cup_array;
          temp_cup_array.gripper_center = sample_gripper_center;
          temp_cup_array.total_contact_points = 0;
          std::lock_guard<std::mutex> lock(mutex_2);

          // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb (
          // sliced_cloud, 0, 255, 0);
          // viewer->addPointCloud<pcl::PointXYZRGB>(sliced_cloud, rgb, "sliced cloud");
          // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb2 (
          // projected_cloud, 0, 255, 0);
          // viewer->addPointCloud<pcl::PointXYZRGB>(projected_cloud, rgb2, "projected_cloud");
          // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb3 (
          // object->cloud, 255, 0, 0);
          // viewer->addPointCloud<pcl::PointXYZRGB>(object->cloud, rgb3, "object->cloud");
          // viewer->addPointCloud<pcl::PointNormal>(sliced_cloud_normal,"cloud-normal");

          // viewer->spin();
          // viewer->close();
          // viewer->removeAllPointClouds();

          // std::cout<< "Before slice xyzrgb: " << object->cloud->points.size() << std::endl;
          // std::cout<< "Before slice normal:
          //  " << object->cloud_normal->points.size() << std::endl;
          // std::cout<< "After slice xyzrgb: " << sliced_cloud->points.size() << std::endl;
          // std::cout<< "After slice normal: " << sliced_cloud_normal->points.size() << std::endl;
          // std::cout << "Slice limit: " << slice_limit << std::endl;


          for (int row = 0, row_updown_toggle = 0;
            row < (this->breadth_dim < this->length_dim ? num_itr_breadth : num_itr_length);
            row += row_updown_toggle ^= 1)
          {
            float row_gap;
            if (this->breadth_dim < this->length_dim) {
              if (is_even_breadth && row == 0) {
                continue;
              }
              row_gap =
                ((row ==
                0 ? 0 : 1) * initial_gap_breadth +
                (row > 0 ? (row - 1) : 0) * this->dist_between_cups_breadth);
            } else {
              if (is_even_length && row == 0) {
                continue;
              }
              row_gap =
                ((row ==
                0 ? 0 : 1) * initial_gap_length +
                (row > 0 ? (row - 1) : 0) * this->dist_between_cups_length);
            }

            Eigen::Vector3f row_cen_point = grasp_centerpoint +
              (row_updown_toggle == 0 ? row_gap : -row_gap) * grasp_direction_normalized;
            std::vector<std::shared_ptr<singleSuctionCup>> temp_row_array;

            for (int col = 0, col_updown_toggle = 0;
              col < (this->breadth_dim < this->length_dim ? num_itr_length : num_itr_breadth);
              col += col_updown_toggle ^= 1)
            {
              float col_gap;
              if (this->breadth_dim < this->length_dim) {
                // For even numbers, the center axis of the gripper does not contain any gripper
                if (is_even_length && col == 0) {
                  continue;
                }
                col_gap =
                  ((col ==
                  0 ? 0 : 1) * initial_gap_length +
                  (col > 0 ? (col - 1) : 0) * this->dist_between_cups_length);
              } else {
                // For even numbers, the center axis of the gripper does not contain any gripper
                if (is_even_breadth && col == 0) {
                  continue;
                }
                col_gap =
                  ((col ==
                  0 ? 0 : 1) * initial_gap_breadth +
                  (col > 0 ? (col - 1) : 0) * this->dist_between_cups_breadth);
              }

              // pcl::PointXYZ sample_gripper_center;
              Eigen::Vector3f cup_vector = row_cen_point +
                (col_updown_toggle == 0 ? col_gap : -col_gap) * object_direction_normalized;
              // std::cout<< cup_vector << " << CUP VECTOR" << std::endl;
              pcl::PointXYZ cup_point;
              cup_point.x = cup_vector(0);
              cup_point.y = cup_vector(1);
              cup_point.z = cup_vector(2);
              float cup_object_center_dist = pcl::geometry::distance(cup_point, object_center);

              pcl::PointCloud<pcl::PointXYZ>::Ptr disk_cloud(new pcl::PointCloud<pcl::PointXYZ>);

              // Check how many points of the projected pointcloud
              //  land on a suction cup of fixed radius.
              float * curvature_sum_ptr;
              float curvature_sum;
              curvature_sum_ptr = &curvature_sum;
              // std::cout << "Curvature: " << *curvature_sum_ptr << std::endl;
              // std::vector<int> contact_points = createDiskFromCloud(
              // projected_cloud, sliced_cloud_normal, cup_point, this->cup_radius
              // , disk_cloud, curvature_sum_ptr);

              std::vector<int> contact_points = createDiskFromCloud(
                projected_cloud, sliced_cloud_normal, cup_point, this->cup_radius,
                disk_cloud, curvature_sum_ptr);


              int num_contact_points = contact_points.size();
              num_contact_points =
                std::round(
                num_contact_points *
                (this->num_contact_points_weight * ( 1 - cup_object_center_dist / object_max_dim)));

              temp_cup_array.total_contact_points += num_contact_points;
              temp_cup_array.total_curvature += curvature_sum;
              float average_curvature = curvature_sum / contact_points.size();
              singleSuctionCup cup;
              cup.curvature = average_curvature;
              cup.cup_center = cup_point;
              temp_row_array.push_back(std::make_shared<singleSuctionCup>(cup));
            }
            temp_cup_array.cup_array.push_back(temp_row_array);
          }

          temp_cup_array.center_dist =
            pcl::geometry::distance(sample_gripper_center, object_center);
          temp_cup_array.average_curvature = temp_cup_array.total_curvature /
            temp_cup_array.total_contact_points;
          // std::cout << " Total Curvature:
          //  " << temp_cup_array.total_curvature << std::endl;
          // std::cout << " Total Contact points:
          //  " << temp_cup_array.total_contact_points << std::endl;
          // std::cout << "Average Curvature:
          //  " << temp_cup_array.average_curvature << std::endl;
          updateMaxMinValues(
            temp_cup_array.total_contact_points, temp_cup_array.average_curvature,
            temp_cup_array.center_dist);
          temp_cup_array.grasp_angle = PI / angle;
          this->cup_array_samples.push_back(std::make_shared<suctionCupArray>(temp_cup_array));
          // End Test
        }
      }
      // viewer->removeShape("projected_cloud");
    };

  // End lambda
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  getStartingPlane(plane, object, top_point);
  float slice_limit = top_point.z;
  std::vector<std::future<void>> futures_1;

  for (int i = 0; i < static_cast<int>(round(this->cup_height / 0.001)); i++) {
    // Start async
    futures_1.push_back(
      std::async(
        std::launch::async,
        GetBestGrasps1,
        i,
        slice_limit,
        std::ref(object),
        std::ref(object_center),
        std::ref(plane),
        viewer
    ));
  }
  // End Test Projected
  viewer->removeShape("object_cloud");
}

void SuctionGripper::visualizeGrasps(pcl::visualization::PCLVisualizer::Ptr viewer)
{
  if (this->cup_array_samples.size() > 0) {
    for (auto suction_cup_array : this->cup_array_samples) {
      int counter = 0;
      std::cout << "RANK: " << suction_cup_array->rank << std::endl;
      for (auto row : suction_cup_array->cup_array) {
        for (auto cup : row) {
          viewer->addSphere(
            cup->cup_center, this->cup_radius, "test_point" + std::to_string(
              counter));
          counter++;
        }
      }
      viewer->spin();
      viewer->close();
      viewer->removeAllShapes();
    }
  } else {
    std::cout << "No grasps found. Nothing to Visualize" << std::endl;
  }
}
void SuctionGripper::getAllGraspRanks(
  emd_msgs::msg::GraspMethod * grasp_method,
  std::shared_ptr<GraspObject> object)
{
  std::vector<std::shared_ptr<suctionCupArray>> sorted_grasps;
  // std::cout << " max_contact_points :" << this-> max_contact_points << std::endl;
  // std::cout << " min_contact_points :" << this-> min_contact_points << std::endl;
  // std::cout << " max_center_dist :" << this-> max_center_dist << std::endl;
  // std::cout << " min_center_dist :" << this-> min_center_dist << std::endl;
  // std::cout << " max_curvature :" << this-> max_curvature << std::endl;
  // std::cout << " min_curvature :" << this-> min_curvature << std::endl;

  // grasp_method->grasp_poses.clear();
  // grasp_method->grasp_ranks.clear();
  for (auto grasp : this->cup_array_samples) {
    float contact_points_norm = PCLFunctions::normalizeInt(
      grasp->total_contact_points,
      this->min_contact_points, this->max_contact_points);

    float curvature_norm = PCLFunctions::normalize(
      grasp->average_curvature,
      this->min_curvature, this->max_curvature);

    float grasp_center_norm = PCLFunctions::normalize(
      grasp->center_dist,
      this->min_center_dist, this->max_center_dist);


    // std::cout << "Contact points: " << grasp->total_contact_points << std::endl;
    // std::cout << "average_curvature: " << grasp->average_curvature << std::endl;
    // std::cout << "center_dist: " << grasp->center_dist << std::endl;

    // std::cout << "Contact points_norm: " << contact_points_norm << std::endl;
    // std::cout << "average_curvature_norm: " << curvature_norm << std::endl;
    // std::cout << "center_dist_norm: " << grasp_center_norm << std::endl;

    grasp->rank = 2.0 - curvature_norm * this->curvature_weight -
      grasp_center_norm * this->grasp_center_distance_weight +
      contact_points_norm * this->num_contact_points_weight;
    geometry_msgs::msg::PoseStamped grasp_pose = getGraspPose(grasp, object);

    // int rank;
    // std::vector<geometry_msgs::msg::PoseStamped>::iterator grasps_it;
    if (sorted_grasps.size() == 0 && grasp_method->grasp_ranks.empty() &&
      grasp_method->grasp_poses.empty())
    {
      sorted_grasps.push_back(grasp);
      grasp_method->grasp_ranks.push_back(grasp->rank);
      grasp_method->grasp_poses.push_back(grasp_pose);
    } else {
      std::vector<std::shared_ptr<suctionCupArray>>::iterator grasp_it;
      std::vector<geometry_msgs::msg::PoseStamped>::iterator pose_it;
      int index;
      for (index = 0, pose_it = grasp_method->grasp_poses.begin(), grasp_it = sorted_grasps.begin();
        index < static_cast<int>(grasp_method->grasp_ranks.size());
        ++index, ++pose_it, ++grasp_it)
      {
        if (grasp->rank > grasp_method->grasp_ranks[index]) {
          sorted_grasps.insert(grasp_it, grasp);
          grasp_method->grasp_ranks.insert(grasp_method->grasp_ranks.begin() + index, grasp->rank);
          grasp_method->grasp_poses.insert(pose_it, grasp_pose);
          break;
        }
      }
    }
  }
  this->cup_array_samples = sorted_grasps;
}

// Get the point index of the centroid of the object
int SuctionGripper::getCentroidIndex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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

// Provides the cloud representing the contact with suction gripper
// (Only for visualization purposes) (CURRENTLY NOT USED)
bool SuctionGripper::getCupContactCloud(
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

// Provides the cloud point indices. For actual implementation.
//  No need to generate the cloud (CURRENTLY NOT USED)
std::vector<int> SuctionGripper::getCupContactIndices(
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

pcl::PointXYZRGB SuctionGripper::findHighestPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  std::vector<std::future<void>> futures;
  auto checkTopPoint = [this](
    pcl::PointXYZRGB & point,
    pcl::PointXYZRGB & top_point) -> void
    {
      if (point.z < top_point.z) {
        top_point = point;
      }
    };
  pcl::PointXYZRGB top_point;
  top_point.x = std::numeric_limits<float>::max();
  top_point.y = std::numeric_limits<float>::max();
  top_point.z = std::numeric_limits<float>::max();
  for (pcl::PointXYZRGB point : cloud->points) {
    futures.push_back(
      std::async(
        std::launch::async,
        checkTopPoint,
        std::ref(point),
        std::ref(top_point)));
  }
  // for(pcl::PointXYZRGB point : cloud->points)
  // {
  //   std::cout << "Z values: " << point.z << std::endl;
  // }
  return top_point;
}

void SuctionGripper::getStartingPlane(
  pcl::ModelCoefficients::Ptr plane_coefficients,
  std::shared_ptr<GraspObject> object, pcl::PointXYZRGB top_point)
{
  float a = object->eigenvectors.col(0)(0);
  float b = object->eigenvectors.col(0)(1);
  float c = object->eigenvectors.col(0)(2);
  float d = -((a * object->centerpoint(0)) + (b * object->centerpoint(1)) + (c * (top_point.z)));
  plane_coefficients->values.resize(4);
  plane_coefficients->values[0] = a;
  plane_coefficients->values[1] = b;
  plane_coefficients->values[2] = c;
  plane_coefficients->values[3] = d;
}


/***************************************************************************//**
 * Function that slices a cloud to the required limit using a passthrough filter.
 * This function assumes that the direction of filtering is in the z direction,
 * which is parallel to the height vector of the object

 * @param input_cloud Input cloud to be sliced
 * @param input_cloud_normal Input cloud normal to be sliced
 * @param top_limit Top limit for passthrough filter
 * @param bottom_limit Bottom limit for passthrough filter
 * @param sliced_cloud Resultant sliced cloud
 * @param sliced_cloud_normal Resultant sliced cloud
 ******************************************************************************/

void SuctionGripper::getSlicedCloud(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud_normal,
  float top_limit, float bottom_limit, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal)
{
  pcl::PassThrough<pcl::PointXYZRGB> ptFilter;
  ptFilter.setInputCloud(input_cloud);
  ptFilter.setFilterFieldName("z");
  ptFilter.setFilterLimits(bottom_limit, top_limit);
  ptFilter.filter(*sliced_cloud);

  for (auto point : input_cloud_normal->points) {
    if (point.z <= top_limit && point.z >= bottom_limit) {
      sliced_cloud_normal->points.push_back(point);
    }
  }
}

/***************************************************************************//**
 * Function that projects a cloud to the required plane. This function is used
 * to approximate a relatively flat point cloud onto a plane to start grasp sampling

 * @param input_cloud Input cloud to be sliced
 * @param plane_coefficients Input cloud normal to be sliced
 * @param projected_cloud Top limit for passthrough filter
 ******************************************************************************/

void SuctionGripper::projectCloudToPlane(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
  pcl::ModelCoefficients::Ptr plane_coefficients,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud)
{
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(input_cloud);
  proj.setModelCoefficients(plane_coefficients);
  proj.filter(*projected_cloud);
}

std::vector<int> SuctionGripper::createDiskFromCloud(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
  pcl::PointCloud<pcl::PointNormal>::Ptr sliced_cloud_normal,
  pcl::PointXYZ centerpoint, float radius,
  pcl::PointCloud<pcl::PointXYZ>::Ptr disk_cloud,
  float * curvature_sum)
{
  float temp_curvature_sum = 0;
  std::vector<int> points_inside;
  for (int i = 0; i < static_cast<int>(input_cloud->points.size()); i++) {
    float eqn = pow((input_cloud->points[i].x - centerpoint.x), 2) +
      pow((input_cloud->points[i].y - centerpoint.y), 2) - pow(radius, 2);
    if (eqn <= 0) {
      pcl::PointXYZ temp_p;
      temp_p.x = input_cloud->points[i].x;
      temp_p.y = input_cloud->points[i].y;
      temp_p.z = input_cloud->points[i].z;

      disk_cloud->points.push_back(temp_p);
      points_inside.push_back(i);
      temp_curvature_sum += sliced_cloud_normal->points[i].curvature;
    }
  }
  *curvature_sum = temp_curvature_sum;
  return points_inside;
}

void SuctionGripper::updateMaxMinValues(
  int num_contact_points, float average_curvature,
  float center_dist)
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

geometry_msgs::msg::PoseStamped SuctionGripper::getGraspPose(
  std::shared_ptr<suctionCupArray> grasp,
  std::shared_ptr<GraspObject> object)
{
  geometry_msgs::msg::PoseStamped result_pose;
  result_pose.pose.position.x = grasp->gripper_center.x;
  result_pose.pose.position.y = grasp->gripper_center.y;
  result_pose.pose.position.z = grasp->gripper_center.z;

  if (this->num_cups_breadth == this->num_cups_length) {
    result_pose.pose.orientation.x = 0;
    result_pose.pose.orientation.y = 0;
    result_pose.pose.orientation.z = 0;
    result_pose.pose.orientation.w = 1;
  } else {
    Eigen::Matrix3f rotationMatrix;
    rotationMatrix << cos(grasp->grasp_angle), -sin(grasp->grasp_angle), 0,
      sin(grasp->grasp_angle), cos(grasp->grasp_angle), 0,
      0, 0, 1;


    Eigen::Matrix3f rotatedEigenVector = rotationMatrix * object->eigenvectors;

    Eigen::Matrix4f projectionTransform_test(Eigen::Matrix4f::Identity());
    projectionTransform_test.block<3, 3>(0, 0) = rotatedEigenVector.transpose();
    // Can try change object->centerpoint.head
    projectionTransform_test.block<3, 1>(
      0,
      3) = -1.f * (projectionTransform_test.block<3, 3>(0, 0) * object->centerpoint.head<3>());
    this->affine_matrix_test = projectionTransform_test;

    Eigen::Matrix3f PointRotation;
    PointRotation << this->affine_matrix_test(0, 0), this->affine_matrix_test(0, 1),
      this->affine_matrix_test(0, 2),
      this->affine_matrix_test(1, 0), this->affine_matrix_test(1, 1),
      this->affine_matrix_test(1, 2),
      this->affine_matrix_test(2, 0), this->affine_matrix_test(2, 1),
      this->affine_matrix_test(2, 2);


    // Eigen::Matrix3f rotatedEigenVector1 = PointRotation;

    // Eigen::Matrix3f rotatedEigenVector1 =
    // rotationMatrix * (AlignedRotationMatrix * object->eigenvectors);

    tf2::Matrix3x3 graspRotation(PointRotation(0, 0), PointRotation(1, 0), PointRotation(2, 0),
      PointRotation(0, 1), PointRotation(1, 1), PointRotation(2, 1),
      PointRotation(0, 2), PointRotation(1, 2), PointRotation(2, 2));
    double r2, p2, y2;
    graspRotation.getRPY(r2, p2, y2);
    tf2::Quaternion quaternion_test_;
    quaternion_test_.setRPY(0, 0, y2);

    result_pose.pose.orientation.x = quaternion_test_.x();
    result_pose.pose.orientation.y = quaternion_test_.y();
    result_pose.pose.orientation.z = quaternion_test_.z();
    result_pose.pose.orientation.w = quaternion_test_.w();
  }
  const auto clock = std::chrono::system_clock::now();
  result_pose.header.frame_id = object->object_frame;
  result_pose.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(
    clock.time_since_epoch()).count();
  return result_pose;
}
