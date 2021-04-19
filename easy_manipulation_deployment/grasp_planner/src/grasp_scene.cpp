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
#include "grasp_planner/grasp_scene.hpp"

namespace uuid
{
static std::random_device rd;
static std::mt19937 gen(rd());
static std::uniform_int_distribution<> dis(0, 15);
static std::uniform_int_distribution<> dis2(8, 11);

std::string generate_uuid()
{
  std::stringstream ss;
  int i;
  ss << std::hex;
  for (i = 0; i < 8; i++) {
    ss << dis(gen);
  }
  ss << "-";
  for (i = 0; i < 4; i++) {
    ss << dis(gen);
  }
  ss << "-4";
  for (i = 0; i < 3; i++) {
    ss << dis(gen);
  }
  ss << "-";
  ss << dis2(gen);
  for (i = 0; i < 3; i++) {
    ss << dis(gen);
  }
  ss << "-";
  for (i = 0; i < 12; i++) {
    ss << dis(gen);
  }
  return ss.str();
}
}  // namespace uuid

GraspScene::~GraspScene()
{}

void GraspScene::planning_init_epd(const epd_msgs::msg::EPDObjectLocalization::ConstSharedPtr & msg)
{
  std::cout << "Inside EPD callback" << std::endl;
  std::cout << "object sizes: " << msg->objects.size() << std::endl;
  EPDCreateWorldCollisionObject(msg);
  this->grasp_objects = processEPDObjects(
    msg->objects,
    this->get_parameter("camera_frame").as_string(),
    static_cast<float>(this->get_parameter("point_cloud_params.cloud_normal_radius").as_double()));
  loadEndEffectors();
  // PCLFunctions::viewerAddRGBCloud(cloud, "original_cloud", viewer);

  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb (
  // object->cloud, 255, 0, 0);
  // viewer->addPointCloud<pcl::PointXYZRGB> (
  // object->cloud, rgb, "rectangle_cloud" + object->object_name);

  emd_msgs::msg::GraspTask grasp_task;
  grasp_task.task_id = uuid::generate_uuid();
  for (auto object : this->grasp_objects) {
    // PCLFunctions::centerCamera(object->cloud, viewer);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for (auto gripper : this->end_effectors) {
      std::chrono::steady_clock::time_point grasp_begin = std::chrono::steady_clock::now();
      emd_msgs::msg::GraspMethod grasp_method;
      grasp_method.ee_id = gripper->getID();
      grasp_method.grasp_ranks.insert(
        grasp_method.grasp_ranks.begin(), std::numeric_limits<float>::min());
      gripper->planGrasps(object, &grasp_method, world_collision_object, viewer);
      grasp_method.grasp_ranks.pop_back();
      object->grasp_target.grasp_methods.push_back(grasp_method);
      std::chrono::steady_clock::time_point grasp_end = std::chrono::steady_clock::now();
      std::cout << "Grasp planning time for " << grasp_method.ee_id << " : " <<
        std::chrono::duration_cast<std::chrono::milliseconds>(grasp_end - grasp_begin).count() <<
        "[ms]" << std::endl;
      gripper->visualizeGrasps(viewer);
    }

    // for(int method_itr = 0; method_itr < static_cast<int>(
    // object->grasp_target.grasp_methods.size()); method_itr++)
    // {
    //   for(int pose_itr = 0; pose_itr < static_cast<int>(
    // object->grasp_target.grasp_methods[method_itr].grasp_poses.size()); pose_itr++)
    //   {
    //     object->grasp_target.target_pose.pose.position.z += 0.1;
    //     object->grasp_target.target_pose.pose.position.y -= 0.1;
    //     object->
    //     grasp_target.grasp_methods[method_itr].grasp_poses[pose_itr].pose.position.z += 0.1;
    //     object->
    //     grasp_target.grasp_methods[method_itr].grasp_poses[pose_itr].pose.position.y -= 0.1;
    //     grasp_task.grasp_targets.push_back(object->grasp_target);
    //   }
    // }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Grasp planning time for object: " <<
      std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" <<
      std::endl;
    grasp_task.grasp_targets.push_back(object->grasp_target);
    viewer->removeAllPointClouds();
  }
  viewer->removeShape("original_cloud");
  std::cout << "Publishing topic" << std::endl;
  this->output_pub->publish(grasp_task);
  this->end_effectors.clear();
  this->grasp_objects.clear();
}

void GraspScene::planning_init(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
// void planning_init(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::cout << " Direct Point Cloud Grasp Planning:  " << std::endl;
  processPointCloud(msg);
  createWorldCollision(msg);
  PCLFunctions::centerCamera(this->cloud, viewer);
  this->grasp_objects = extractObjects(
    this->get_parameter("camera_frame").as_string(),
    static_cast<float>(this->get_parameter("point_cloud_params.cloud_normal_radius").as_double()),
    cloud_plane_removed, cluster_tolerance, min_cluster_size);
  loadEndEffectors();
  PCLFunctions::viewerAddRGBCloud(cloud, "original_cloud", viewer);
  emd_msgs::msg::GraspTask grasp_task;
  grasp_task.task_id = uuid::generate_uuid();
  for (auto object : this->grasp_objects) {
    PCLFunctions::centerCamera(object->cloud, viewer);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for (auto gripper : this->end_effectors) {
      std::chrono::steady_clock::time_point grasp_begin = std::chrono::steady_clock::now();
      emd_msgs::msg::GraspMethod grasp_method;
      grasp_method.ee_id = gripper->getID();
      grasp_method.grasp_ranks.insert(
        grasp_method.grasp_ranks.begin(), std::numeric_limits<float>::min());
      gripper->planGrasps(object, &grasp_method, world_collision_object, viewer);
      grasp_method.grasp_ranks.pop_back();
      object->grasp_target.grasp_methods.push_back(grasp_method);
      std::chrono::steady_clock::time_point grasp_end = std::chrono::steady_clock::now();
      std::cout << "Grasp planning time for " << grasp_method.ee_id << " : " <<
        std::chrono::duration_cast<std::chrono::milliseconds>(grasp_end - grasp_begin).count() <<
        "[ms]" << std::endl;
      for(auto rank : grasp_method.grasp_ranks)
      {
        std::cout << "grasp method rank: " << rank << std::endl;
      }
      gripper->visualizeGrasps(viewer);
    }

    // for(int method_itr = 0;  method_itr < static_cast<int>(
    // object->grasp_target.grasp_methods.size()); method_itr++)
    // {
    //   for(int pose_itr = 0; pose_itr < static_cast<int>(
    // object->grasp_target.grasp_methods[method_itr].grasp_poses.size()); pose_itr++)
    //   {
    //     object->grasp_target.target_pose.pose.position.z += 0.1;
    //     object->grasp_target.target_pose.pose.position.y -= 0.1;
    //     object->
    //     grasp_target.grasp_methods[method_itr].grasp_poses[pose_itr].pose.position.z += 0.1;
    //     object->
    //     grasp_target.grasp_methods[method_itr].grasp_poses[pose_itr].pose.position.y -= 0.1;
    //     grasp_task.grasp_targets.push_back(object->grasp_target);
    //   }
    // }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " <<
    // std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() <<
    //  "[µs]" << std::endl;
    std::cout << "Grasp planning time for object: " <<
      std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" <<
      std::endl;
    grasp_task.grasp_targets.push_back(object->grasp_target);
  }
  viewer->removeShape("original_cloud");
  std::cout << "Publishing topic" << std::endl;
  this->output_pub->publish(grasp_task);
  this->end_effectors.clear();
  this->grasp_objects.clear();
}


void GraspScene::getCameraPosition()
{
  Eigen::Vector3f worldZVector(0, 0, 1);
  Eigen::Vector3f table_normal_vector(this->table_coeff->values[0],
    this->table_coeff->values[1],
    this->table_coeff->values[2]);

  float cos_world_table = std::abs(
    (table_normal_vector.dot(worldZVector)) /
    (table_normal_vector.norm() * worldZVector.norm()));

  std::cout << table_normal_vector(0) << " , " << table_normal_vector(1) << " , " <<
    table_normal_vector(2) << std::endl;
  std::cout << "Angle: " << cos_world_table << std::endl;
  // Compute initial points accordingly
  if (cos_world_table > 0.9) {
    std::cout << "Camera in top view\n";
  } else {
    std::cout << "Camera in side view\n";
  }
}

std::vector<std::shared_ptr<GraspObject>> GraspScene::extractObjects(
  std::string camera_frame,
  float cloud_normal_radius,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  float cluster_tolerance,
  int min_cluster_size)
{
  std::vector<std::shared_ptr<GraspObject>> grasp_objects;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecExtractor;

  tree->setInputCloud(cloud);
  ecExtractor.setClusterTolerance(cluster_tolerance);
  ecExtractor.setMinClusterSize(min_cluster_size);
  ecExtractor.setSearchMethod(tree);
  ecExtractor.setInputCloud(cloud);
  ecExtractor.extract(clusterIndices);
  if (clusterIndices.empty()) {
    std::cout << "No objects. " << std::endl;
  } else {
    std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();
    // int objectNumber = 0;
    for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      for (std::vector<int>::const_iterator pit = it->indices.begin();
        pit != it->indices.end(); ++pit)
      {
        objectCloud->points.push_back(cloud->points[*pit]);
      }

      objectCloud->width = objectCloud->points.size();
      objectCloud->height = 1;
      objectCloud->is_dense = true;

      // Get the centroid of the point cloud
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*objectCloud, centroid);
      std::shared_ptr<GraspObject> object = std::make_shared<GraspObject>(
        camera_frame, objectCloud,
        centroid);
      PCLFunctions::computeCloudNormal(objectCloud, object->cloud_normal, cloud_normal_radius);
      object->get_object_bb();
      object->get_object_world_angles();
      object->grasp_target.target_shape = object->getObjectShape();
      object->grasp_target.target_pose = object->getObjectPose(camera_frame);
      grasp_objects.push_back(object);

      // viewer->removeAllPointClouds();
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb2(object->cloud, 255, 0,
        0);
      viewer->addPointCloud<pcl::PointXYZRGB>(object->cloud, rgb2, "cloud_" + object->object_name);
      viewer->addCube(
        object->bboxTransform, object->bboxQuaternion,
        object->maxPoint.x - object->minPoint.x,
        object->maxPoint.y - object->minPoint.y,
        object->maxPoint.z - object->minPoint.z, "bbox_" + object->object_name);
    }
  }
  return grasp_objects;
}

std::vector<std::shared_ptr<GraspObject>> GraspScene::processEPDObjects(
  std::vector<epd_msgs::msg::LocalizedObject> objects,
  std::string camera_frame,
  float cloud_normal_radius)
{
  std::vector<std::shared_ptr<GraspObject>> grasp_objects;
  for (auto raw_object : objects) {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PCLPointCloud2 * pcl_pc2(new pcl::PCLPointCloud2);
    PCLFunctions::SensorMsgtoPCLPointCloud2((raw_object.segmented_pcl), *pcl_pc2);
    pcl::fromPCLPointCloud2(*pcl_pc2, *(objectCloud));
    // PCLFunctions::removeAllZeros(objectCloud);
    PCLFunctions::removeStatisticalOutlier(objectCloud, 0.5);


    objectCloud->width = objectCloud->points.size();
    objectCloud->height = 1;
    objectCloud->is_dense = true;

    // Get the centroid of the point cloud
    Eigen::Vector4f centroid;
    centroid(0) = raw_object.centroid.x;
    centroid(1) = raw_object.centroid.y;
    centroid(2) = raw_object.centroid.z;


    // pcl::compute3DCentroid(*objectCloud, centroid);
    std::shared_ptr<GraspObject> object = std::make_shared<GraspObject>(
      raw_object.name,
      camera_frame, objectCloud,
      centroid);
    PCLFunctions::computeCloudNormal(objectCloud, object->cloud_normal, cloud_normal_radius);
    object->get_object_bb();
    object->get_object_world_angles();
    object->grasp_target.target_shape = object->getObjectShape();
    object->grasp_target.target_pose = object->getObjectPose(camera_frame);
    grasp_objects.push_back(object);

    // viewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb2(object->cloud, 255, 0,
      0);
    viewer->addPointCloud<pcl::PointXYZRGB>(object->cloud, rgb2, "cloud_" + object->object_name);
    viewer->addCube(
      object->bboxTransform, object->bboxQuaternion,
      object->maxPoint.x - object->minPoint.x,
      object->maxPoint.y - object->minPoint.y,
      object->maxPoint.z - object->minPoint.z, "bbox_" + object->object_name);
  }
  std::cout << "processEPDObjects size: " << grasp_objects.size() << std::endl;
  return grasp_objects;
}

void GraspScene::EPDCreateWorldCollisionObject(
  const epd_msgs::msg::EPDObjectLocalization::ConstSharedPtr & msg)
{
  float ppx = 323.3077697753906;
  float fx = 610.3740844726562;
  float ppy = 235.43516540527344;
  float fy = 609.8685913085938;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg->depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
  cv::Mat depth_img = cv_ptr->image;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  for (size_t i = 0; i < msg->depth_image.width; i++) {
    for (size_t j = 0; j < msg->depth_image.height; j++) {
      pcl::PointXYZRGB temp_point;
      auto depth = depth_img.at<ushort>(j, i) * 0.001;    // NOLINT
      temp_point.x = (i - ppx) / fx * depth;
      temp_point.y = (j - ppy) / fy * depth;
      temp_point.z = depth;
      scene_cloud->points.push_back(temp_point);
    }
  }

  PCLFunctions::passthroughFilter(
    scene_cloud,
    this->ptFilter_Ulimit_x,
    this->ptFilter_Llimit_x,
    this->ptFilter_Ulimit_y,
    this->ptFilter_Llimit_y,
    this->ptFilter_Ulimit_z,
    this->ptFilter_Llimit_z);
  PCLFunctions::removeStatisticalOutlier(scene_cloud, 1.0);

  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb_r(scene_cloud, 255, 0, 0);
  // viewer->addPointCloud<pcl::PointXYZRGB>(scene_cloud, rgb_r, "rectangle_cloud");
  // viewer->spin();
  // viewer->close();


  geometry_msgs::msg::TransformStamped sensorToWorldTf =
    this->buffer_->lookupTransform(
    "base_link", msg->header.frame_id,
    msg->header.stamp);
  octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation);
  PCLFunctions::voxelizeCloud<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
    pcl::VoxelGrid<pcl::PointXYZRGB>>(scene_cloud, 0.005, this->org_cloud);
  this->world_collision_object = FCLFunctions::createCollisionObjectFromPointCloudRGB(
    this->org_cloud, sensor_origin, 0.01);
  // PCLFunctions::planeSegmentation(this->cloud, this->cloud_plane_removed,
  //  this->cloud_table, this->segmentation_max_iterations, segmentation_distance_threshold);
}

void GraspScene::loadEndEffectors()
{
  std::vector<std::string> end_effector_array = this->get_parameter(
    "end_effectors.end_effector_names").as_string_array();
  for (std::string end_effector : end_effector_array) {
    std::string end_effector_type =
      this->get_parameter("end_effectors." + end_effector + ".type").as_string();
    if (end_effector_type.compare("finger") == 0) {
      std::cout << "[Grasp Planning] Finger Gripper Available." << std::endl;
      FingerGripper gripper(
        end_effector,
        this->get_parameter("end_effectors." + end_effector + ".num_fingers_side_1").as_int(),
        this->get_parameter("end_effectors." + end_effector + ".num_fingers_side_2").as_int(),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".distance_between_fingers_1").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".distance_between_fingers_2").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".finger_thickness").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".gripper_stroke").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.voxel_size").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.grasp_rank_weight_1").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.grasp_rank_weight_2").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.grasp_plane_dist_limit").as_double()),
        static_cast<float>(this->get_parameter(
          "point_cloud_params.cloud_normal_radius").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.world_x_angle_threshold").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.world_y_angle_threshold").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.world_z_angle_threshold").as_double())
      );
      std::shared_ptr<EndEffector> gripper_ptr = std::make_shared<FingerGripper>(gripper);
      this->end_effectors.push_back(gripper_ptr);
    } else if (end_effector_type.compare("suction") == 0) {
      std::cout << "[Grasp Planning] Suction Gripper Available." << std::endl;

      SuctionGripper gripper(
        end_effector,
        this->get_parameter("end_effectors." + end_effector + ".num_cups_length").as_int(),
        this->get_parameter("end_effectors." + end_effector + ".num_cups_breadth").as_int(),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector + ".dist_between_cups_length").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector + ".dist_between_cups_breadth").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector + ".cup_radius").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector + ".cup_height").as_double()),
        this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.num_sample_along_axis").as_int(),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.search_resolution").as_double()),
        this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.search_angle_resolution").as_int(),
        static_cast<float>(this->get_parameter(
          "point_cloud_params.cloud_normal_radius").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.weights.curvature").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.weights.grasp_distance_to_center").as_double()),
        static_cast<float>(this->get_parameter(
          "end_effectors." + end_effector +
          ".grasp_planning_params.weights.number_contact_points").as_double()));

      std::shared_ptr<EndEffector> gripper_ptr = std::make_shared<SuctionGripper>(gripper);

      this->end_effectors.push_back(gripper_ptr);
    }
  }
}

void GraspScene::printPose(const geometry_msgs::msg::Pose & _pose)
{
  std::cout << "Position:" << std::endl;

  std::cout << "X: " << _pose.position.x << std::endl;
  std::cout << "Y: " << _pose.position.y << std::endl;
  std::cout << "Z: " << _pose.position.z << std::endl << std::endl;

  std::cout << "X: " << _pose.orientation.x << std::endl;
  std::cout << "Y: " << _pose.orientation.y << std::endl;
  std::cout << "Z: " << _pose.orientation.z << std::endl;
  std::cout << "W: " << _pose.orientation.w << std::endl << std::endl;

  std::cout << "Orientation:" << std::endl;
  // if (_euler) {
  //   double yaw, pitch, roll;
  //   tf2::Quaternion q_tmp{
  //     _pose.orientation.x,
  //     _pose.orientation.y,
  //     _pose.orientation.z,
  //     _pose.orientation.w};
  //   tf2::impl::getEulerYPR(q_tmp, yaw, pitch, roll);
  //   std::cout << "Yaw: " << yaw << std::endl;
  //   std::cout << "Pitch: " << pitch << std::endl;
  //   std::cout << "Roll: " << roll << std::endl << std::endl;
  // } else {
  //   std::cout << "X: " << _pose.orientation.x << std::endl;
  //   std::cout << "Y: " << _pose.orientation.y << std::endl;
  //   std::cout << "Z: " << _pose.orientation.z << std::endl;
  //   std::cout << "W: " << _pose.orientation.w << std::endl << std::endl;
  // }
}

// Print PoseStamped Message
void GraspScene::printPose(const geometry_msgs::msg::PoseStamped & _pose)
{
  std::cout << "Frame ID: " << _pose.header.frame_id << std::endl;
  printPose(_pose.pose);
}

void GraspScene::processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  pcl::PCLPointCloud2 * pcl_pc2(new pcl::PCLPointCloud2);
  PCLFunctions::SensorMsgtoPCLPointCloud2(*msg, *pcl_pc2);
  pcl::fromPCLPointCloud2(*pcl_pc2, *(this->cloud));
  PCLFunctions::passthroughFilter(
    this->cloud,
    this->ptFilter_Ulimit_x,
    this->ptFilter_Llimit_x,
    this->ptFilter_Ulimit_y,
    this->ptFilter_Llimit_y,
    this->ptFilter_Ulimit_z,
    this->ptFilter_Llimit_z);
  PCLFunctions::removeStatisticalOutlier(this->cloud, 1.0);
  PCLFunctions::voxelizeCloud<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
    pcl::VoxelGrid<pcl::PointXYZRGB>>(this->cloud, 0.005, this->org_cloud);
  PCLFunctions::planeSegmentation(
    this->cloud, this->cloud_plane_removed, this->cloud_table,
    this->segmentation_max_iterations,
    segmentation_distance_threshold);
}

void GraspScene::createWorldCollision(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  geometry_msgs::msg::TransformStamped sensorToWorldTf =
    this->buffer_->lookupTransform(
    "base_link", msg->header.frame_id,
    msg->header.stamp);
  octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation);
  this->world_collision_object = FCLFunctions::createCollisionObjectFromPointCloudRGB(
    this->org_cloud, sensor_origin, 0.01);
  // std::cout << "[planeSegmentation]" << std::endl;
  PCLFunctions::planeSegmentation(
    this->cloud, this->cloud_plane_removed, this->cloud_table,
    this->segmentation_max_iterations,
    segmentation_distance_threshold);
}
