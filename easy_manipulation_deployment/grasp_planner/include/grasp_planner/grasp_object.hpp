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

#ifndef GRASP_OBJECT_HPP_
#define GRASP_OBJECT_HPP_

// Main PCL files
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>

// For Plane Segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// For Object Segmentation
#include <pcl/segmentation/extract_clusters.h>

// For Cloud Filtering
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

// For Visualization
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

// ROS2 Libraries
#include <sensor_msgs/msg/point_cloud2.hpp>

// Other Libraries
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "shape_msgs/msg/solid_primitive.hpp"

// Custom Msg
#include <emd_msgs/msg/grasp_method.hpp>  // NOLINT
#include <emd_msgs/msg/grasp_target.hpp>  // NOLINT

// EMD Libraries
#include "grasp_planner/common/pcl_functions.hpp"


struct GraspContacts
{
  pcl::PointXYZ point_1;
  pcl::PointXYZ point_2;
};

class GraspObject
{
public:
  /*! \brief Message output to describe grasp decisions for this object */
  emd_msgs::msg::GraspTarget grasp_target;
  /*! \brief Object Name */
  std::string object_name;
  /*! \brief Tf frame the object pose is with respect to */
  std::string object_frame;
  /*! \brief Object point cloud */
  pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud;
  /*! \brief Projected Objected Point Cloud */
  pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud_projected;
  /*! \brief Grasp Object Normal Point cloud */
  pcl::PointCloud < pcl::PointNormal > ::Ptr cloud_normal;
  /*! \brief Vector representing the major axis of the Grasp Object*/
  Eigen::Vector3f axis;  // Obj coeff 3,4,5
  /*! \brief Computed 3D centroid of the Grasp Object point cloud*/
  Eigen::Vector4f centerpoint;  // Obj coeff 1,2,3
  /*! \brief Eigenvalues of the Object point cloud from the covariance matrix */
  Eigen::Vector3f eigenvalues;
  /*! \brief Eigenvectors of the Object point cloud from the covariance matrix */
  Eigen::Matrix3f eigenvectors;
  /*! \brief Covariance Matrix calculated using Principal Component Analysis of the Object point cloud */
  Eigen::Matrix3f covariance_matrix;
  /*! \brief Quaternion of the 3D bounding box */
  Eigen::Quaternionf bboxQuaternion;
  /*! \brief Transform of the 3D bounding box */
  Eigen::Vector3f bboxTransform;
  /*! \brief Calculated Minimum 3D point of the object point cloud */
  pcl::PointXYZRGB minPoint;
  /*! \brief Calculated Maximum 3D point of the object point cloud */
  pcl::PointXYZRGB maxPoint;
  /*! \brief Cos angle of the object with respect to the world X axis*/
  float objectWorldCosX;
  /*! \brief Cos angle of the object with respect to the world Y axis*/
  float objectWorldCosY;
  /*! \brief Cos angle of the object with respect to the world Z axis*/
  float objectWorldCosZ;
  /*! \brief */
  Eigen::Matrix4f affine_matrix;
  /*! \brief */
  int max_grasp_samples;  // Maximum number of grasp options

  // // To be removed, temp variable
  // pcl::ModelCoefficients::Ptr cutting_plane;


  GraspObject(
    std::string object_frame_, pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud_,
    Eigen::Vector4f centerpoint_);
  GraspObject(
    std::string object_name_, std::string object_frame_,
    pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud_, Eigen::Vector4f centerpoint_);
  void get_object_bb();
  void get_object_world_angles();
  // void add_cutting_plane_viewer(int pos, pcl::visualization::PCLVisualizer::Ptr viewer);
  void add_bb_viewer(int pos, pcl::visualization::PCLVisualizer::Ptr viewer);
  shape_msgs::msg::SolidPrimitive getObjectShape();
  geometry_msgs::msg::PoseStamped getObjectPose(std::string pose_frame);
};

#endif  // GRASP_OBJECT_HPP_
