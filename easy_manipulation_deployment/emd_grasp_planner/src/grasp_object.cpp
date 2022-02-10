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
#include "emd/grasp_planner/grasp_object.hpp"

GraspObject::GraspObject(
  std::string object_frame_,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_,
  Eigen::Vector4f centerpoint_)
: object_name("unknown_object"),
  object_frame(object_frame_),
  cloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
  cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>()),
  cloud_normal(new pcl::PointCloud<pcl::PointNormal>()),
  centerpoint(centerpoint_),
  max_grasp_samples(1)
{
  cloud = cloud_;
  this->grasp_target.target_type = "unknown_object";
  // centerpoint = centerpoint_;
  // max_grasp_samples = 1;
  // object_name = "unknown_object";
}

GraspObject::GraspObject(
  std::string object_name_, std::string object_frame_,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_,
  Eigen::Vector4f centerpoint_)
: object_name(object_name_),
  object_frame(object_frame_),
  cloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
  cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>()),
  cloud_normal(new pcl::PointCloud<pcl::PointNormal>()),
  centerpoint(centerpoint_),
  max_grasp_samples(1)
{
  cloud = cloud_;
  grasp_target.target_type = object_name_;
  // centerpoint = centerpoint_;
  // max_grasp_samples = 1;
  // object_name = object_name_;
}

void GraspObject::get_object_bb()
{
  pcl::compute3DCentroid(*(this->cloud), this->centerpoint);
  pcl::computeCovarianceMatrix(*(this->cloud), this->centerpoint, this->covariance_matrix);
  pcl::eigen33(this->covariance_matrix, this->eigenvectors, this->eigenvalues);

  this->axis = Eigen::Vector3f(
    this->eigenvectors.col(2)(0),
    this->eigenvectors.col(2)(1), this->eigenvectors.col(2)(2));

  this->minor_axis = Eigen::Vector3f(
    this->eigenvectors.col(0)(0),
    this->eigenvectors.col(0)(1), this->eigenvectors.col(0)(2));

  this->grasp_axis = Eigen::Vector3f(
    this->eigenvectors.col(1)(0),
    this->eigenvectors.col(1)(1), this->eigenvectors.col(1)(2));

  // http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html?m=1
  // Transform the original cloud to the origin where
  // the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3, 3>(0, 0) = this->eigenvectors.transpose();
  projectionTransform.block<3, 1>(
    0,
    3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * this->centerpoint.head<3>());

  pcl::transformPointCloud(*(this->cloud), *(this->cloud_projected), projectionTransform);
  this->affine_matrix = projectionTransform;
  pcl::getMinMax3D(*(this->cloud_projected), this->minPoint, this->maxPoint);

  const Eigen::Vector3f meanDiagonal = 0.5f *
    (this->maxPoint.getVector3fMap() + this->minPoint.getVector3fMap());
  Eigen::Quaternionf bboxQuaternion(this->eigenvectors);
  this->bboxQuaternion = bboxQuaternion;
  this->bboxTransform = this->eigenvectors * meanDiagonal + this->centerpoint.head<3>();
  get_object_dimensions();
  get_axis_alignments();
}

void GraspObject::get_object_dimensions()
{
  // this->dimensions[0] = abs(this->maxPoint.x - this->minPoint.x);
  // this->dimensions[1] = abs(this->maxPoint.y - this->minPoint.y);
  // this->dimensions[2] = abs(this->maxPoint.z - this->minPoint.z);

  this->dimensions[0] = this->maxPoint.x - this->minPoint.x;
  this->dimensions[1] = this->maxPoint.y - this->minPoint.y;
  this->dimensions[2] = this->maxPoint.z - this->minPoint.z;
}

void GraspObject::get_object_world_angles()
{
  Eigen::Vector3f worldXVector = Eigen::Vector3f(1, 0, 0);
  Eigen::Vector3f worldYVector = Eigen::Vector3f(0, 1, 0);
  Eigen::Vector3f worldZVector = Eigen::Vector3f(0, 0, 1);

  this->objectWorldCosX = std::abs(
    (this->axis.dot(worldXVector)) /
    (this->axis.norm() * worldXVector.norm()));
  this->objectWorldCosY = std::abs(
    (this->axis.dot(worldYVector)) /
    (this->axis.norm() * worldYVector.norm()));
  this->objectWorldCosZ = std::abs(
    (this->axis.dot(worldZVector)) /
    (this->axis.norm() * worldZVector.norm()));
}

geometry_msgs::msg::PoseStamped GraspObject::get_object_pose(std::string pose_frame)
{
  geometry_msgs::msg::PoseStamped result_pose;

  result_pose.pose.position.x = this->centerpoint(0);
  result_pose.pose.position.y = this->centerpoint(1);
  result_pose.pose.position.z = this->centerpoint(2);

  Eigen::Matrix3f midPointRotation;
  midPointRotation << this->affine_matrix(0, 0), this->affine_matrix(0, 1),
    this->affine_matrix(0, 2),
    this->affine_matrix(1, 0), this->affine_matrix(1, 1), this->affine_matrix(1, 2),
    this->affine_matrix(2, 0), this->affine_matrix(2, 1), this->affine_matrix(2, 2);
  Eigen::Quaternionf quaternion_(midPointRotation.transpose());
  result_pose.pose.orientation.x = quaternion_.x();
  result_pose.pose.orientation.y = quaternion_.y();
  result_pose.pose.orientation.z = quaternion_.z();
  result_pose.pose.orientation.w = quaternion_.w();

  const auto clock = std::chrono::system_clock::now();
  result_pose.header.frame_id = pose_frame;
  result_pose.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(
    clock.time_since_epoch()).count();
  /* result_pose.header.stamp.nsec =
  std::chrono::duration_cast<std::chrono::nanoseconds>(clock.time_since_epoch()).count();
  */
  return result_pose;
}

shape_msgs::msg::SolidPrimitive GraspObject::get_object_shape()
{  // This function is highly dependant on what the camera axis is.
  shape_msgs::msg::SolidPrimitive object_shape;
  object_shape.type = object_shape.BOX;
  object_shape.dimensions.resize(3);
  object_shape.dimensions[0] = this->dimensions[0];
  object_shape.dimensions[1] = this->dimensions[1];
  object_shape.dimensions[2] = this->dimensions[2];
  return object_shape;
}

void GraspObject::get_axis_alignments()
{
  alignments[0] = get_axis(axis);
  alignments[1] = get_axis(grasp_axis);
  alignments[2] = get_axis(minor_axis);
}

char GraspObject::get_axis(Eigen::Vector3f vector)
{
  Eigen::Vector3f worldXVector = Eigen::Vector3f(1, 0, 0);
  Eigen::Vector3f worldYVector = Eigen::Vector3f(0, 1, 0);
  Eigen::Vector3f worldZVector = Eigen::Vector3f(0, 0, 1);

  float vec_x = std::abs(vector.dot(worldXVector));
  float vec_y = std::abs(vector.dot(worldYVector));
  float vec_z = std::abs(vector.dot(worldZVector));

  float max_temp = std::max(vec_x, vec_y);
  float max_ = std::max(max_temp, vec_z);
  if (vec_x == max_) {
    return 'x';
  } else if (vec_y == max_) {
    return 'y';
  } else if (vec_z == max_) {
    return 'z';
  }
  return '-';
}
