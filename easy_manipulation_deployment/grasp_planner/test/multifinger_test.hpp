// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//`
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef MULTIFINGER_TEST_HPP_
#define MULTIFINGER_TEST_HPP_

#include <gtest/gtest.h>
#include "grasp_planner/end_effectors/finger_gripper.hpp"
#include "grasp_planner/common/pcl_functions.hpp"
#include "grasp_planner/grasp_object.hpp"

class MultiFingerTest : public ::testing::Test
{
public:
  std::shared_ptr<GraspObject> object;
  std::string id;
  int num_fingers_side_1;
  int num_fingers_side_2;
  float distance_between_fingers_1;
  float distance_between_fingers_2;
  float finger_thickness;
  float gripper_stroke;
  float voxel_size;
  float grasp_quality_weight1;
  float grasp_quality_weight2;
  float grasp_plane_dist_limit;
  float cloud_normal_radius;
  float worldXAngleThreshold;
  float worldYAngleThreshold;
  float worldZAngleThreshold;

  std::shared_ptr<FingerGripper> gripper;

  //pcl::visualization::PCLVisualizer::Ptr viewer;
  
  MultiFingerTest();
  void ResetVariables();
  void LoadGripper();
  


  void SetUp(void)
  {
    std::cout << "Setup" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rectangle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    float length = 0.05;
    float breadth = 0.01;
    float height = 0.02;

    for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
      for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
        for (float height_ = 0.0; height_ < height; height_ += 0.0025) {
          pcl::PointXYZRGB temp_point(length_, breadth_, height_);
          rectangle_cloud->points.push_back(temp_point);
        }
      }
    }
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*rectangle_cloud, centroid);
    GraspObject object_("camera_frame", rectangle_cloud, centroid);
    object = std::make_shared<GraspObject>(object_);
    PCLFunctions::computeCloudNormal(object->cloud, object->cloud_normal, 0.03);
    object->get_object_bb();
  }
  void TearDown(void)
  {
    std::cout << "Teardown" << std::endl;
  }
};

#endif  // MULTIFINGER_TEST_HPP_