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

#include <gtest/gtest.h>
#include "pcl_functions_test.hpp"

PCLFunctionsTest::PCLFunctionsTest()
: rectangle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
}

void PCLFunctionsTest::GenerateCloud(float length, float breadth, float height)
{
  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.0; height_ < height; height_ += 0.0025) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = length_;
        temp_point.y = breadth_;
        temp_point.z = height_;
        rectangle_cloud->points.push_back(temp_point);
      }
    }
  }
}

TEST_F(PCLFunctionsTest, NormalizeTest)
{
  float result = MathFunctions::normalize(10.0, 0.0, 100.0);
  EXPECT_NEAR(result, 0.1, 0.00001);
}

TEST_F(PCLFunctionsTest, PCLToEigenTest)
{
  pcl::PointNormal point_normal;
  point_normal.x = 0.001;
  point_normal.y = 0.002;
  point_normal.z = 0.003;

  Eigen::Vector3f point_normal_converted =
    PCLFunctions::convert_pcl_to_eigen(point_normal);

  EXPECT_NEAR(point_normal_converted(0), 0.001, 0.00001);
  EXPECT_NEAR(point_normal_converted(1), 0.002, 0.00001);
  EXPECT_NEAR(point_normal_converted(2), 0.003, 0.00001);

  pcl::PointXYZ point_xyz;
  point_xyz.x = 0.004;
  point_xyz.y = 0.005;
  point_xyz.z = 0.006;

  Eigen::Vector3f point_xyz_converted =
    PCLFunctions::convert_pcl_to_eigen(point_xyz);

  EXPECT_NEAR(point_xyz_converted(0), 0.004, 0.00001);
  EXPECT_NEAR(point_xyz_converted(1), 0.005, 0.00001);
  EXPECT_NEAR(point_xyz_converted(2), 0.006, 0.00001);

  pcl::PointXYZRGB point_xyz_rgb;
  point_xyz_rgb.x = 0.007;
  point_xyz_rgb.y = 0.008;
  point_xyz_rgb.z = 0.009;

  Eigen::Vector3f point_xyz_rgb_converted =
    PCLFunctions::convert_pcl_to_eigen(point_xyz_rgb);

  EXPECT_NEAR(point_xyz_rgb_converted(0), 0.007, 0.00001);
  EXPECT_NEAR(point_xyz_rgb_converted(1), 0.008, 0.00001);
  EXPECT_NEAR(point_xyz_rgb_converted(2), 0.009, 0.00001);
}


TEST_F(PCLFunctionsTest, PCLNormalToEigenTest)
{
  pcl::PointNormal point_normal;
  point_normal.normal_x = 0.011;
  point_normal.normal_y = 0.022;
  point_normal.normal_z = 0.033;

  Eigen::Vector3f point_normal_converted =
    PCLFunctions::convert_pcl_normal_to_eigen(point_normal);

  EXPECT_NEAR(point_normal_converted(0), 0.011, 0.00001);
  EXPECT_NEAR(point_normal_converted(1), 0.022, 0.00001);
  EXPECT_NEAR(point_normal_converted(2), 0.033, 0.00001);
}

TEST_F(PCLFunctionsTest, passThroughTest)
{
  GenerateCloud(0.05, 0.01, 0.02);

  float ptFilter_Ulimit_x = 0.04;
  float ptFilter_Llimit_x = 0.01;
  float ptFilter_Ulimit_y = 0.008;
  float ptFilter_Llimit_y = 0.001;
  float ptFilter_Ulimit_z = 0.018;
  float ptFilter_Llimit_z = 0.01;

  EXPECT_TRUE(
    PCLFunctions::passthrough_filter(
      rectangle_cloud,
      ptFilter_Ulimit_x,
      ptFilter_Llimit_x,
      ptFilter_Ulimit_y,
      ptFilter_Llimit_y,
      ptFilter_Ulimit_z,
      ptFilter_Llimit_z));

  for (auto point : rectangle_cloud->points) {

    EXPECT_GE(point.x, ptFilter_Llimit_x);
    EXPECT_LE(point.x, ptFilter_Ulimit_x);
    EXPECT_GE(point.y, ptFilter_Llimit_y);
    EXPECT_LE(point.y, ptFilter_Ulimit_y);
    EXPECT_GE(point.z, ptFilter_Llimit_z);
    EXPECT_LE(point.z, ptFilter_Ulimit_z);
  }
}

TEST_F(PCLFunctionsTest, planeSegmentationTest)
{

  float length = 0.05;
  float breadth = 0.01;
  float height = 0.03;

  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.01; height_ < height; height_ += 0.0025) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = length_;
        temp_point.y = breadth_;
        temp_point.z = height_;
        rectangle_cloud->points.push_back(temp_point);
      }
    }
  }

  for (float length_ = 0.0; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.0; breadth_ < breadth; breadth_ += 0.0025) {
      pcl::PointXYZRGB temp_point;
      temp_point.x = length_;
      temp_point.y = breadth_;
      temp_point.z = 0;
      rectangle_cloud->points.push_back(temp_point);
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_removed(
    new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_table(
    new pcl::PointCloud<pcl::PointXYZRGB>());

  EXPECT_EQ(0, static_cast<int>(cloud_plane_removed->points.size()));
  EXPECT_EQ(0, static_cast<int>(cloud_table->points.size()));

  EXPECT_TRUE(
    PCLFunctions::plane_segmentation(
      rectangle_cloud, cloud_plane_removed, cloud_table, 50, 0.001));
  EXPECT_GT(static_cast<int>(cloud_plane_removed->points.size()), 0);
  EXPECT_GT(static_cast<int>(cloud_table->points.size()), 0);
  EXPECT_GT(rectangle_cloud->points.size(), cloud_plane_removed->points.size());
  EXPECT_GT(rectangle_cloud->points.size(), cloud_table->points.size());
  EXPECT_GT(cloud_plane_removed->points.size(), cloud_table->points.size());

}

TEST_F(PCLFunctionsTest, outliersTest)
{
  GenerateCloud(0.05, 0.01, 0.02);

  pcl::PointXYZRGB outlier_1;
  outlier_1.x = 0.09;
  outlier_1.y = 0.09;
  outlier_1.z = 0.09;

  pcl::PointXYZRGB outlier_2;
  outlier_2.x = 0.09;
  outlier_2.y = 0.02;
  outlier_2.z = 0.02;

  pcl::PointXYZRGB outlier_3;
  outlier_3.x = 0.07;
  outlier_3.y = 0.06;
  outlier_3.z = 0.09;

  rectangle_cloud->points.push_back(outlier_1);
  rectangle_cloud->points.push_back(outlier_2);
  rectangle_cloud->points.push_back(outlier_3);
  int initial = static_cast<int>(rectangle_cloud->points.size());

  PCLFunctions::remove_statistical_outlier(
    rectangle_cloud,
    0.5);
  int final = static_cast<int>(rectangle_cloud->points.size());

  EXPECT_EQ(3, initial - final);
}

TEST_F(PCLFunctionsTest, getClosestPointsByRadiusTestNone)
{
  GenerateCloud(0.05, 0.01, 0.02);
  pcl::PointNormal point;
  point.x = 0.09;
  point.y = 0.09;
  point.z = 0.09;
  float radius = 0.01;

  pcl::PointCloud<pcl::PointNormal>::Ptr rectNormalCloud(
    new pcl::PointCloud<pcl::PointNormal>());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(
    new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PointCloud<pcl::PointNormal>::Ptr outputNormalCloud(
    new pcl::PointCloud<pcl::PointNormal>());

  PCLFunctions::compute_cloud_normal(
    rectangle_cloud, rectNormalCloud,
    0.03);

  PCLFunctions::get_closest_points_by_radius(
    point,
    radius,
    rectangle_cloud,
    rectNormalCloud,
    outputCloud,
    outputNormalCloud);

  EXPECT_EQ(0, static_cast<int>(outputCloud->points.size()));
  EXPECT_EQ(0, static_cast<int>(outputNormalCloud->points.size()));
}

TEST_F(PCLFunctionsTest, getClosestPointsByRadiusTest)
{
  GenerateCloud(0.05, 0.01, 0.02);
  pcl::PointNormal point;
  point.x = 0.01;
  point.y = 0.01;
  point.z = 0.01;
  float radius = 0.01;

  pcl::PointCloud<pcl::PointNormal>::Ptr rectNormalCloud(
    new pcl::PointCloud<pcl::PointNormal>());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(
    new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PointCloud<pcl::PointNormal>::Ptr outputNormalCloud(
    new pcl::PointCloud<pcl::PointNormal>());

  PCLFunctions::compute_cloud_normal(
    rectangle_cloud, rectNormalCloud,
    0.03);

  PCLFunctions::get_closest_points_by_radius(
    point,
    radius,
    rectangle_cloud,
    rectNormalCloud,
    outputCloud,
    outputNormalCloud);

  EXPECT_GT(static_cast<int>(outputCloud->points.size()), 0);
  EXPECT_GT(static_cast<int>(outputNormalCloud->points.size()), 0);
}

TEST_F(PCLFunctionsTest, computeCloudNormalTest)
{
  GenerateCloud(0.05, 0.01, 0.02);
  pcl::PointCloud<pcl::PointNormal>::Ptr rectNormalCloud(
    new pcl::PointCloud<pcl::PointNormal>());

  PCLFunctions::compute_cloud_normal(
    rectangle_cloud, rectNormalCloud,
    0.03);
  EXPECT_GT(static_cast<int>(rectNormalCloud->points.size()), 0);
}

TEST_F(PCLFunctionsTest, extractPointCloudClustersTest)
{
  GenerateCloud(0.05, 0.01, 0.02);

  float length = 0.09;
  float breadth = 0.08;
  float height = 0.07;

  for (float length_ = 0.07; length_ < length; length_ += 0.0025) {
    for (float breadth_ = 0.04; breadth_ < breadth; breadth_ += 0.0025) {
      for (float height_ = 0.04; height_ < height; height_ += 0.0025) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = length_;
        temp_point.y = breadth_;
        temp_point.z = height_;
        rectangle_cloud->points.push_back(temp_point);
      }
    }
  }
  std::vector<pcl::PointIndices> indices =
    PCLFunctions::extract_pointcloud_clusters(rectangle_cloud, 0.005, 50);
  EXPECT_FALSE(indices.empty());

  EXPECT_EQ(2, static_cast<int>(indices.size()));
}

TEST_F(PCLFunctionsTest, extractPointCloudClustersTestEmpty)
{
  std::vector<pcl::PointIndices> indices =
    PCLFunctions::extract_pointcloud_clusters(rectangle_cloud, 0.005, 50);
  EXPECT_TRUE(indices.empty());
}
