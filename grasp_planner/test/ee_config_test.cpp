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
#include <string>
#include "ee_config.hpp"

TEST(GetPackagePathTest, HasPackagePath)
{
  EXPECT_TRUE(
    get_package_filepath("grasp_planning").find(
      "install/grasp_planning/share/grasp_planning") != std::string::npos);
}

TEST(GetPackagePathTest, NonExistPackagePath)
{
  ASSERT_THROW(get_package_filepath("wrong_pkg"), std::exception);
}

TEST(GoToPathTest, HasFolderPath)
{
  ASSERT_NO_THROW(goto_folder_filepath("config", "grasp_planning"));
  ASSERT_NO_THROW(goto_folder_filepath("test", "grasp_planning"));
}

TEST(GoToPathTest, NoFolderPath)
{
  ASSERT_THROW(
    goto_folder_filepath("no_folder", "grasp_planning"),
    boost::filesystem::filesystem_error);
}

TEST(LoadYamlTest, HasYamlFile)
{
  ASSERT_NO_THROW(load_config_yaml("attributes.yaml", "test/test_yaml", "grasp_planning"));
}

TEST(LoadYamlTest, NoYamlFile)
{
  ASSERT_THROW(
    load_config_yaml("doesnt_exist.yaml", "test/test_yaml", "grasp_planning"),
    YAML::BadFile);
}
TEST(LoadYamlTest, BadYamlFile)
{
  ASSERT_THROW(
    load_config_yaml("bad_yaml.yaml", "test/test_yaml", "grasp_planning"),
    std::exception);
}
TEST(LoadYamlTest, BadFolder)
{
  ASSERT_THROW(
    load_config_yaml("finger.yaml", "doesnt_exist_folder", "grasp_planning"),
    boost::filesystem::filesystem_error);
}
TEST(GetEETypeTest, GetSuction)
{
  ASSERT_EQ(get_ee_type("suction.yaml", "test/test_yaml", "grasp_planning").compare("suction"), 0);
}
TEST(GetEETypeTest, GetFinger)
{
  ASSERT_EQ(get_ee_type("finger.yaml", "test/test_yaml", "grasp_planning").compare("finger"), 0);
}
TEST(GetEETypeTest, NoFields)
{
  ASSERT_ANY_THROW(get_ee_type("no_ee.yaml", "test/test_yaml", "grasp_planning"));
}

TEST(GetEETypeTest, WrongFolder)
{
  ASSERT_THROW(
    get_ee_type("no_ee.yaml", "wrong_folder", "grasp_planning"),
    boost::filesystem::filesystem_error);
}

TEST(GetEETypeTest, WrongYAML)
{
  ASSERT_THROW(
    get_ee_type("doesnt_exist.yaml", "test/test_yaml", "grasp_planning"),
    YAML::BadFile);
}

TEST(LoadFingerTest, HasFinger)
{
  try {
    TwoFinger result = load_finger_attributes(
      "finger.yaml",
      "test/test_yaml/finger",
      "grasp_planning");
    EXPECT_EQ(570, static_cast<int>(result.table_height));
    EXPECT_EQ(25, static_cast<int>(result.gripper_thickness));
    EXPECT_EQ(240, static_cast<int>(result.distance_between_fingers));
    EXPECT_NEAR(0.01, result.min_zero_angle, 0.001);
    EXPECT_EQ(5, static_cast<int>(result.min_height_diff_to_grip));
    EXPECT_EQ(5, static_cast<int>(result.min_gdi_diff_for_comparison));
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }
}
TEST(LoadFingerTest, NoExistYaml)
{
  ASSERT_THROW(
    load_finger_attributes(
      "finger_noexist.yaml",
      "test/test_yaml/finger",
      "grasp_planning"),
    YAML::BadFile);
}

TEST(LoadFingerTest, NoExistFolder)
{
  ASSERT_THROW(
    load_finger_attributes(
      "finger.yaml",
      "nonexistent_folder",
      "grasp_planning"),
    boost::filesystem::filesystem_error);
}

TEST(LoadSuctionTest, BadYaml)
{
  ASSERT_THROW(
    load_finger_attributes(
      "bad_yaml.yaml",
      "test/test_yaml",
      "grasp_planning"),
    std::exception);
}

TEST(LoadFingerTest, WrongFingerNum)
{
  try {
    load_finger_attributes(
      "1_finger.yaml",
      "test/test_yaml/finger/wrong_finger",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "Only 2 Finger Grippers supported for current implementation");
  }

  try {
    load_finger_attributes(
      "3_finger.yaml",
      "test/test_yaml/finger/wrong_finger",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "Only 2 Finger Grippers supported for current implementation");
  }
}

TEST(LoadFingerTest, NoFields)
{
  try {
    load_finger_attributes(
      "no_attributes.yaml",
      "test/test_yaml/finger/no_fields",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "No attribute field available in YAML File");
  }

  try {
    load_finger_attributes(
      "no_dist.yaml",
      "test/test_yaml/finger/no_fields",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "yaml file does not contain the distance_between_fingers parameter");
  }

  try {
    load_finger_attributes(
      "no_fingers.yaml",
      "test/test_yaml/finger/no_fields",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "yaml file does not contain the fingers parameter");
  }

  try {
    load_finger_attributes(
      "no_tableh.yaml",
      "test/test_yaml/finger/no_fields",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "yaml file does not contain the table_height parameter");
  }
  try {
    load_finger_attributes(
      "no_longestdim.yaml",
      "test/test_yaml/finger/no_fields",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "yaml file does not contain the longest_gripper_dim parameter");
  }
}

TEST(LoadFingerTest, NegativeVals)
{
  try {
    load_finger_attributes(
      "neg_dist.yaml",
      "test/test_yaml/finger/neg_vals",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "Error: Negative value for distance_between_fingers");
  }

  try {
    load_finger_attributes(
      "neg_longestdim.yaml",
      "test/test_yaml/finger/neg_vals",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "Error: Negative value for longest_gripper_dim");
  }

  try {
    load_finger_attributes(
      "neg_tableh.yaml",
      "test/test_yaml/finger/neg_vals",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "Error: Negative value for table_height");
  }
}

TEST(LoadSuctionTest, HasSuction)
{
  SuctionCupArray result = load_suction_attributes(
    "suction.yaml",
    "test/test_yaml/suction",
    "grasp_planning");

  EXPECT_EQ(1, result.length_cup_num);
  EXPECT_EQ(1, result.breadth_cup_num);
  EXPECT_EQ(19, result.radius);
  EXPECT_EQ(560, result.table_height);
}

TEST(LoadSuctionTest, NoFields)
{
  try {
    load_suction_attributes(
      "no_attributes.yaml",
      "test/test_yaml/suction/no_fields",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "No attribute field available in YAML File");
  }

  try {
    load_suction_attributes(
      "no_breadth.yaml",
      "test/test_yaml/suction/no_fields",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "[ERROR] yaml file does not contain the breadth_cup_num parameter");
  }

  try {
    load_suction_attributes(
      "no_length.yaml",
      "test/test_yaml/suction/no_fields",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "[ERROR] yaml file does not contain the length_cup_num parameter");
  }

  try {
    load_suction_attributes(
      "no_tableh.yaml",
      "test/test_yaml/suction/no_fields",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "[ERROR] yaml file does not contain the table_height parameter");
  }

  try {
    load_suction_attributes(
      "no_radius.yaml",
      "test/test_yaml/suction/no_fields",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "[ERROR] yaml file does not contain the radius parameter");
  }
}

TEST(LoadSuctionTest, WrongCups)
{
  try {
    load_suction_attributes(
      "1x0_cups.yaml",
      "test/test_yaml/suction/wrong_cups",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "[ERROR] Breadth of suction array must have at least 1 cup.");
  }

  try {
    load_suction_attributes(
      "0x1_cups.yaml",
      "test/test_yaml/suction/wrong_cups",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "[ERROR] Length of suction array must have at least 1 cup.");
  }

  try {
    load_suction_attributes(
      "2x3_cups.yaml",
      "test/test_yaml/suction/wrong_cups",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "[ERROR] Current planner only supports 1x1 cup array");
  }

  try {
    load_suction_attributes(
      "5x2_cups.yaml",
      "test/test_yaml/suction/wrong_cups",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "[ERROR] Current planner only supports 1x1 cup array");
  }
}

TEST(LoadSuctionTest, NegVals)
{
  try {
    load_suction_attributes(
      "neg_breadth.yaml",
      "test/test_yaml/suction/neg_vals",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "[ERROR] Breadth of suction array must have at least 1 cup.");
  }

  try {
    load_suction_attributes(
      "neg_height.yaml",
      "test/test_yaml/suction/neg_vals",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "[ERROR] table height needs to be a positive number ");
  }

  try {
    load_suction_attributes(
      "neg_length.yaml",
      "test/test_yaml/suction/neg_vals",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "[ERROR] Length of suction array must have at least 1 cup.");
  }
  try {
    load_suction_attributes(
      "neg_radius.yaml",
      "test/test_yaml/suction/neg_vals",
      "grasp_planning");
  } catch (const char * err) {
    EXPECT_STREQ(err, "[ERROR] radius needs to be a positive number ");
  }
}
TEST(LoadSuctionTest, WrongFolder)
{
  ASSERT_THROW(
    load_suction_attributes(
      "suction.yaml",
      "doesnt_exist_folder",
      "grasp_planning"),
    boost::filesystem::filesystem_error);
}
TEST(LoadSuctionTest, WrongFile)
{
  ASSERT_THROW(
    load_suction_attributes(
      "doesnt_exist.yaml",
      "test/test_yaml/suction",
      "grasp_planning"),
    YAML::BadFile);
}

TEST(LoadSuctionTest, BadFile)
{
  ASSERT_THROW(
    load_suction_attributes(
      "bad_yaml.yaml",
      "test/test_yaml",
      "grasp_planning"),
    std::exception);
}
