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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDROBOT_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDROBOT_H_

#include <QDialog>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>

#include "attributes/robot.h"

namespace Ui
{
  class AddRobot; // NOLINT
}

class AddRobot: public QDialog
{
  Q_OBJECT

public:
  std::vector < std::vector < Robot >> available_robots;
  std::vector < std::string > available_brands;
  boost::filesystem::path original_path;
  Robot robot;
  bool success;
  bool editing_mode = false;

  int ErrorCheckOrigin();
  std::vector < Robot > LoadUR(std::vector < std::string > robot_list);
  Robot LoadRobot(std::string folder, std::string brand);
  std::vector < std::string > GetLinks(std::string filename);
  int LoadAvailableRobots();
  void LoadExistingRobot(Robot robot_input);
  explicit AddRobot(QWidget * parent = nullptr);
  ~AddRobot();

private slots:
  void on_robot_brand_currentIndexChanged(int index);

  void on_robot_model_currentIndexChanged(int index);

  void on_include_origin_stateChanged(int arg1);

  void on_exit_2_clicked();

  void on_ok_2_clicked();

  void keyPressEvent(QKeyEvent * e);

private:
  Ui::AddRobot * ui;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDROBOT_H_
