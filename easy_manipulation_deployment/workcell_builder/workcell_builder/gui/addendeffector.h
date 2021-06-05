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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDENDEFFECTOR_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDENDEFFECTOR_H_


#include <boost/filesystem.hpp>
#include <QDialog>
#include <robot.h>
#include <string>
#include <vector>

#include "attributes/end_effector.h"


namespace Ui
{
  class AddEndEffector; // NOLINT
}

class AddEndEffector: public QDialog
{
  Q_OBJECT

public:
  bool success;
  EndEffector ee;
  std::vector < std::string > available_brands;
  std::vector < std::vector < EndEffector >> available_ee;
  std::vector < std::string > supported_types {"finger", "suction"};
  std::vector < std::vector < std::vector < int >> > supported_attributes {{{2}}, {{1}, {1}}};
  boost::filesystem::path original_path;

  EndEffector LoadEE(std::string file, std::string brand);
  int LoadAvailableEE(Robot robot);
  void LoadExistingEE(EndEffector ee_input);
  std::vector < std::string > GetLinks(std::string filename);
  int ErrorCheckOrigin();

  explicit AddEndEffector(QWidget * parent = nullptr);
  ~AddEndEffector();

private slots:
  void on_ee_brand_currentIndexChanged(int index);

  void on_ee_model_currentIndexChanged(int index);

  void on_ee_type_currentIndexChanged(int index);

  void on_include_origin_stateChanged(int arg1);

  void on_ok_clicked();

  void on_exit_clicked();

  void keyPressEvent(QKeyEvent * e);

private:
  Ui::AddEndEffector * ui;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDENDEFFECTOR_H_
