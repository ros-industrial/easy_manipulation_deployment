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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDSCENE_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDSCENE_H_

#include <QDialog>
#include <QListWidgetItem>
#include <boost/filesystem.hpp>

#include "attributes/scene.h"


namespace Ui
{
  class AddScene; // NOLINT
}

class AddScene: public QDialog
{
  Q_OBJECT

public:
  Scene scene;
  Link world_link;
  bool success;
  // boost::filesystem::path original_path;
  boost::filesystem::path assets_path;
  boost::filesystem::path scenes_path;
  boost::filesystem::path workcell_path;

  void LoadScene(Scene scene_input);
  void LoadRobot(Robot robot);
  void LoadEE(EndEffector ee);
  void LoadObject(Object object);
  bool CheckRobot();
  bool CheckEE();
  bool CheckExtJoint();
  bool CheckSceneName();

  explicit AddScene(QWidget * parent = nullptr);
  ~AddScene();

private slots:
  void on_add_object_clicked();

  void on_object_list_itemDoubleClicked(QListWidgetItem * item);

  void on_include_robot_stateChanged(int arg1);

  void on_include_ee_stateChanged(int arg1);

  void on_object_list_2_itemDoubleClicked(QListWidgetItem * item);

  void on_delete_object_clicked();

  void on_add_robot_clicked();

  void on_add_ee_clicked();

  void on_remove_robot_clicked();

  void on_remove_ee_clicked();

  void on_ok_clicked();

  void on_exit_clicked();

  void keyPressEvent(QKeyEvent * e);

  void on_load_object_clicked();

private:
  Ui::AddScene * ui;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDSCENE_H_
