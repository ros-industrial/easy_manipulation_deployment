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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__MAINWINDOW_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__MAINWINDOW_H_

#include <QMainWindow>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>

#include "attributes/workcell.h"

QT_BEGIN_NAMESPACE
namespace Ui {class MainWindow;}
QT_END_NAMESPACE

class MainWindow: public QMainWindow
{
  Q_OBJECT

public:
  friend class TestMain;
  boost::filesystem::path workcell_path;
  Workcell workcell;
  bool success;
  std::vector < std::vector < std::string >> ros_dist {{"melodic"}, {"eloquent", "foxy"}};
  bool is_good_scene(boost::filesystem::path original_path, std::string scene_name);

  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow();

private slots:
  void on_load_workcell_clicked();

  void on_next_clicked();

  void on_change_workcell_clicked();

  void on_ros_version_currentIndexChanged(int index);

private:
  Ui::MainWindow * ui;
};
#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__MAINWINDOW_H_
