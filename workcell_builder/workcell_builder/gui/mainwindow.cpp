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

#include "gui/mainwindow.h"
#include <QFileDialog>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include "gui/ui_mainwindow.h"
#include "gui/scene_select.h"
#include "attributes/scene.h"


MainWindow::MainWindow(QWidget * parent)
: QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  ui->next->setDisabled(true);
  ui->change_workcell->setDisabled(true);
  success = false;
  ui->error_label->setWordWrap(true);
  ui->error_label->setText("<font color='red'>Workcell not available</font>");
  on_ros_version_currentIndexChanged(0);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_load_workcell_clicked()
{
  QString workcell_file = QFileDialog::getExistingDirectory(
    this,
    "Target workcell project destination",
    QDir::homePath());
  if (!workcell_file.isEmpty()) {
    boost::filesystem::current_path(workcell_file.toStdString());
    if (!boost::filesystem::exists("src")) {
      boost::filesystem::create_directory("src");
    }
    boost::filesystem::current_path("src");
    workcell_path = boost::filesystem::current_path();
    if (!boost::filesystem::exists("assets")) {
      boost::filesystem::create_directory("assets");
    }
    if (!boost::filesystem::exists("scenes")) {
      boost::filesystem::create_directory("scenes");
    }

    boost::filesystem::current_path("assets");
    if (!boost::filesystem::exists("robots")) {
      boost::filesystem::create_directory("robots");
    }
    if (!boost::filesystem::exists("end_effectors")) {
      boost::filesystem::create_directory("end_effectors");
    }
    if (!boost::filesystem::exists("environment")) {
      boost::filesystem::create_directory("environment");
    }

    boost::filesystem::current_path(workcell_path);
    boost::filesystem::current_path("scenes");

    workcell.scene_vector.clear();
    for (auto & filepath :
      boost::filesystem::directory_iterator(boost::filesystem::current_path()))
    {
      Scene temp_scene;
      temp_scene.filepath = filepath.path().string();

      std::string scene_name;
      for (auto it = filepath.path().string().crbegin(); it != filepath.path().string().crend();
        ++it)
      {
        if (*it != '/') {
          scene_name = std::string(1, *it) + scene_name;
        } else {
          break;
        }
      }
      if (is_good_scene(boost::filesystem::current_path(), scene_name)) {
        temp_scene.name = scene_name;
        temp_scene.loaded = false;
        workcell.scene_vector.push_back(temp_scene);
      }
    }
    ui->error_label->setText("<font color='green'>Workcell loaded</font>");
    ui->next->setDisabled(false);
    ui->load_workcell->setDisabled(true);
    ui->change_workcell->setDisabled(false);
    success = true;
    ui->filepath->setText(workcell_file);
    workcell.workcell_filepath = workcell_file.toStdString();
  }
}

void MainWindow::on_next_clicked()
{
  boost::filesystem::path before_scene_select(boost::filesystem::current_path());
  workcell.ros_ver = ui->ros_version->currentIndex() + 1;
  workcell.ros_distro = ui->ros_distro->currentText().toStdString();
  SceneSelect scene_window;
  scene_window.load_workcell(workcell);
  scene_window.setWindowTitle("Create New Environment");
  scene_window.setModal(true);
  scene_window.exec();
  boost::filesystem::current_path(before_scene_select);
}

void MainWindow::on_change_workcell_clicked()
{
  ui->next->setDisabled(true);
  ui->load_workcell->setDisabled(false);
  ui->change_workcell->setDisabled(true);
  ui->filepath->clear();
}

void MainWindow::on_ros_version_currentIndexChanged(int index)
{
  bool oldState = ui->ros_distro->blockSignals(true);
  ui->ros_distro->clear();
  for (int i = 0; i < static_cast<int>(ros_dist[index].size()); i++) {
    ui->ros_distro->addItem(QString::fromStdString(ros_dist[index][i]));
  }
  ui->ros_distro->blockSignals(oldState);
}
bool MainWindow::is_good_scene(boost::filesystem::path original_path, std::string scene_name)
{
  boost::filesystem::current_path(original_path);
  boost::filesystem::current_path(scene_name);

  if (!boost::filesystem::exists("urdf") || !boost::filesystem::exists("environment.yaml") ||
    !boost::filesystem::exists("CMakeLists.txt") || !boost::filesystem::exists("package.xml"))
  {
    boost::filesystem::current_path(original_path);
    return false;
  }
  boost::filesystem::current_path(original_path);
  return true;
}
