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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDLINK_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDLINK_H_

#include <QDialog>
#include <QListWidgetItem>
#include <string>
#include <vector>
#include "boost/filesystem.hpp"


#include "attributes/link.h"
#include "gui/addcollision.h"
#include "gui/addinertial.h"
#include "gui/addvisual.h"
#include "attributes/visual.h"

namespace Ui
{
  class AddLink; // NOLINT
}

class AddLink: public QDialog
{
  Q_OBJECT

public:
  // std::vector<VisualLink> visual_list;

  Link link;
  bool success;
  bool editing_mode;
  std::vector < std::string > available_link_names;
  boost::filesystem::path workcell_path;

  void load_link(Link link);
  bool ErrorLinkName();
  bool ErrorToggle();

  explicit AddLink(QWidget * parent = nullptr);
  ~AddLink();

private slots:
  void on_enable_visual_stateChanged(int arg1);

  void on_enable_collision_stateChanged(int arg1);

  void on_enable_inertial_stateChanged(int arg1);

  void on_create_visual_clicked();

  void on_create_collision_clicked();

  void on_edit_visual_clicked();

  void on_edit_collision_clicked();

  void on_delete_visual_clicked();

  void on_delete_collision_clicked();

  void on_create_inertial_clicked();

  void on_lineEdit_textChanged(const QString & arg1);

  void on_ok_clicked();

  void on_close_clicked();

  void on_edit_inertial_clicked();

  void keyPressEvent(QKeyEvent * e);
  void on_delete_inertial_clicked();

private:
  Ui::AddLink * ui;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDLINK_H_
