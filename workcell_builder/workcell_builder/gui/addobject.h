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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDOBJECT_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDOBJECT_H_

#include <QDialog>
#include <QListWidgetItem>
#include <string>
#include <vector>

#include "boost/filesystem.hpp"
#include "attributes/object.h"

namespace Ui
{
  class AddObject; // NOLINT
}

class AddObject: public QDialog
{
  Q_OBJECT

public:
  bool success;
  bool editing_mode;
  Object object;

  std::vector < std::string > available_object_names;
  explicit AddObject(QWidget * parent = nullptr);
  void LoadObject(Object input_object);
  bool ErrorObjectName();

  boost::filesystem::path workcell_path;
  ~AddObject();

private slots:
  void on_AddLink_clicked();

  void on_delete_link_clicked();

  void on_link_list_itemDoubleClicked(QListWidgetItem * item);

  void on_AddJoint_clicked();

  void on_joint_list_itemDoubleClicked(QListWidgetItem * item);

  void on_DeleteJoint_clicked();

  void on_pushButton_clicked();

  void on_pushButton_2_clicked();

  void on_lineEdit_textChanged(const QString & arg1);

  void on_available_links_currentIndexChanged(int index);
  void keyPressEvent(QKeyEvent * e);

private:
  Ui::AddObject * ui;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDOBJECT_H_
