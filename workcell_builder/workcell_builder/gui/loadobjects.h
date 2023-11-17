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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__LOADOBJECTS_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__LOADOBJECTS_H_

#include <QDialog>
#include <string>
#include <vector>

#include "attributes/object.h"

namespace Ui
{
  class LoadObjects; // NOLINT
}

class LoadObjects: public QDialog
{
  Q_OBJECT

public:
  bool success;
  std::vector < std::string > available_objects;
  Object chosen_object;
  std::vector < std::string > current_object_names;

  void get_all_objects();
  bool load_object_from_yaml(std::string object_name);

  explicit LoadObjects(QWidget * parent = nullptr);
  ~LoadObjects();

private slots:
  void on_ok_clicked();

  void on_exit_clicked();

  void on_available_objects_currentIndexChanged(int index);

private:
  Ui::LoadObjects * ui;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__LOADOBJECTS_H_
