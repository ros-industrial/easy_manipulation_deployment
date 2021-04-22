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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDEXTERNALJOINT_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDEXTERNALJOINT_H_


#include <QDialog>
#include <string>
#include <vector>

#include "attributes/external_joint.h"
#include "attributes/object.h"

namespace Ui
{
  class AddExternalJoint; // NOLINT
}

class AddExternalJoint: public QDialog
{
  Q_OBJECT

public:
  bool success;
  bool editing_mode;
  Object target_object;
  std::vector < Object > available_objects;  // Objects that can be
                                             // connected to the target_object
  std::vector < Object > all_objects;     // All Objects in scene
  int ErrorCheckAxis();
  int ErrorCheckOrigin();
  void LoadExternalJoint();
  void LoadNewExternalJoint();
  explicit AddExternalJoint(QWidget * parent = nullptr);
  ~AddExternalJoint();

private slots:
  void on_includeaxis_2_stateChanged(int arg1);

  void DisableAxis();

  void on_parent_object_currentIndexChanged(int index);

  void on_ok_2_clicked();

  void on_exit_2_clicked();

  void on_include_origin_stateChanged(int arg1);

  void on_connect_world_stateChanged(int arg1);

  void keyPressEvent(QKeyEvent * e);

private:
  Ui::AddExternalJoint * ui;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDEXTERNALJOINT_H_
