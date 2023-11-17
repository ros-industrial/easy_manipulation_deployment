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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDINERTIAL_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDINERTIAL_H_

#include <QDialog>
#include "attributes/inertia.h"

namespace Ui
{
  class AddInertial; // NOLINT
}

class AddInertial: public QDialog
{
  Q_OBJECT

public:
  bool editing_mode;
  bool success;
  bool is_error;
  Inertial inertial;
  void load_inertial(Inertial inertial);
  int ErrorCheckOrigin();
  int ErrorCheckInertia();
  int ErrorCheckMass();

  explicit AddInertial(QWidget * parent = nullptr);
  ~AddInertial();

private slots:
  void on_includeorigin_stateChanged(int arg1);

  void on_ok_clicked();

  void on_exit_clicked();

  void keyPressEvent(QKeyEvent * e);

private:
  Ui::AddInertial * ui;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDINERTIAL_H_
