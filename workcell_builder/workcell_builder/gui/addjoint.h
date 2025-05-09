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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDJOINT_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDJOINT_H_

#include <QDialog>
#include <string>
#include <vector>

#include "attributes/joint.h"
#include "attributes/link.h"

namespace Ui
{
  class AddJoint; // NOLINT
}

class AddJoint: public QDialog
{
  Q_OBJECT

public:
  Joint joint;
  bool success;
  int edit_pos;
  std::vector < Link > available_links;
  std::vector < Joint > available_joints;
  std::vector < std::string > available_link_names;
  std::vector < int > included_child_pos;     // Current legal links for parent links

  std::vector < int > CheckPossibleChild();
  bool ErrorCheckOrigin();
  bool ErrorCheckLinkUpdate();
  bool ErrorCheckAxis();
  bool ErrorJointName();

  void load_joint(Joint joint);
  void LoadLinks();
  explicit AddJoint(QWidget * parent = nullptr);
  ~AddJoint();

private slots:
  void on_includeorigin_stateChanged(int arg1);

  void on_includeaxis_stateChanged(int arg1);

  void on_ok_clicked();

  void on_joint_type_currentIndexChanged(int index);

  void on_parent_link_currentIndexChanged(int index);

  void on_exit_clicked();

  void keyPressEvent(QKeyEvent * e);

private:
  Ui::AddJoint * ui;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDJOINT_H_
