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

#include "gui/addjoint.h"
#include <QString>
#include <QKeyEvent>
#include <iostream>
#include <vector>
#include <string>

#include "gui/ui_addjoint.h"

AddJoint::AddJoint(QWidget * parent)
: QDialog(parent),
  ui(new Ui::AddJoint)
{
  ui->setupUi(this);
  on_includeaxis_stateChanged(0);
  on_includeorigin_stateChanged(0);
  success = false;
  joint.type = "revolute";
  edit_pos = -1;
}

void AddJoint::LoadLinks()
{
  bool oldState = ui->parent_link->blockSignals(true);

  for (auto link : available_links) {
    ui->parent_link->addItem(QString::fromStdString(link.name));
  }
  ui->parent_link->blockSignals(oldState);
  on_parent_link_currentIndexChanged(0);
}

AddJoint::~AddJoint()
{
  delete ui;
}

void AddJoint::load_joint(Joint joint_input)
{
  success = true;

  // Populate Name
  ui->joint_name->setText(QString::fromStdString(joint_input.name));
  joint = joint_input;

  // Populate Origin
  if (joint.origin.is_origin) {
    ui->includeorigin->setChecked(true);
    on_includeorigin_stateChanged(2);
    if (joint.origin.x >= 0) {ui->x->setText(QString::number(joint.origin.x));}
    if (joint.origin.y >= 0) {ui->y->setText(QString::number(joint.origin.y));}
    if (joint.origin.z >= 0) {ui->z->setText(QString::number(joint.origin.z));}
    if (joint.origin.roll >= 0) {ui->roll->setText(QString::number(joint.origin.roll));}
    if (joint.origin.pitch >= 0) {ui->pitch->setText(QString::number(joint.origin.pitch));}
    if (joint.origin.yaw >= 0) {ui->yaw->setText(QString::number(joint.origin.yaw));}
  } else {
    on_includeorigin_stateChanged(0);
  }

  // Populate Axis
  if (joint.axis.is_axis) {
    ui->includeaxis->setChecked(true);
    on_includeaxis_stateChanged(2);
    if (joint.axis.x >= 0) {ui->axis_x->setText(QString::number(joint.axis.x));}
    if (joint.axis.y >= 0) {ui->axis_y->setText(QString::number(joint.axis.y));}
    if (joint.axis.z >= 0) {ui->axis_z->setText(QString::number(joint.axis.z));}
  } else {
    on_includeaxis_stateChanged(0);
  }

  // Populate Type

  if (joint.type == "revolute") {
    ui->joint_type->setCurrentIndex(0);
  } else if (joint.type == "continuous") {
    ui->joint_type->setCurrentIndex(1);
  } else if (joint.type == "prismatic") {
    ui->joint_type->setCurrentIndex(2);
  } else if (joint.type == "fixed") {
    ui->joint_type->setCurrentIndex(3);
  } else if (joint.type == "floating") {
    ui->joint_type->setCurrentIndex(4);
  } else if (joint.type == "planar") {
    ui->joint_type->setCurrentIndex(5);
  }
  for (int i = 0; i < static_cast<int>(available_links.size()); i++) {
    if (available_links[i].name.compare(joint.parent_link.name) == 0) {
      bool oldState = ui->parent_link->blockSignals(true);
      ui->parent_link->setCurrentIndex(i);
      ui->parent_link->blockSignals(oldState);
      on_parent_link_currentIndexChanged(i);
      break;
    }
  }
  for (int pos = 0; pos < static_cast<int>(included_child_pos.size()); pos++) {
    if (available_links[included_child_pos[pos]].name.compare(joint.child_link.name) == 0) {
      ui->child_link->setCurrentIndex(pos);
      break;
    }
  }
}


bool AddJoint::ErrorCheckLinkUpdate()  // Not used for now. Delete later?
{
  int child_loaded = 0;
  int parent_loaded = 0;
  for (int i = 0; i < static_cast<int>(available_links.size()); i++) {
    if (available_links[i].name.compare(joint.child_link.name) == 0) {
      ui->child_link->setCurrentIndex(i);
      child_loaded = 1;
    }
    if (available_links[i].name.compare(joint.parent_link.name) == 0) {
      ui->parent_link->setCurrentIndex(i);
      parent_loaded = 1;
    }
  }

  if (parent_loaded == 0) {
    ui->errorlist->append("Parent Link error: Link does not exist");
    return true;
  }
  if (child_loaded == 0) {
    ui->errorlist->append("Child Link error: Link does not exist");
    return true;
  }
  return false;
}

void AddJoint::on_includeorigin_stateChanged(int arg1)
{
  if (arg1 == 0) {
    // No Origin
    joint.origin.disableOrigin();

    ui->x->setDisabled(true);
    ui->x_label->setDisabled(true);
    ui->y->setDisabled(true);
    ui->y_label->setDisabled(true);
    ui->z->setDisabled(true);
    ui->z_label->setDisabled(true);
    ui->roll->setDisabled(true);
    ui->roll_label->setDisabled(true);
    ui->pitch->setDisabled(true);
    ui->pitch_label->setDisabled(true);
    ui->yaw->setDisabled(true);
    ui->yaw_label->setDisabled(true);
    ui->origin_label->setDisabled(true);
    ui->position_label->setDisabled(true);
    ui->orientation_label->setDisabled(true);
  } else {
    // Have origin
    joint.origin.is_origin = true;

    ui->x->setDisabled(false);
    ui->x_label->setDisabled(false);
    ui->y->setDisabled(false);
    ui->y_label->setDisabled(false);
    ui->z->setDisabled(false);
    ui->z_label->setDisabled(false);
    ui->roll->setDisabled(false);
    ui->roll_label->setDisabled(false);
    ui->pitch->setDisabled(false);
    ui->pitch_label->setDisabled(false);
    ui->yaw->setDisabled(false);
    ui->yaw_label->setDisabled(false);
    ui->origin_label->setDisabled(false);
    ui->position_label->setDisabled(false);
    ui->orientation_label->setDisabled(false);
  }
}

void AddJoint::on_includeaxis_stateChanged(int arg1)
{
  if (arg1 == 0) {
    // No Axis
    joint.axis.disableAxis();
    ui->axis_x->setDisabled(true);
    ui->axis_x_label->setDisabled(true);
    ui->axis_y->setDisabled(true);
    ui->axis_y_label->setDisabled(true);
    ui->axis_z->setDisabled(true);
    ui->axis_z_label->setDisabled(true);
    ui->axis_label->setDisabled(true);
    ui->axis_position_label->setDisabled(true);
  } else {
    // Have origin
    joint.axis.is_axis = true;
    ui->axis_x->setDisabled(false);
    ui->axis_x_label->setDisabled(false);
    ui->axis_y->setDisabled(false);
    ui->axis_y_label->setDisabled(false);
    ui->axis_z->setDisabled(false);
    ui->axis_z_label->setDisabled(false);
    ui->axis_label->setDisabled(false);
    ui->axis_position_label->setDisabled(false);
  }
}

void AddJoint::on_joint_type_currentIndexChanged(int index)
{
  if (index == 0) {
    joint.type = "revolute";
    ui->includeaxis->setDisabled(false);
  } else if (index == 1) {
    joint.type = "continuous";
    ui->includeaxis->setDisabled(false);
  } else if (index == 2) {
    joint.type = "prismatic";
    ui->includeaxis->setDisabled(false);
  } else if (index == 3) {
    joint.type = "fixed";     // No axis option for fixed joints
    on_includeaxis_stateChanged(0);
    ui->includeaxis->setDisabled(true);
  } else if (index == 4) {
    joint.type = "floating";     // No axis option for floating joints
    on_includeaxis_stateChanged(0);
    ui->includeaxis->setDisabled(true);
  } else if (index == 5) {
    joint.type = "planar";
    ui->includeaxis->setDisabled(false);
  }
}

void AddJoint::on_parent_link_currentIndexChanged(int index)
{
  joint.parent_link = available_links[index];
  std::vector<int> excluded_pos = CheckPossibleChild();
  bool oldState = ui->child_link->blockSignals(true);
  ui->child_link->clear();
  included_child_pos.clear();
  for (int i = 0; i < static_cast<int>(available_links.size()); i++) {
    bool add = true;
    for (int i2 = 0; i2 < static_cast<int>(excluded_pos.size()); i2++) {
      if (i == excluded_pos[i2]) {
        add = false;
        break;
      }
    }
    if (add) {
      included_child_pos.push_back(i);
      ui->child_link->addItem(QString::fromStdString(available_links[i].name));
    }
  }
  ui->child_link->blockSignals(oldState);
}

void AddJoint::on_ok_clicked()
{
  ui->errorlist->clear();
  joint.name = ui->joint_name->text().toStdString();
  std::string child_name = ui->child_link->currentText().toStdString();
  joint.parent_link = available_links[ui->parent_link->currentIndex()];
  joint.child_link = available_links[included_child_pos[ui->child_link->currentIndex()]];
  if (ErrorCheckOrigin() || ErrorCheckAxis() || ErrorJointName()) {} else {
    success = true;
    this->close();
  }
}

void AddJoint::on_exit_clicked()
{
  this->close();
}

std::vector<int> AddJoint::CheckPossibleChild()
{
  std::vector<int> excluded_pos;
  excluded_pos.push_back(ui->parent_link->currentIndex());   // You cant be your own child
  Link parent_link = available_links[ui->parent_link->currentIndex()];   // get current parent link

  for (int i = 0; i < static_cast<int>(available_joints.size()); i++) {
    // Check the child
    if (available_joints[i].child_link.name.compare(parent_link.name) == 0) {
      // Target link is already a child of this link
      Link parent_of_parent = available_joints[i].parent_link;
      for (int link = 0; link < static_cast<int>(available_links.size()); link++) {
        // check the pos of the link
        if (available_links[link].name.compare(parent_of_parent.name) == 0) {
          excluded_pos.push_back(link);
          break;
        }
      }
    }
  }
  return excluded_pos;
}

bool AddJoint::ErrorCheckOrigin()
{
  if (joint.origin.is_origin) {
    bool all_empty_xyz =
      (ui->x->text().isEmpty() && ui->y->text().isEmpty() && ui->z->text().isEmpty());
    bool all_full_xyz =
      (!ui->x->text().isEmpty() && !ui->y->text().isEmpty() && !ui->z->text().isEmpty());

    bool all_empty_rpy =
      (ui->roll->text().isEmpty() && ui->pitch->text().isEmpty() && ui->yaw->text().isEmpty());
    bool all_full_rpy =
      (!ui->roll->text().isEmpty() && !ui->pitch->text().isEmpty() && !ui->yaw->text().isEmpty());
    if (all_empty_rpy && all_empty_xyz) {
      ui->errorlist->append(
        "All Origin Fields are empty.Uncheck the Origin selection to disable the option.");
      return true;
    } else {
      if (!all_full_xyz && !all_empty_xyz) {
        ui->errorlist->append("XYZ values not complete. Leave it all blank for default values");
        return true;
      } else {
        if (all_full_xyz) {
          int i = 0;
          auto float_validator = new QDoubleValidator();
          QString input_x = ui->x->text();
          QString input_y = ui->y->text();
          QString input_z = ui->z->text();
          if (float_validator->validate(
              input_x,
              i) != QValidator::Acceptable ||
            float_validator->validate(
              input_y,
              i) != QValidator::Acceptable ||
            float_validator->validate(input_z, i) != QValidator::Acceptable)
          {
            ui->errorlist->append("Type Error: XYZ need to be floats");
            return true;
          } else {
            joint.origin.x = input_x.toFloat();
            joint.origin.y = input_y.toFloat();
            joint.origin.z = input_z.toFloat();
          }
        }
      }

      if (!all_full_rpy && !all_empty_rpy) {
        ui->errorlist->append("RPY values not complete. Leave it all blank for default values");
        return true;
      } else {
        if (all_full_rpy) {
          int i = 0;
          auto float_validator = new QDoubleValidator();
          QString input_roll = ui->roll->text();
          QString input_pitch = ui->pitch->text();
          QString input_yaw = ui->yaw->text();
          if (float_validator->validate(
              input_roll,
              i) != QValidator::Acceptable ||
            float_validator->validate(
              input_pitch,
              i) != QValidator::Acceptable ||
            float_validator->validate(input_yaw, i) != QValidator::Acceptable)
          {
            ui->errorlist->append("Type Error: RPY need to be floats");
            return true;
          } else {
            joint.origin.roll = input_roll.toFloat();
            joint.origin.pitch = input_pitch.toFloat();
            joint.origin.yaw = input_yaw.toFloat();
          }
        }
      }
    }
  }
  return false;
}

bool AddJoint::ErrorCheckAxis()
{
  if (joint.axis.is_axis) {
    bool all_empty_xyz =
      (ui->axis_x->text().isEmpty() && ui->axis_y->text().isEmpty() &&
      ui->axis_z->text().isEmpty());
    bool all_full_xyz =
      (!ui->axis_x->text().isEmpty() && !ui->axis_y->text().isEmpty() &&
      !ui->axis_z->text().isEmpty());

    if (all_empty_xyz) {
      ui->errorlist->append(
        "All Axis Fields are empty.Uncheck the Axis selection to disable the option.");
      return true;
    } else {
      if (!all_full_xyz && !all_empty_xyz) {
        ui->errorlist->append(
          "Axis XYZ values not complete."
          " Leave it all blank for default values");
        return true;
      } else {
        if (all_full_xyz) {
          int i = 0;
          auto float_validator = new QDoubleValidator();
          QString input_x = ui->axis_x->text();
          QString input_y = ui->axis_y->text();
          QString input_z = ui->axis_z->text();
          if (float_validator->validate(
              input_x,
              i) != QValidator::Acceptable ||
            float_validator->validate(
              input_y,
              i) != QValidator::Acceptable ||
            float_validator->validate(input_z, i) != QValidator::Acceptable)
          {
            ui->errorlist->append("Type Error: Axis XYZ need to be floats");
            return true;
          } else {
            joint.axis.x = input_x.toFloat();
            joint.axis.y = input_y.toFloat();
            joint.axis.z = input_z.toFloat();
          }
        }
      }
    }
  }
  return false;
}

bool AddJoint::ErrorJointName()
{
  if (joint.name.find_first_not_of(' ') != std::string::npos) {
    for (int i = 0; i < static_cast<int>(available_joints.size()); i++) {
      if (edit_pos >= 0) {   // If we are editing a joint, dont compare that joint name
        if (i != edit_pos) {
          if (available_joints[i].name.compare(joint.name) == 0) {
            ui->errorlist->setText("Name Error: Joint of this name already exists ");
            return true;
          }
        }
      }
    }
  } else {
    ui->errorlist->setText("Joint Name required.");
    return true;
  }
  return false;
}

void AddJoint::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}
