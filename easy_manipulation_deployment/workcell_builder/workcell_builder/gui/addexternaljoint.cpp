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

#include "gui/addexternaljoint.h"
#include <QKeyEvent>
#include <iostream>

#include "gui/ui_addexternaljoint.h"
#include "attributes/external_joint.h"

AddExternalJoint::AddExternalJoint(QWidget * parent)
: QDialog(parent),
  ui(new Ui::AddExternalJoint)
{
  ui->setupUi(this);
}

AddExternalJoint::~AddExternalJoint()
{
  delete ui;
}

void AddExternalJoint::on_include_origin_stateChanged(int arg1)
{
  if (arg1 == 0) {
    // No Origin
    target_object.ext_joint.origin.disableOrigin();
    ui->x_2->setDisabled(true);
    ui->x_label_2->setDisabled(true);
    ui->y_2->setDisabled(true);
    ui->y_label_2->setDisabled(true);
    ui->z_2->setDisabled(true);
    ui->z_label_2->setDisabled(true);
    ui->roll_2->setDisabled(true);
    ui->roll_label_2->setDisabled(true);
    ui->pitch_2->setDisabled(true);
    ui->pitch_label_2->setDisabled(true);
    ui->yaw_2->setDisabled(true);
    ui->yaw_label_2->setDisabled(true);
    ui->origin_label_2->setDisabled(true);
    ui->position_label_2->setDisabled(true);
    ui->orientation_label_2->setDisabled(true);
  } else {
    // Have origin
    target_object.ext_joint.origin.is_origin = true;

    ui->x_2->setDisabled(false);
    ui->x_label_2->setDisabled(false);
    ui->y_2->setDisabled(false);
    ui->y_label_2->setDisabled(false);
    ui->z_2->setDisabled(false);
    ui->z_label_2->setDisabled(false);
    ui->roll_2->setDisabled(false);
    ui->roll_label_2->setDisabled(false);
    ui->pitch_2->setDisabled(false);
    ui->pitch_label_2->setDisabled(false);
    ui->yaw_2->setDisabled(false);
    ui->yaw_label_2->setDisabled(false);
    ui->origin_label_2->setDisabled(false);
    ui->position_label_2->setDisabled(false);
    ui->orientation_label_2->setDisabled(false);
  }
}

void AddExternalJoint::on_includeaxis_2_stateChanged(int arg1)
{
  if (arg1 == 0) {
    // No Axis
    target_object.ext_joint.axis.disableAxis();
    ui->axis_x_2->setDisabled(true);
    ui->axis_x_label_2->setDisabled(true);
    ui->axis_y_2->setDisabled(true);
    ui->axis_y_label_2->setDisabled(true);
    ui->axis_z_2->setDisabled(true);
    ui->axis_z_label_2->setDisabled(true);
    ui->axis_label_2->setDisabled(true);
  } else {
    // Have Axis
    target_object.ext_joint.axis.is_axis = true;
    ui->axis_x_2->setDisabled(false);
    ui->axis_x_label_2->setDisabled(false);
    ui->axis_y_2->setDisabled(false);
    ui->axis_y_label_2->setDisabled(false);
    ui->axis_z_2->setDisabled(false);
    ui->axis_z_label_2->setDisabled(false);
    ui->axis_label_2->setDisabled(false);
  }
}


void AddExternalJoint::on_parent_object_currentIndexChanged(int index)
{
  ui->parent_link->clear();
  for (int i = 0; i < static_cast<int>(available_objects[index].link_vector.size()); i++) {
    ui->parent_link->addItem(QString::fromStdString(available_objects[index].link_vector[i].name));
  }
}

int AddExternalJoint::ErrorCheckOrigin()
{
  int num_errors = 0;
  if (target_object.ext_joint.origin.is_origin) {
    bool all_empty_xyz =
      (ui->x_2->text().isEmpty() && ui->y_2->text().isEmpty() && ui->z_2->text().isEmpty());
    bool all_full_xyz =
      (!ui->x_2->text().isEmpty() && !ui->y_2->text().isEmpty() && !ui->z_2->text().isEmpty());

    bool all_empty_rpy =
      (ui->roll_2->text().isEmpty() && ui->pitch_2->text().isEmpty() &&
      ui->yaw_2->text().isEmpty());
    bool all_full_rpy =
      (!ui->roll_2->text().isEmpty() && !ui->pitch_2->text().isEmpty() &&
      !ui->yaw_2->text().isEmpty());
    if (all_empty_rpy && all_empty_xyz) {
      ui->errorlist_2->append(
        "All Origin Fields are empty.Uncheck the Origin selection to disable the option.");
      num_errors++;
    } else {
      if (!all_full_xyz && !all_empty_xyz) {
        ui->errorlist_2->append("XYZ values not complete. Leave it all blank for default values");
        num_errors++;
      } else {
        if (all_full_xyz) {
          int i = 0;
          auto float_validator = new QDoubleValidator();
          QString input_x = ui->x_2->text();
          QString input_y = ui->y_2->text();
          QString input_z = ui->z_2->text();
          if (float_validator->validate(
              input_x,
              i) != QValidator::Acceptable ||
            float_validator->validate(
              input_y,
              i) != QValidator::Acceptable ||
            float_validator->validate(input_z, i) != QValidator::Acceptable)
          {
            ui->errorlist_2->append("Type Error: XYZ need to be floats");
            num_errors++;
          } else {
            target_object.ext_joint.origin.x = input_x.toFloat();
            target_object.ext_joint.origin.y = input_y.toFloat();
            target_object.ext_joint.origin.z = input_z.toFloat();
          }
        }
      }

      if (!all_full_rpy && !all_empty_rpy) {
        ui->errorlist_2->append("RPY values not complete. Leave it all blank for default values");
        num_errors++;
      } else {
        if (all_full_rpy) {
          int i = 0;
          auto float_validator = new QDoubleValidator();
          QString input_roll = ui->roll_2->text();
          QString input_pitch = ui->pitch_2->text();
          QString input_yaw = ui->yaw_2->text();
          if (float_validator->validate(
              input_roll,
              i) != QValidator::Acceptable ||
            float_validator->validate(
              input_pitch,
              i) != QValidator::Acceptable ||
            float_validator->validate(input_yaw, i) != QValidator::Acceptable)
          {
            ui->errorlist_2->append("Type Error: RPY need to be floats");
            num_errors++;
          } else {
            target_object.ext_joint.origin.roll = input_roll.toFloat();
            target_object.ext_joint.origin.pitch = input_pitch.toFloat();
            target_object.ext_joint.origin.yaw = input_yaw.toFloat();
          }
        }
      }
    }
  }
  return num_errors;
}

int AddExternalJoint::ErrorCheckAxis()
{
  int num_errors = 0;
  if (target_object.ext_joint.axis.is_axis) {
    bool all_empty_xyz =
      (ui->axis_x_2->text().isEmpty() && ui->axis_y_2->text().isEmpty() &&
      ui->axis_z_2->text().isEmpty());
    bool all_full_xyz =
      (!ui->axis_x_2->text().isEmpty() && !ui->axis_y_2->text().isEmpty() &&
      !ui->axis_z_2->text().isEmpty());

    if (all_empty_xyz) {
      ui->errorlist_2->
      append("All Axis Fields are empty.Uncheck the Axis selection to disable the option.");
      num_errors++;
    } else {
      if (!all_full_xyz && !all_empty_xyz) {
        ui->errorlist_2->
        append("Axis XYZ values not complete. Leave it all blank for default values");
        num_errors++;
      } else {
        if (all_full_xyz) {
          int i = 0;
          auto float_validator = new QDoubleValidator();
          QString input_x = ui->axis_x_2->text();
          QString input_y = ui->axis_y_2->text();
          QString input_z = ui->axis_z_2->text();
          if (float_validator->validate(
              input_x,
              i) != QValidator::Acceptable ||
            float_validator->validate(
              input_y,
              i) != QValidator::Acceptable ||
            float_validator->validate(input_z, i) != QValidator::Acceptable)
          {
            ui->errorlist_2->append("Type Error: Axis XYZ need to be floats");
            num_errors++;
          } else {
            target_object.ext_joint.axis.x = input_x.toFloat();
            target_object.ext_joint.axis.y = input_y.toFloat();
            target_object.ext_joint.axis.z = input_z.toFloat();
          }
        }
      }
    }
  }
  return num_errors;
}

void AddExternalJoint::on_ok_2_clicked()
{
  ui->errorlist_2->clear();

  int error_axis = ErrorCheckAxis();
  int error_origin = ErrorCheckOrigin();
  if (error_axis + error_origin == 0) {
    if (ui->connect_world->isChecked()) {
      target_object.ext_joint.parent_obj_pos = -1;
      target_object.ext_joint.parent_link_pos = -1;
    } else {
      for (int i = 0; i < static_cast<int>(all_objects.size()); i++) {
        if (ui->parent_object->currentText().toStdString().compare(all_objects[i].name) == 0) {
          target_object.ext_joint.parent_obj_pos = i;
          break;
        }
      }
      target_object.ext_joint.parent_link_pos = ui->parent_link->currentIndex();
    }
    success = true;
    this->close();
  }
}

void AddExternalJoint::on_exit_2_clicked()
{
  this->close();
}

void AddExternalJoint::DisableAxis()
{
  ui->includeaxis_2->setChecked(false);
  ui->axis_x_2->clear();
  ui->axis_y_2->clear();
  ui->axis_z_2->clear();


  ui->axis_x_label_2->setDisabled(true);
  ui->axis_y_label_2->setDisabled(true);
  ui->axis_z_label_2->setDisabled(true);
  ui->axis_x_2->setDisabled(true);
  ui->axis_y_2->setDisabled(true);
  ui->axis_z_2->setDisabled(true);
  ui->axis_label_2->setDisabled(true);
  //ui->includeaxis_2->setDisabled(true);
  ui->axis_position_label_2->setDisabled(true);
}

void AddExternalJoint::LoadNewExternalJoint()
{
  success = false;
  editing_mode = false;
  ui->includeaxis_2->setChecked(false);
  on_includeaxis_2_stateChanged(0);
  ui->include_origin->setChecked(false);
  on_include_origin_stateChanged(0);

  ui->child_link->setText(
    QString::fromStdString(
      target_object.link_vector[target_object.ext_joint.
      child_link_pos].name));
  ui->child_object->setText(QString::fromStdString(target_object.name));

  ui->connect_world->setChecked(true);
  on_connect_world_stateChanged(2);
  if (available_objects.size() > 0) {
    // populate available objects
    for (int i = 0; i < static_cast<int>(available_objects.size()); i++) {
      ui->parent_object->addItem(QString::fromStdString(available_objects[i].name));
    }

    on_parent_object_currentIndexChanged(0);
  } else {
    ui->connect_world->setDisabled(true);
    ui->connect_world_label->setText(
      "<font color='orange'>No other objects available in scene."
      "You need to connect this object to the World link</font>");
  }

  if (target_object.ext_joint.type.compare("fixed") == 0 ||
    target_object.ext_joint.type.compare("floating") == 0)
  {
    DisableAxis();
  }
}

void AddExternalJoint::LoadExternalJoint()
{
  editing_mode = true;
  success = true;

  // Populate Origin
  if (target_object.ext_joint.origin.is_origin) {
    ui->include_origin->setChecked(true);
    on_include_origin_stateChanged(1);
    if (target_object.ext_joint.origin.x >= 0) {
      ui->x_2->setText(QString::number(target_object.ext_joint.origin.x));
    }
    if (target_object.ext_joint.origin.y >= 0) {
      ui->y_2->setText(QString::number(target_object.ext_joint.origin.y));
    }
    if (target_object.ext_joint.origin.z >= 0) {
      ui->z_2->setText(QString::number(target_object.ext_joint.origin.z));
    }
    if (target_object.ext_joint.origin.roll >= 0) {
      ui->roll_2->setText(QString::number(target_object.ext_joint.origin.roll));
    }
    if (target_object.ext_joint.origin.pitch >= 0) {
      ui->pitch_2->setText(QString::number(target_object.ext_joint.origin.pitch));
    }
    if (target_object.ext_joint.origin.yaw >= 0) {
      ui->yaw_2->setText(QString::number(target_object.ext_joint.origin.yaw));
    }
  } else {
    ui->include_origin->setChecked(false);
    on_include_origin_stateChanged(0);
  }

  // Populate Axis
  if (target_object.ext_joint.axis.is_axis) {
    ui->includeaxis_2->setChecked(true);
    on_includeaxis_2_stateChanged(2);
    if (target_object.ext_joint.axis.x >= 0) {
      ui->axis_x_2->setText(QString::number(target_object.ext_joint.axis.x));
    }
    if (target_object.ext_joint.axis.y >= 0) {
      ui->axis_y_2->setText(QString::number(target_object.ext_joint.axis.y));
    }
    if (target_object.ext_joint.axis.z >= 0) {
      ui->axis_z_2->setText(QString::number(target_object.ext_joint.axis.z));
    }
  } else {
    on_includeaxis_2_stateChanged(0);
  }

  // Populate the child attributes of the joints
  ui->child_link->setText(
    QString::fromStdString(
      target_object.link_vector[target_object.ext_joint.
      child_link_pos].name));
  ui->child_object->setText(QString::fromStdString(target_object.name));

  if (available_objects.size() > 0) {
    for (int i = 0; i < static_cast<int>(available_objects.size()); i++) {
      ui->parent_object->addItem(QString::fromStdString(available_objects[i].name));
      if (target_object.ext_joint.parent_obj_pos >= 0 &&
        target_object.ext_joint.parent_link_pos >= 0)
      {
        if (available_objects[i].name.compare(
            all_objects[target_object.ext_joint.parent_obj_pos].
            name) == 0)  // Check for parent object
        {
          ui->parent_object->setCurrentIndex(i);  // Set combo box to the parent object
          on_parent_object_currentIndexChanged(i);
          for (int j = 0; j < static_cast<int>(available_objects[i].link_vector.size()); j++) {
            // Check for Parent link
            if (available_objects[i].link_vector[j].name.compare(
                all_objects[target_object.ext_joint
                .parent_obj_pos].link_vector[target_object.ext_joint.parent_link_pos].name) == 0)
            {
              ui->parent_link->setCurrentIndex(j);  // Set combo box to the parent link
              break;
            }
          }
          // break;
        }
      }
    }
    if (target_object.ext_joint.parent_obj_pos == -1 &&
      target_object.ext_joint.parent_link_pos == -1)
    {
      ui->connect_world->setChecked(true);
      on_connect_world_stateChanged(2);
    }
  } else {
    ui->connect_world->setChecked(true);
    ui->connect_world->setDisabled(true);
    on_connect_world_stateChanged(2);
    ui->connect_world_label->setText(
      "<font color='orange'>No other objects available in scene."
      " you need to connect this object to the World link</font>");
  }

  // Fixed and floating joints do not have axis
  if (target_object.ext_joint.type.compare("fixed") == 0 ||
    target_object.ext_joint.type.compare("floating") == 0)
  {
    DisableAxis();
  }
}

void AddExternalJoint::on_connect_world_stateChanged(int arg1)
{
  if (arg1 == 0) {
    ui->parent_link->setDisabled(false);
    ui->parent_link_label->setDisabled(false);
    ui->parent_object->setDisabled(false);
    ui->parent_object_label->setDisabled(false);
  } else {
    ui->parent_link->setDisabled(true);
    ui->parent_link_label->setDisabled(true);
    ui->parent_object->setDisabled(true);
    ui->parent_object_label->setDisabled(true);
  }
}

void AddExternalJoint::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}
