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

#include "gui/addinertial.h"
#include <QKeyEvent>
#include <QDoubleValidator>
#include "gui/ui_addinertial.h"

AddInertial::AddInertial(QWidget * parent)
: QDialog(parent),
  ui(new Ui::AddInertial)
{
  editing_mode = false;
  ui->setupUi(this);
  AddInertial::on_includeorigin_stateChanged(0);
  success = false;
}

AddInertial::~AddInertial()
{
  delete ui;
}

void AddInertial::on_includeorigin_stateChanged(int arg1)
{
  if (arg1 == 0) {
    // No Origin
    inertial.origin.disableOrigin();
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
    ui->position_label->setDisabled(true);
    ui->orientation_label->setDisabled(true);
  } else {
    // Have origin
    inertial.origin.is_origin = true;

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
    ui->position_label->setDisabled(false);
    ui->orientation_label->setDisabled(false);
  }
}

int AddInertial::ErrorCheckOrigin()
{
  int num_errors = 0;
  if (inertial.origin.is_origin) {
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
      num_errors++;
    } else {
      if (!all_full_xyz && !all_empty_xyz) {
        ui->errorlist->append("XYZ values not complete. Leave it all blank for default values");
        num_errors++;
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
            num_errors++;
          } else {
            inertial.origin.x = input_x.toFloat();
            inertial.origin.y = input_y.toFloat();
            inertial.origin.z = input_z.toFloat();
          }
        }
      }
      if (!all_full_rpy && !all_empty_rpy) {
        ui->errorlist->append("RPY values not complete. Leave it all blank for default values");
        num_errors++;
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
            num_errors++;
          } else {
            inertial.origin.roll = input_roll.toFloat();
            inertial.origin.pitch = input_pitch.toFloat();
            inertial.origin.yaw = input_yaw.toFloat();
          }
        }
      }
    }
  }
  return num_errors;
}

int AddInertial::ErrorCheckInertia()
{
  int num_errors = 0;
  bool all_empty_inertial =
    (ui->ixx->text().isEmpty() && ui->ixy->text().isEmpty() && ui->ixz->text().isEmpty() &&
    ui->iyy->text().isEmpty() && ui->iyz->text().isEmpty() && ui->izz->text().isEmpty());
  bool all_full_inertial =
    (!ui->ixx->text().isEmpty() && !ui->ixy->text().isEmpty() && !ui->ixz->text().isEmpty() &&
    !ui->iyy->text().isEmpty() && !ui->iyz->text().isEmpty() && !ui->izz->text().isEmpty());

  if (all_empty_inertial) {
    ui->errorlist->append(
      "Some inertia fields are not filled up. All inertia fiekds are required to be filled.");
    num_errors++;
  } else {
    if (!all_full_inertial) {
      ui->errorlist->append("Inertia values not complete. Please fill up all values");
      num_errors++;
    } else {
      if (all_full_inertial) {
        int i = 0;
        auto float_validator = new QDoubleValidator();
        QString input_ixx = ui->ixx->text();
        QString input_ixy = ui->ixy->text();
        QString input_ixz = ui->ixz->text();
        QString input_iyy = ui->iyy->text();
        QString input_iyz = ui->iyz->text();
        QString input_izz = ui->izz->text();
        if (float_validator->validate(
            input_ixx,
            i) != QValidator::Acceptable ||
          float_validator->validate(
            input_ixy,
            i) != QValidator::Acceptable ||
          float_validator->validate(
            input_ixz,
            i) != QValidator::Acceptable ||
          float_validator->validate(
            input_iyy,
            i) != QValidator::Acceptable ||
          float_validator->validate(
            input_iyz,
            i) != QValidator::Acceptable ||
          float_validator->validate(input_izz, i) != QValidator::Acceptable)
        {
          ui->errorlist->append("Type Error: all inertia values need to be floats.");
          num_errors++;
        } else {
          inertial.ixx = input_ixx.toFloat();
          inertial.ixy = input_ixy.toFloat();
          inertial.ixz = input_ixz.toFloat();
          inertial.iyy = input_iyy.toFloat();
          inertial.iyz = input_iyz.toFloat();
          inertial.izz = input_izz.toFloat();
        }
      }
    }
  }
  return num_errors;
}

int AddInertial::ErrorCheckMass()
{
  int num_errors = 0;
  bool all_empty_mass = (ui->mass->text().isEmpty());
  bool all_full_mass = (!ui->mass->text().isEmpty());

  if (all_empty_mass) {
    ui->errorlist->append("Mass value is not filled up");
    num_errors++;
  } else {
    if (all_full_mass) {
      int i = 0;
      auto float_validator = new QDoubleValidator();
      QString input_mass = ui->mass->text();
      if (float_validator->validate(input_mass, i) != QValidator::Acceptable) {
        ui->errorlist->append("Type Error: Mass value needs to be a float.");
        num_errors++;
      } else {
        inertial.mass = input_mass.toFloat();
      }
    }
  }
  return num_errors;
}

void AddInertial::on_ok_clicked()
{
  ui->errorlist->clear();

  int origin_errors = ErrorCheckOrigin();
  int inertia_errors = ErrorCheckInertia();
  int mass_errors = ErrorCheckMass();
  if (origin_errors + inertia_errors + mass_errors == 0) {
    success = true;
    this->close();
  }
}

void AddInertial::on_exit_clicked()
{
  success = false;
  this->close();
}

void AddInertial::load_inertial(Inertial input_inertial)
{
  editing_mode = true;
  success = true;
  inertial.origin = input_inertial.origin;
  inertial.ixx = input_inertial.ixx;
  inertial.ixy = input_inertial.ixy;
  inertial.ixz = input_inertial.ixz;
  inertial.iyy = input_inertial.iyy;
  inertial.iyz = input_inertial.iyz;
  inertial.izz = input_inertial.izz;
  inertial.mass = input_inertial.mass;

  ui->ixx->setText(QString::number(inertial.ixx));
  ui->ixy->setText(QString::number(inertial.ixy));
  ui->ixz->setText(QString::number(inertial.ixz));
  ui->iyy->setText(QString::number(inertial.iyy));
  ui->iyz->setText(QString::number(inertial.iyz));
  ui->izz->setText(QString::number(inertial.izz));
  ui->mass->setText(QString::number(inertial.mass));


  if (inertial.origin.is_origin) {
    ui->includeorigin->setChecked(true);
    on_includeorigin_stateChanged(2);
    if (inertial.origin.x >= 0) {ui->x->setText(QString::number(inertial.origin.x));}
    if (inertial.origin.y >= 0) {ui->y->setText(QString::number(inertial.origin.y));}
    if (inertial.origin.z >= 0) {ui->z->setText(QString::number(inertial.origin.z));}
    if (inertial.origin.roll >= 0) {ui->roll->setText(QString::number(inertial.origin.roll));}
    if (inertial.origin.pitch >= 0) {ui->pitch->setText(QString::number(inertial.origin.pitch));}
    if (inertial.origin.yaw >= 0) {ui->yaw->setText(QString::number(inertial.origin.yaw));}
  } else {
    on_includeorigin_stateChanged(0);
  }
}

void AddInertial::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}
