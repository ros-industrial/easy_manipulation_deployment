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

#include "gui/addcollision.h"
#include <QFileDialog>
#include <QKeyEvent>
#include <iostream>
#include <string>

#include "gui/ui_addcollision.h"
#include "attributes/collision.h"

AddCollision::AddCollision(QWidget * parent)
: QDialog(parent),
  ui(new Ui::AddCollision)
{
  editing_mode = false;
  ui->setupUi(this);
  ui->stl_select->toggle();
  AddCollision::on_includeorigin_stateChanged(0);
  success = false;

  collision.name = "None";
}

AddCollision::~AddCollision()
{
  delete ui;
}

int AddCollision::ErrorCheckOrigin()
{
  int num_errors = 0;
  if (collision.origin.is_origin) {
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
            collision.origin.x = input_x.toFloat();
            collision.origin.y = input_y.toFloat();
            collision.origin.z = input_z.toFloat();
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
            collision.origin.roll = input_roll.toFloat();
            collision.origin.pitch = input_pitch.toFloat();
            collision.origin.yaw = input_yaw.toFloat();
          }
        }
      }
    }
  }
  return num_errors;
}

int AddCollision::ErrorCheckGeometry()
{
  int num_errors = 0;
  if (collision.geometry.is_stl) {
    std::string filetype = "";
    for (auto it = collision.geometry.filepath.crbegin(); it != collision.geometry.filepath.crend();
      ++it)
    {
      if (*it != '.') {
        filetype = std::string(1, *it) + filetype;
      } else {
        break;
      }
    }
    // get the new filepath which will replace the current stl filepath after it is copied
    std::string filename = "";
    for (auto it = collision.geometry.filepath.crbegin(); it != collision.geometry.filepath.crend();
      ++it)
    {
      if (*it != '/') {
        filename = std::string(1, *it) + filename;
      } else {
        break;
      }
    }
    collision.geometry.filepath_new = workcell_path.string() + "/assets/environment/" +
      "_description/meshes/collision/" + filename;

    if (filetype != "stl") {
      collision.geometry.filepath = "None";
      ui->errorlist->append("File Type Error: Not an stl file.");
      num_errors++;
    }

    bool all_empty_scale =
      (ui->scale_x->text().isEmpty() && ui->scale_y->text().isEmpty() &&
      ui->scale_z->text().isEmpty());
    bool all_full_scale =
      (!ui->scale_x->text().isEmpty() && !ui->scale_y->text().isEmpty() &&
      !ui->scale_z->text().isEmpty());
    if (all_empty_scale) {
      ui->errorlist->
      append("All Scale Fields are empty.Leave all fields at 1 for original STL size");
      num_errors++;
    } else {
      if (!all_full_scale) {
        ui->errorlist->append(
          "Scale Fields not complete. Leave all fields at 1 for original STL size");
        num_errors++;
      } else {
        int i = 0;
        auto float_validator = new QDoubleValidator();
        QString input_scalex = ui->scale_x->text();
        QString input_scaley = ui->scale_y->text();
        QString input_scalez = ui->scale_z->text();
        if (float_validator->validate(
            input_scalex,
            i) != QValidator::Acceptable ||
          float_validator->validate(
            input_scaley,
            i) != QValidator::Acceptable ||
          float_validator->validate(input_scalez, i) != QValidator::Acceptable)
        {
          ui->errorlist->append("Type Error: Scale need to be floats");
          num_errors++;
        } else {
          collision.geometry.scale_x = input_scalex.toFloat();
          collision.geometry.scale_y = input_scaley.toFloat();
          collision.geometry.scale_z = input_scalez.toFloat();
        }
      }
    }
  } else {
    // bool all_empty = false;
    bool all_full = false;
    QString input_1 = ui->parameter1->text();
    QString input_2 = ui->parameter2->text();
    QString input_3 = ui->parameter3->text();
    int i = 0;

    auto float_validator = new QDoubleValidator();
    if (collision.geometry.shape == "Box") {
      all_full =
        (!ui->parameter1->text().isEmpty() && !ui->parameter2->text().isEmpty() &&
        !ui->parameter3->text().isEmpty());
      if (!all_full) {
        ui->errorlist->append("Fill up all required parameters");
        num_errors++;
      } else {
        if (float_validator->validate(
            input_1,
            i) != QValidator::Acceptable ||
          float_validator->validate(
            input_2,
            i) != QValidator::Acceptable ||
          float_validator->validate(input_3, i) != QValidator::Acceptable)
        {
          ui->errorlist->append("Type Error: Parameters need to be floats");
          num_errors++;
        } else {
          collision.geometry.length = input_1.toFloat();
          collision.geometry.breadth = input_2.toFloat();
          collision.geometry.height = input_3.toFloat();
        }
      }
    } else if (collision.geometry.shape == "Cylinder") {
      all_full = (!ui->parameter1->text().isEmpty() && !ui->parameter2->text().isEmpty());
      if (!all_full) {
        ui->errorlist->append("Fill up all required parameters");
        num_errors++;
      } else {
        if (float_validator->validate(
            input_1,
            i) != QValidator::Acceptable ||
          float_validator->validate(input_2, i) != QValidator::Acceptable)
        {
          ui->errorlist->append("Type Error: Parameters need to be floats");
          num_errors++;
        } else {
          collision.geometry.height = input_1.toFloat();
          collision.geometry.radius = input_2.toFloat();
        }
      }
    } else if (collision.geometry.shape == "Sphere") {
      all_full = (!ui->parameter1->text().isEmpty());
      if (!all_full) {
        ui->errorlist->append("Fill up all required parameters");
        num_errors++;
      } else {
        if (float_validator->validate(input_1, i) != QValidator::Acceptable) {
          ui->errorlist->append("Type Error: Parameters need to be floats");
          num_errors++;
        } else {
          collision.geometry.radius = input_1.toFloat();
        }
      }
    }
  }
  return num_errors;
}

void AddCollision::on_geometry_shape_currentIndexChanged(int index)
{
  if (index == 0) {
    //  Box
    collision.geometry.shape = "Box";
    collision.geometry.radius = -1;

    ui->parameter3->setDisabled(false);
    ui->parameter3_label->setText("Height");
    ui->parameter3_label->show();
    ui->parameter3_label->setDisabled(false);

    ui->parameter2->setDisabled(false);
    ui->parameter2_label->setText("Breadth");
    ui->parameter2_label->show();
    ui->parameter2_label->setDisabled(false);

    ui->parameter1->setDisabled(false);
    ui->parameter1_label->setText("Length");
    ui->parameter1_label->show();
    ui->parameter1_label->setDisabled(false);

  } else if (index == 1) {
    //  Cylinder
    collision.geometry.shape = "Cylinder";
    collision.geometry.length = -1;
    collision.geometry.breadth = -1;

    ui->parameter3->setDisabled(true);
    ui->parameter3_label->hide();

    ui->parameter2->setDisabled(false);
    ui->parameter2_label->setText("Radius");
    ui->parameter2_label->show();
    ui->parameter2_label->setDisabled(false);

    ui->parameter1->setDisabled(false);
    ui->parameter1_label->setText("Height");
    ui->parameter1_label->show();
    ui->parameter1_label->setDisabled(false);

  } else if (index == 2) {
    //  Circle
    collision.geometry.shape = "Sphere";
    collision.geometry.length = -1;
    collision.geometry.breadth = -1;
    collision.geometry.height = -1;


    ui->parameter3->setDisabled(true);
    ui->parameter3_label->hide();

    ui->parameter2->setDisabled(true);
    ui->parameter2_label->hide();

    ui->parameter1->setDisabled(false);
    ui->parameter1_label->setText("Radius");
  }
}

void AddCollision::on_includeorigin_stateChanged(int arg1)
{
  if (arg1 == 0) {
    //  No Origin
    collision.origin.disableOrigin();
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
    //  Have origin
    collision.origin.is_origin = true;
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

void AddCollision::on_stl_select_toggled(bool checked)
{
  if (checked) {
    //  Use STL
    collision.geometry.disableShape();
    ui->MeshFIle->setDisabled(false);
    ui->geometry_filename->setDisabled(false);

    ui->scale_x->setDisabled(false);
    ui->scale_y->setDisabled(false);
    ui->scale_z->setDisabled(false);

    ui->scale_x_label->setDisabled(false);
    ui->scale_y_label->setDisabled(false);
    ui->scale_z_label->setDisabled(false);

    ui->geometry_shape->setDisabled(true);
    ui->parameter3->setDisabled(true);
    ui->parameter3_label->setDisabled(true);

    ui->parameter2->setDisabled(true);
    ui->parameter2_label->setDisabled(true);

    ui->parameter1->setDisabled(true);
    ui->parameter1_label->setDisabled(true);
  } else {
    //  Use Shape
    collision.geometry.disableSTL();

    ui->MeshFIle->setDisabled(true);
    ui->geometry_filename->setDisabled(true);

    ui->scale_x->setDisabled(true);
    ui->scale_y->setDisabled(true);
    ui->scale_z->setDisabled(true);

    ui->scale_x_label->setDisabled(true);
    ui->scale_y_label->setDisabled(true);
    ui->scale_z_label->setDisabled(true);

    ui->geometry_shape->setCurrentIndex(0);
    ui->geometry_shape->setDisabled(false);

    AddCollision::on_geometry_shape_currentIndexChanged(ui->geometry_shape->currentIndex());
    this->on_geometry_shape_currentIndexChanged(0);
  }
}

void AddCollision::on_collision_name_textChanged(const QString & arg1)
{
  collision.name = arg1.toStdString();
}

int AddCollision::ErrorCollisionName()
{
  int num_errors = 0;
  if (!editing_mode) {
    for (std::string name : available_collisions) {
      if (name.compare(collision.name) == 0) {
        ui->errorlist->append("Name Error: Collision Link of this name already exists ");
        num_errors++;
      }
    }
  }
  if (collision.name.compare("cylinder") == 0 || collision.name.compare("box") == 0 ||
    collision.name.compare("sphere") == 0)
  {
    ui->errorlist->append(
      QString::fromStdString(
        "Name Error: " + collision.name +
        " is a reserved name. Please use another name for your visual "));
    num_errors++;
  }
  return num_errors;
}

void AddCollision::on_ok_clicked()
{
  ui->errorlist->clear();

  if (ErrorCheckOrigin() + ErrorCheckGeometry() + ErrorCollisionName() == 0) {
    success = true;
    this->close();
  }
  //  Check Origin Error
}

void AddCollision::on_exit_clicked()
{
  success = false;
  this->close();
}

void AddCollision::load_collision(Collision collision)
{
  editing_mode = true;
  success = true;
  collision.name = collision.name;
  collision.origin = collision.origin;
  collision.geometry = collision.geometry;

  if (collision.name != "None") {
    ui->collision_name->setText(QString::fromStdString(collision.name));
  }
  if (collision.origin.is_origin) {
    ui->includeorigin->setChecked(true);
    on_includeorigin_stateChanged(2);
    if (collision.origin.x >= 0) {ui->x->setText(QString::number(collision.origin.x));}
    if (collision.origin.y >= 0) {ui->y->setText(QString::number(collision.origin.y));}
    if (collision.origin.z >= 0) {ui->z->setText(QString::number(collision.origin.z));}
    if (collision.origin.roll >= 0) {ui->roll->setText(QString::number(collision.origin.roll));}
    if (collision.origin.pitch >= 0) {ui->pitch->setText(QString::number(collision.origin.pitch));}
    if (collision.origin.yaw >= 0) {ui->yaw->setText(QString::number(collision.origin.yaw));}
  } else {
    on_includeorigin_stateChanged(0);
  }

  if (collision.geometry.is_stl) {
    ui->stl_select->toggle();
    on_stl_select_toggled(true);
    ui->geometry_filename->setText(QString::fromStdString(collision.geometry.filepath)); \
    ui->scale_x->setText(QString::number(collision.geometry.scale_x));
    ui->scale_y->setText(QString::number(collision.geometry.scale_y));
    ui->scale_z->setText(QString::number(collision.geometry.scale_z));
  } else {
    ui->geometry_select->toggle();
    on_stl_select_toggled(false);
    if (collision.geometry.shape == "Box") {
      ui->geometry_shape->setCurrentIndex(0);
      on_geometry_shape_currentIndexChanged(0);
      ui->parameter1->setText(QString::number(collision.geometry.length));
      ui->parameter2->setText(QString::number(collision.geometry.breadth));
      ui->parameter3->setText(QString::number(collision.geometry.height));
    } else if (collision.geometry.shape == "Cylinder") {
      ui->geometry_shape->setCurrentIndex(1);
      on_geometry_shape_currentIndexChanged(1);
      ui->parameter1->setText(QString::number(collision.geometry.height));
      ui->parameter2->setText(QString::number(collision.geometry.radius));
    } else if (collision.geometry.shape == "Sphere") {
      ui->geometry_shape->setCurrentIndex(2);
      on_geometry_shape_currentIndexChanged(2);
      ui->parameter1->setText(QString::number(collision.geometry.radius));
    }
  }
}

void AddCollision::on_MeshFIle_clicked()
{
  QString mesh_file = QFileDialog::getOpenFileName(this, "Open mesh filepath", QDir::homePath());
  ui->geometry_filename->setText(mesh_file);
  collision.geometry.filepath = mesh_file.toStdString();
}

void AddCollision::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}
