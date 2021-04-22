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

#include "gui/addlink.h"

#include <QKeyEvent>
#include <QString>
#include <iostream>
#include <string>
#include <vector>


#include "gui/ui_addlink.h"
#include "gui/addcollision.h"
#include "gui/addinertial.h"
#include "gui/addvisual.h"


AddLink::AddLink(QWidget * parent)
: QDialog(parent),
  ui(new Ui::AddLink)
{
  editing_mode = false;
  ui->setupUi(this);
  success = false;

  on_enable_visual_stateChanged(0);
  on_enable_collision_stateChanged(0);
  on_enable_inertial_stateChanged(0);
}

AddLink::~AddLink()
{
  delete ui;
}

void AddLink::on_lineEdit_textChanged(const QString & arg1)
{
  link.name = arg1.toStdString();
}

void AddLink::load_link(Link link_input)
{
  editing_mode = true;
  success = true;
  ui->lineEdit->setText(QString::fromStdString(link_input.name));
  link = link_input;
  if (link.visual_vector.size() == 0) {
    on_enable_visual_stateChanged(0);
  } else {
    ui->enable_visual->toggle();
    ui->edit_visual->setDisabled(false);
    ui->delete_visual->setDisabled(false);
    ui->create_visual->setDisabled(true);
  }

  if (link.collision_vector.size() == 0) {
    on_enable_collision_stateChanged(0);
  } else {
    ui->enable_collision->toggle();
    ui->edit_collision->setDisabled(false);
    ui->delete_collision->setDisabled(false);
    ui->create_collision->setDisabled(true);
  }

  if (link.inertial_vector.size() == 0) {
    on_enable_inertial_stateChanged(0);
  } else {
    ui->enable_inertial->toggle();
    ui->edit_inertial->setDisabled(false);
    ui->delete_inertial->setDisabled(false);
    ui->create_inertial->setDisabled(true);
  }
}

void AddLink::on_enable_inertial_stateChanged(int arg1)
{
  if (arg1 == 0) {
    link.is_inertial = false;
    ui->create_inertial->setDisabled(true);
    ui->edit_inertial->setDisabled(true);
    ui->delete_inertial->setDisabled(true);
  } else {
    link.is_inertial = true;
    ui->create_inertial->setDisabled(false);
  }
}
void AddLink::on_create_inertial_clicked()
{
  AddInertial inertial_window;
  inertial_window.setModal(true);
  inertial_window.setWindowTitle("Add Inertial");
  inertial_window.exec();
  if (inertial_window.success) {
    link.inertial_vector.push_back(inertial_window.inertial);
    ui->create_inertial->setDisabled(true);
    ui->edit_inertial->setDisabled(false);
    ui->delete_inertial->setDisabled(false);
  }
}
void AddLink::on_edit_inertial_clicked()
{
  AddInertial inertial_window;
  inertial_window.load_inertial(link.inertial_vector[0]);
  inertial_window.inertial = link.inertial_vector[0];
  inertial_window.setModal(true);
  inertial_window.setWindowTitle("Add Inertial");
  inertial_window.exec();

  if (inertial_window.success) {
    link.inertial_vector.clear();
    link.inertial_vector.push_back(inertial_window.inertial);
  }
}
void AddLink::on_delete_inertial_clicked()
{
  link.inertial_vector.clear();
  ui->create_inertial->setDisabled(false);
  ui->edit_inertial->setDisabled(true);
  ui->delete_inertial->setDisabled(true);
}

void AddLink::on_enable_visual_stateChanged(int arg1)
{
  if (arg1 == 0) {
    link.visual_vector.clear();
    link.is_visual = false;
    ui->create_visual->setDisabled(true);
    ui->edit_visual->setDisabled(true);
    ui->delete_visual->setDisabled(true);
  } else {
    link.is_visual = true;
    ui->create_visual->setDisabled(false);
  }
}
void AddLink::on_create_visual_clicked()
{
  AddVisual visual_window;
  visual_window.workcell_path = workcell_path;
  visual_window.setModal(true);
  visual_window.setWindowTitle("Add Visual");
  visual_window.exec();

  if (visual_window.success) {
    link.visual_vector.push_back(visual_window.visual);
    ui->create_visual->setDisabled(true);
    ui->edit_visual->setDisabled(false);
    ui->delete_visual->setDisabled(false);
  }
}
void AddLink::on_edit_visual_clicked()
{
  AddVisual visual_window;
  visual_window.load_visual(link.visual_vector[0]);
  visual_window.visual = link.visual_vector[0];
  visual_window.setModal(true);
  visual_window.setWindowTitle("Edit Visual");
  visual_window.workcell_path = workcell_path;
  visual_window.exec();
  if (visual_window.success) {
    link.visual_vector.clear();
    link.visual_vector.push_back(visual_window.visual);
  }
}
void AddLink::on_delete_visual_clicked()
{
  link.visual_vector.clear();
  ui->create_visual->setDisabled(false);
  ui->edit_visual->setDisabled(true);
  ui->delete_visual->setDisabled(true);
}

void AddLink::on_enable_collision_stateChanged(int arg1)
{
  if (arg1 == 0) {
    link.collision_vector.clear();
    link.is_collision = false;
    ui->create_collision->setDisabled(true);
    ui->edit_collision->setDisabled(true);
    ui->delete_collision->setDisabled(true);
  } else {
    link.is_collision = true;
    ui->create_collision->setDisabled(false);
  }
}
void AddLink::on_create_collision_clicked()
{
  AddCollision collision_window;
  collision_window.setModal(true);
  collision_window.setWindowTitle("Add Collision");
  collision_window.workcell_path = workcell_path;
  collision_window.exec();

  if (collision_window.success) {
    link.collision_vector.push_back(collision_window.collision);
    ui->create_collision->setDisabled(true);
    ui->edit_collision->setDisabled(false);
    ui->delete_collision->setDisabled(false);
  }
}
void AddLink::on_edit_collision_clicked()
{
  AddCollision collision_window;
  collision_window.load_collision(link.collision_vector[0]);
  collision_window.collision = link.collision_vector[0];
  collision_window.setModal(true);
  collision_window.setWindowTitle("Edit Collision");
  collision_window.workcell_path = workcell_path;
  collision_window.exec();
  if (collision_window.success) {
    link.collision_vector.clear();
    link.collision_vector.push_back(collision_window.collision);
  }
}
void AddLink::on_delete_collision_clicked()
{
  link.collision_vector.clear();
  ui->create_collision->setDisabled(false);
  ui->edit_collision->setDisabled(true);
  ui->delete_collision->setDisabled(true);
}

bool AddLink::ErrorLinkName()
{
  if (link.name.find_first_not_of(' ') != std::string::npos) {
    for (std::string name : available_link_names) {
      if (name.compare(link.name) == 0) {
        ui->error_list->append("Name Error: Link of this name already exists ");
        return true;
      }
    }
  } else {
    ui->error_list->append("Link Name required.");
    return true;
  }

  if (link.name.compare("cylinder") == 0 || link.name.compare("box") == 0 ||
    link.name.compare("sphere") == 0)
  {
    ui->error_list->append(
      QString::fromStdString(
        "Name Error: " + link.name +
        " is a reserved name. Please use another name for your link "));
    return true;
  }

  return false;
}

bool AddLink::ErrorToggle()
{
  int errors = 0;
  if (ui->enable_visual->isChecked() && link.visual_vector.size() == 0) {
    ui->error_list->append(
      "No visual component created. Uncheck the box if you do not require a visual element");
    errors++;
  }

  if (ui->enable_collision->isChecked() && link.collision_vector.size() == 0) {
    ui->error_list->append(
      "No collision component created. Uncheck the box if you do not require a collision element");
    errors++;
  }

  if (ui->enable_inertial->isChecked() && link.inertial_vector.size() == 0) {
    ui->error_list->append(
      "No inertia component created. Uncheck the box if you do not require a inertia element");
    errors++;
  }

  if (errors > 0) {
    return true;
  } else {
    return false;
  }
}

void AddLink::on_ok_clicked()
{
  ui->error_list->clear();

  if (!ErrorLinkName() && !ErrorToggle() ) {
    success = true;
    this->close();
  }
}

void AddLink::on_close_clicked()
{
  this->close();
}

void AddLink::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}
