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

#include "gui/addobject.h"

#include <QKeyEvent>
#include <iostream>
#include <string>
#include <vector>

#include "gui/ui_addobject.h"
#include "gui/addlink.h"
#include "gui/addjoint.h"


AddObject::AddObject(QWidget * parent)
: QDialog(parent),
  ui(new Ui::AddObject)
{
  editing_mode = false;
  success = false;
  ui->setupUi(this);
}

AddObject::~AddObject()
{
  delete ui;
}


void AddObject::on_AddLink_clicked()
{
  AddLink link_window;
  std::vector<std::string> available_link_names;
  for (int i = 0; i < static_cast<int>(object.link_vector.size()); i++) {
    available_link_names.push_back(object.link_vector[i].name);
  }
  link_window.available_link_names = available_link_names;
  link_window.workcell_path = workcell_path;
  link_window.setModal(true);
  link_window.setWindowTitle("New Link");
  link_window.exec();
  if (link_window.success) {
    ui->link_list->addItem(QString::fromStdString(link_window.link.name));
    object.link_vector.push_back(link_window.link);
    if (object.ext_joint.child_link_pos == -1) {
      object.ext_joint.child_link_pos = 0;
    }
  }
  bool oldState = ui->available_links->blockSignals(true);
  ui->available_links->clear();

  for (int link = 0; link < static_cast<int>(object.link_vector.size()); link++) {
    ui->available_links->addItem(QString::fromStdString(object.link_vector[link].name));
  }
  ui->available_links->setCurrentIndex(object.ext_joint.child_link_pos);
  ui->available_links->blockSignals(oldState);
}


void AddObject::on_delete_link_clicked()
{
  // TODO(Glenn): Check joints and if there is any joints with the link dependant on it,
  // then delete that joint
  auto list = ui->link_list->selectionModel()->selectedIndexes();
  if (list.size() != 0) {
    QListWidgetItem * item = ui->link_list->selectedItems().first();
    int pos = ui->link_list->row(item);

    for (int joint = 0; joint < static_cast<int>(object.joint_vector.size()); joint++) {
      if (object.joint_vector[joint].parent_link.name.compare(object.link_vector[pos].name) == 0 ||
        object.joint_vector[joint].child_link.name.compare(object.link_vector[pos].name) == 0)
      {
        object.joint_vector.erase(object.joint_vector.begin() + joint);
        delete ui->joint_list->takeItem(joint);
        delete ui->parent_list->takeItem(joint);
        delete ui->child_list->takeItem(joint);
      }
    }

    bool oldState = ui->available_links->blockSignals(true);
    delete ui->link_list->takeItem(ui->link_list->row(item));
    ui->available_links->removeItem(pos);
    // If child link position is more than deleted link
    if (object.ext_joint.child_link_pos > pos) {
      object.ext_joint.child_link_pos--;       // shift down position by 1
    }
    if (object.link_vector.size() > 1) {
      object.link_vector.erase(object.link_vector.begin() + list[0].row());
      // The child link was not deleted
      if (pos != object.ext_joint.child_link_pos) {
        ui->available_links->setCurrentIndex(object.ext_joint.child_link_pos);
      } else {  // Child link deleted
        ui->available_links->setCurrentIndex(0);
      }
    } else {
      object.link_vector.clear();
    }
    ui->available_links->blockSignals(oldState);
  }
}

void AddObject::on_link_list_itemDoubleClicked(QListWidgetItem * item)
{
  int pos = ui->link_list->row(item);
  AddLink link_window;
  std::vector<std::string> available_link_names;
  for (int i = 0; i < static_cast<int>(object.link_vector.size()); i++) {
    if (i != pos) {
      available_link_names.push_back(object.link_vector[i].name);
    }
  }
  link_window.available_link_names = available_link_names;
  link_window.load_link(object.link_vector[pos]);
  link_window.setModal(true);
  link_window.setWindowTitle("Edit Link");
  link_window.exec();
  object.link_vector[pos] = link_window.link;
  item->setText(QString::fromStdString(link_window.link.name));
}

void AddObject::on_AddJoint_clicked()
{
  if (object.link_vector.size() > 1) {
    AddJoint joint_window;
    joint_window.available_joints = object.joint_vector;
    joint_window.available_links = object.link_vector;
    joint_window.LoadLinks();
    joint_window.setModal(true);
    joint_window.setWindowTitle("New Joint");
    joint_window.exec();
    if (joint_window.success) {
      ui->joint_list->addItem(QString::fromStdString(joint_window.joint.name));
      ui->parent_list->addItem(QString::fromStdString(joint_window.joint.parent_link.name));
      ui->child_list->addItem(QString::fromStdString(joint_window.joint.child_link.name));
      object.joint_vector.push_back(joint_window.joint);
    }
  } else {
    ui->error_check->setText("Not enough Links to create a Joint. You need at least 2.");
  }
}

void AddObject::on_joint_list_itemDoubleClicked(QListWidgetItem * item)
{
  AddJoint joint_window;
  int pos = ui->joint_list->row(item);
  joint_window.edit_pos = pos;
  joint_window.available_links = object.link_vector;
  joint_window.available_joints = object.joint_vector;
  joint_window.LoadLinks();
  joint_window.load_joint(object.joint_vector[pos]);
  joint_window.joint = object.joint_vector[pos];
  joint_window.setModal(true);
  joint_window.setWindowTitle("Edit Joint");
  joint_window.exec();

  object.joint_vector[pos] = joint_window.joint;
  item->setText(QString::fromStdString(joint_window.joint.name));
  ui->child_list->item(pos)->setText(QString::fromStdString(joint_window.joint.child_link.name));
  ui->parent_list->item(pos)->setText(QString::fromStdString(joint_window.joint.parent_link.name));
}


void AddObject::on_DeleteJoint_clicked()
{
  auto list = ui->joint_list->selectionModel()->selectedIndexes();
  if (list.size() != 0) {
    object.joint_vector.erase(object.joint_vector.begin() + list[0].row());

    QListWidgetItem * item = ui->joint_list->selectedItems().first();
    delete ui->joint_list->takeItem(ui->joint_list->row(item));

    QListWidgetItem * parent = ui->parent_list->takeItem(list[0].row());
    delete ui->parent_list->takeItem(ui->parent_list->row(parent));

    QListWidgetItem * child = ui->child_list->takeItem(list[0].row());
    delete ui->child_list->takeItem(ui->child_list->row(child));
  }
}

void AddObject::on_pushButton_clicked()
{
  if (ErrorObjectName() == false) {
    if (object.link_vector.size() != 0) {
      object.ext_joint.name = object.name + "_base_joint";
      object.ext_joint.child_object = object.name;
      object.ext_joint.type = ui->ext_joint_type->currentText().toStdString();
      object.ext_joint.child_link_pos = ui->available_links->currentIndex();

      // Once name is confirmed, edit the workcell path again.
      for (int i = 0; i < static_cast<int>(object.link_vector.size()); i++) {
        if (object.link_vector[i].is_visual) {
          if (object.link_vector[i].visual_vector[0].geometry.is_stl) {
            std::string old_filepath_new =
              object.link_vector[i].visual_vector[0].geometry.filepath_new;
            std::string first_half;
            std::string mid_half = "/" + object.name + "_description";
            std::string second_half;
            int slash_count = 0;
            for (auto it = old_filepath_new.crbegin(); it != old_filepath_new.crend(); ++it) {
              if (slash_count < 3) {
                second_half = std::string(1, *it) + second_half;
              }

              if (slash_count > 3) {
                first_half = std::string(1, *it) + first_half;
              }
              if (*it == '/') {
                slash_count++;
              }
            }
            object.link_vector[i].visual_vector[0].geometry.filepath_new = first_half + mid_half +
              second_half;
          }
        }
        if (object.link_vector[i].is_collision) {
          if (object.link_vector[i].collision_vector[0].geometry.is_stl) {
            std::string old_filepath_new =
              object.link_vector[i].collision_vector[0].geometry.filepath_new;
            std::string first_half;
            std::string mid_half = "/" + object.name + "_description";
            std::string second_half;
            int slash_count = 0;
            for (auto it = old_filepath_new.crbegin(); it != old_filepath_new.crend(); ++it) {
              if (slash_count < 3) {
                second_half = std::string(1, *it) + second_half;
              }

              if (slash_count > 3) {
                first_half = std::string(1, *it) + first_half;
              }
              if (*it == '/') {
                slash_count++;
              }
            }
            object.link_vector[i].collision_vector[0].geometry.filepath_new = first_half +
              mid_half + second_half;
          }
        }
      }
      success = true;
      this->close();
    } else {
      ui->error_check->setText("No links available. An object should have at least 1 link.");
    }
  }
}

void AddObject::on_pushButton_2_clicked()
{
  success = false;
  this->close();
}

void AddObject::on_lineEdit_textChanged(const QString & arg1)
{
  object.name = arg1.toStdString();
}


bool AddObject::ErrorObjectName()
{
  if (!editing_mode) {
    for (std::string name : available_object_names) {
      if (name.compare(object.name) == 0) {
        ui->error_check->setText("Name Error: Object of that name already exists");
        return true;
      }
    }
  }

  if (object.name.empty()) {
    ui->error_check->setText("Name Error: Input a name for this object");
    return true;
  }
  if (object.name.compare("cylinder") == 0 || object.name.compare("box") == 0 ||
    object.name.compare("sphere") == 0)
  {
    ui->error_check->setText(
      QString::fromStdString(
        "Name Error: " + object.name +
        " is a reserved name. Please use another name for your object "));
    return true;
  }
  std::string initial_name = object.name;
  std::string final_name;
  if (initial_name.find_first_not_of(' ') != std::string::npos) {
    for(int i = 0 ; i < static_cast<int>(initial_name.length()); i++){
        if(isspace(initial_name[i])){
            final_name += "_";
        }
        else{
            final_name += std::tolower(initial_name[i]);
        }
    }
    object.name = final_name;
  }
  return false;
}

void AddObject::LoadObject(Object input_object)
{
  editing_mode = true;
  success = true;

  ui->lineEdit->setText(QString::fromStdString(input_object.name));
  object = input_object;
  int pos = 0;
  for (int j = 0; j < static_cast<int>(input_object.link_vector.size()); j++) {
    if (object.link_vector[j].name.compare(object.link_vector[object.ext_joint.child_link_pos].name)
      ==
      0)
    {
      pos = j;
    }
    ui->link_list->addItem(QString::fromStdString(object.link_vector[j].name));
    ui->available_links->addItem(QString::fromStdString(object.link_vector[j].name));
  }
  ui->available_links->setCurrentIndex(pos);

  for (int i = 0; i < static_cast<int>(input_object.joint_vector.size()); i++) {
    ui->joint_list->addItem(QString::fromStdString(object.joint_vector[i].name));
    ui->parent_list->addItem(QString::fromStdString(object.joint_vector[i].parent_link.name));
    ui->child_list->addItem(QString::fromStdString(object.joint_vector[i].child_link.name));
  }

  if (input_object.ext_joint.type.compare("fixed") == 0) {
    ui->ext_joint_type->setCurrentIndex(0);
  } else if (input_object.ext_joint.type.compare("revolute") == 0) {
    ui->ext_joint_type->setCurrentIndex(1);
  } else if (input_object.ext_joint.type.compare("continuous") == 0) {
    ui->ext_joint_type->setCurrentIndex(2);
  } else if (input_object.ext_joint.type.compare("prismatic") == 0) {
    ui->ext_joint_type->setCurrentIndex(3);
  } else if (input_object.ext_joint.type.compare("floating") == 0) {
    ui->ext_joint_type->setCurrentIndex(4);
  } else if (input_object.ext_joint.type.compare("planar") == 0) {
    ui->ext_joint_type->setCurrentIndex(5);
  }
}

void AddObject::on_available_links_currentIndexChanged(int index)
{
  object.ext_joint.child_link_pos = index;
}

void AddObject::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}
