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

#include "gui/new_scene.h"
#include <QFileDialog>
#include <boost/filesystem.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

#include "gui/addexternaljoint.h"
#include "gui/addobject.h"
#include "gui/ui_new_scene.h"


NewScene::NewScene(QWidget * parent)
: QDialog(parent),
  ui(new Ui::NewScene)
{
  ui->setupUi(this);
  on_enable_robot_stateChanged(0);
  on_enable_ee_stateChanged(0);
  success = false;
}

NewScene::~NewScene()
{
  delete ui;
}

void NewScene::on_add_object_clicked()
{
  AddObject object_window;
  object_window.setWindowTitle("Add New Environmental Object");
  object_window.setModal(true);
  object_window.exec();

  if (object_window.success) {
    ui->object_list->addItem(QString::fromStdString(object_window.object.name));
    // visual_list.push_back(visual_window);
    gui_environment.environment.object_vector.push_back(object_window.object);
    gui_environment.object_names.push_back(object_window.object.name);
    std::vector<std::string> link_vec;
    for (int i = 0; i < static_cast<int>(object_window.object.link_vector.size()); i++) {
      link_vec.push_back(object_window.object.link_vector[i].name);
    }

    if (link_vec.size() == 0) {
      link_vec.push_back("");
    }
    gui_environment.object_links.push_back(link_vec);

    std::vector<std::string> joint_vec;
    for (int i = 0; i < static_cast<int>(object_window.object.joint_vector.size()); i++) {
      joint_vec.push_back(object_window.object.joint_vector[i].name);
    }
    if (joint_vec.size() == 0) {
      joint_vec.push_back("");
    }
    gui_environment.object_joints.push_back(joint_vec);
  }
}

void NewScene::on_object_list_itemDoubleClicked(QListWidgetItem * item)
{
  int pos = ui->object_list->row(item);
  AddObject object_window;
  object_window.setModal(true);
  object_window.setWindowTitle("Edit Object");
  object_window.LoadObject(gui_environment.environment.object_vector[pos]);

  object_window.link_names = gui_environment.object_links[pos];

  object_window.joint_names = gui_environment.object_joints[pos];
  object_window.available_object_names = gui_environment.object_names;
  object_window.object = gui_environment.environment.object_vector[pos];

  object_window.exec();


  gui_environment.environment.object_vector[pos] = object_window.object;
  gui_environment.object_names[pos] = object_window.object.name;
  std::vector<std::string> link_vec;
  for (int i = 0; i < static_cast<int>(object_window.object.link_vector.size()); i++) {
    link_vec.push_back(object_window.object.link_vector[i].name);
  }
  gui_environment.object_links[pos] = link_vec;
  item->setText(QString::fromStdString(object_window.object.name));
}

std::vector<std::string> NewScene::GetLinks(std::string filename)
{
  std::vector<std::string> links;

  std::ifstream infile(filename);
  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream ss(line);
    ss >> std::ws;
    std::string temp_string;
    temp_string.resize(11);
    ss.read(&temp_string[0], 11);
    if (std::strcmp(temp_string.c_str(), "<link name=") == 0) {
      remove_if(line.begin(), line.end(), isspace);
      line.erase(0, line.find("\"") + 1);
      line.erase(line.begin() + line.find("\""), line.end());
      line.erase(0, line.find("}") + 1);
      links.push_back(line);
    }
  }
  return links;
}

void NewScene::add_desc_links(QString OutputFolder, int ee_or_robot)  // 0 for robot, 1 for ee
{
  int error = 0;
  boost::filesystem::current_path(OutputFolder.toStdString());
  boost::filesystem::path urdf = "urdf";
  std::string object_name;
  for (auto it = OutputFolder.toStdString().crbegin(); it != OutputFolder.toStdString().crend();
    ++it)
  {
    if (*it != '/') {
      object_name = std::string(1, *it) + object_name;
    } else {
      break;
    }
  }
  if (object_name.find("_description") > object_name.length()) {
    if (ee_or_robot == 0) {
      ui->robot_desc_error->setText(
        "<font color='red'>Folder error: Please provide the filepath to"
        " the robot description folder</font>");
    } else {
      ui->ee_desc_error->setText(
        "<font color='red'>Folder error: Please provide the filepath to"
        " the End Effector description folder</font>");
    }
    return;
  } else {
    object_name.erase(object_name.find("_description"), 12);
  }


  if (!boost::filesystem::exists(urdf) ) {
    if (ee_or_robot == 0) {
      ui->robot_desc_error->setText("<font color='red'>Folder error: No urdf folder exists</font>");
    } else {
      ui->ee_desc_error->setText("<font color='red'>Folder error: No urdf folder exists</font>");
    }
    return;
  } else {
    boost::filesystem::current_path("urdf");
    std::string urdf_filename = object_name + ".urdf.xacro";
    if (!boost::filesystem::exists(urdf_filename) ) {
      if (ee_or_robot == 0) {
        ui->robot_desc_error->setText("<font color='red'>Error! no URDF XACRO file exists</font>");
      } else {
        ui->ee_desc_error->setText("<font color='red'>Error! no URDF XACRO file exists</font>");
      }
      return;
    } else {
//         robot_links = GetLinks(urdf_filename);
      for (int i = 0; i < static_cast<int>(gui_environment.object_names.size()); i++) {
        if (strcmp(gui_environment.object_names[i].c_str(), object_name.c_str()) == 0) {
          error++;
        }
      }
      if (error == 0) {
        if (ee_or_robot == 0) {
          gui_environment.environment.robot_vector.clear();
          Robot temp_robot;
          temp_robot.name = object_name;
          gui_environment.environment.robot_vector.push_back(temp_robot);
          gui_environment.robot_loaded = true;
          ErrorCheckOrigin(0);
          ui->robot_x->setDisabled(false);
          ui->robot_x_label->setDisabled(false);
          ui->robot_y->setDisabled(false);
          ui->robot_y_label->setDisabled(false);
          ui->robot_z->setDisabled(false);
          ui->robot_z_label->setDisabled(false);
          ui->robot_roll->setDisabled(false);
          ui->robot_roll_label->setDisabled(false);
          ui->robot_pitch->setDisabled(false);
          ui->robot_pitch_label->setDisabled(false);
          ui->robot_yaw->setDisabled(false);
          ui->robot_yaw_label->setDisabled(false);
          gui_environment.robot_names.push_back(object_name);
          gui_environment.robot_links.push_back(GetLinks(urdf_filename));
        } else {
          gui_environment.environment.ee_vector.clear();
          EndEffector temp_ee;
          temp_ee.name = object_name;
          gui_environment.environment.ee_vector.push_back(temp_ee);
          gui_environment.ee_loaded = true;
          ErrorCheckOrigin(1);
          ui->ee_x->setDisabled(false);
          ui->ee_x_label->setDisabled(false);
          ui->ee_y->setDisabled(false);
          ui->ee_y_label->setDisabled(false);
          ui->ee_z->setDisabled(false);
          ui->ee_z_label->setDisabled(false);
          ui->ee_roll->setDisabled(false);
          ui->ee_roll_label->setDisabled(false);
          ui->ee_pitch->setDisabled(false);
          ui->ee_pitch_label->setDisabled(false);
          ui->ee_yaw->setDisabled(false);
          ui->ee_yaw_label->setDisabled(false);
          gui_environment.ee_names.push_back(object_name);
          gui_environment.ee_links.push_back(GetLinks(urdf_filename));
        }
      }
    }
  }
}

void NewScene::on_load_robot_desc_clicked()
{
  QString OutputFolder;
  OutputFolder = QFileDialog::getExistingDirectory(
    0, ("Select Robot Description Folder"),
    QDir::currentPath());
  if (!OutputFolder.isEmpty()) {
    ui->robot_desc_filepath->setText(OutputFolder);
    gui_environment.environment.ee_vector[0].filepath = OutputFolder.toStdString();
    add_desc_links(OutputFolder, 0);
  }
}

void NewScene::on_load_ee_desc_clicked()
{
  QString OutputFolder =
    QFileDialog::getExistingDirectory(
    0, ("Select End Effector Description Folder"),
    QDir::currentPath());
  if (!OutputFolder.isEmpty()) {
    ui->ee_desc_filepath->setText(OutputFolder);
    gui_environment.environment.robot_vector[0].filepath = OutputFolder.toStdString();
    add_desc_links(OutputFolder, 1);
  }
}

void NewScene::on_add_ext_joint_clicked()
{
  int total_objects = gui_environment.object_names.size() + gui_environment.robot_names.size() +
    gui_environment.ee_names.size();
  if (total_objects > 1) {
    AddExternalJoint external_joint_window;
    external_joint_window.setWindowTitle("Add New External Joint");
    external_joint_window.setModal(true);
    std::vector<std::string> combine_names;

    combine_names.insert(
      combine_names.end(),
      gui_environment.robot_names.begin(), gui_environment.robot_names.end() );
    combine_names.insert(
      combine_names.end(),
      gui_environment.ee_names.begin(), gui_environment.ee_names.end() );
    combine_names.insert(
      combine_names.end(),
      gui_environment.object_names.begin(), gui_environment.object_names.end() );

    std::vector<std::vector<std::string>> combine_links;
    combine_links.insert(
      combine_links.end(),
      gui_environment.robot_links.begin(), gui_environment.robot_links.end());
    combine_links.insert(
      combine_links.end(),
      gui_environment.ee_links.begin(), gui_environment.ee_links.end());
    combine_links.insert(
      combine_links.end(),
      gui_environment.object_links.begin(), gui_environment.object_links.end());

    external_joint_window.available_links = combine_links;
    external_joint_window.available_objects = combine_names;
    external_joint_window.available_joint_names = gui_environment.ext_joint_names;

    external_joint_window.LoadObjects();

    external_joint_window.exec();

    if (external_joint_window.success) {
      ui->parent_link_list->addItem(
        QString::fromStdString(external_joint_window.joint.parent_link));
      ui->parent_obj_list->addItem(QString::fromStdString(external_joint_window.parent_object_str));
      ui->child_obj_list->addItem(QString::fromStdString(external_joint_window.child_object_str));
      ui->child_link_list->addItem(QString::fromStdString(external_joint_window.joint.child_link));
      ui->ext_joint_list->addItem(QString::fromStdString(external_joint_window.joint.name));

      gui_environment.ext_joint_names.push_back(external_joint_window.joint.name);
      gui_environment.environment.ext_joint_vector.push_back(external_joint_window.joint);
      gui_environment.child_objects.push_back(external_joint_window.child_object_str);
      gui_environment.parent_objects.push_back(external_joint_window.parent_object_str);
    }
  } else {
    ui->ext_joint_error->setText(
      " <font color='red'>Error: You need at least 2 Objects to create an external joint</font>");
  }
}

void NewScene::on_delete_object_clicked()
{
  auto list = ui->object_list->selectionModel()->selectedIndexes();
  if (list.size() != 0) {
    gui_environment.environment.object_vector.erase(
      gui_environment.environment.object_vector.begin() + list[0].row());
    QListWidgetItem * item = ui->object_list->selectedItems().first();
    delete ui->object_list->takeItem(ui->object_list->row(item));
    gui_environment.object_names.erase(gui_environment.object_names.begin() + list[0].row());
    gui_environment.object_links.erase(gui_environment.object_links.begin() + list[0].row());
    gui_environment.object_joints.erase(gui_environment.object_joints.begin() + list[0].row());
  }
}

void NewScene::on_ext_joint_list_itemDoubleClicked(QListWidgetItem * item)
{
  int pos = ui->ext_joint_list->row(item);
  AddExternalJoint external_joint_window;
  external_joint_window.setWindowTitle("Add New Environmental Object");
  external_joint_window.setModal(true);

  external_joint_window.available_links = gui_environment.object_links;
  external_joint_window.available_objects = gui_environment.object_names;
  external_joint_window.available_joint_names = gui_environment.ext_joint_names;

  external_joint_window.LoadObjects();

  external_joint_window.child_object_str = gui_environment.child_objects[pos];
  external_joint_window.parent_object_str = gui_environment.parent_objects[pos];

  external_joint_is_origin.LoadExternalJoint(gui_environment.environment.ext_joint_vector[pos]);


  external_joint_window.exec();

  ui->parent_link_list->item(pos)->setText(
    QString::fromStdString(
      external_joint_window.joint.
      parent_link));
  ui->parent_obj_list->item(pos)->setText(
    QString::fromStdString(
      external_joint_window.
      parent_object_str));
  ui->child_obj_list->item(pos)->setText(
    QString::fromStdString(
      external_joint_window.
      child_object_str));
  ui->child_link_list->item(pos)->setText(
    QString::fromStdString(
      external_joint_window.joint.
      child_link));
  ui->ext_joint_list->item(pos)->setText(QString::fromStdString(external_joint_window.joint.name));

  gui_environment.ext_joint_names[pos] = external_joint_window.joint.name;
  gui_environment.environment.ext_joint_vector[pos] = external_joint_window.joint;
}

void NewScene::on_del_ext_joint_clicked()
{
  auto list = ui->ext_joint_list->selectionModel()->selectedIndexes();
  if (list.size() != 0) {
    gui_environment.environment.ext_joint_vector.erase(
      gui_environment.environment.ext_joint_vector.begin() + list[0].row());

    QListWidgetItem * item = ui->ext_joint_list->selectedItems().first();
    delete ui->ext_joint_list->takeItem(ui->ext_joint_list->row(item));

    QListWidgetItem * parent = ui->parent_obj_list->takeItem(list[0].row());
    delete ui->parent_obj_list->takeItem(ui->parent_obj_list->row(parent));

    QListWidgetItem * parent_link = ui->parent_link_list->takeItem(list[0].row());
    delete ui->parent_link_list->takeItem(ui->parent_link_list->row(parent_link));

    QListWidgetItem * child = ui->child_obj_list->takeItem(list[0].row());
    delete ui->child_obj_list->takeItem(ui->child_obj_list->row(child));

    QListWidgetItem * child_link = ui->child_link_list->takeItem(list[0].row());
    delete ui->child_link_list->takeItem(ui->child_link_list->row(child_link));

    gui_environment.ext_joint_names.erase(gui_environment.ext_joint_names.begin() + list[0].row());
    gui_environment.parent_objects.erase(gui_environment.parent_objects.begin() + list[0].row());
    gui_environment.child_objects.erase(gui_environment.child_objects.begin() + list[0].row());
  }
}

int NewScene::ErrorCheckOrigin(int robot_or_ee)
{
  int num_errors = 0;
  if (robot_or_ee == 0) {
    if (ui->robot_x->text().isEmpty() || ui->robot_y->text().isEmpty() ||
      ui->robot_z->text().isEmpty() || ui->robot_roll->text().isEmpty() ||
      ui->robot_pitch->text().isEmpty() || ui->robot_yaw->text().isEmpty())
    {
      ui->robot_desc_error->setText(
        " <font color='red'>Error: XYZ or RPY values for Robot not completely filled.</font>");
      num_errors++;
    } else {
      int i = 0;
      auto float_validator = new QDoubleValidator();
      QString input_x = ui->robot_x->text();
      QString input_y = ui->robot_y->text();
      QString input_z = ui->robot_z->text();

      QString input_roll = ui->robot_roll->text();
      QString input_pitch = ui->robot_pitch->text();
      QString input_yaw = ui->robot_yaw->text();

      if (float_validator->validate(
          input_x,
          i) != QValidator::Acceptable ||
        float_validator->validate(
          input_y,
          i) != QValidator::Acceptable ||
        float_validator->validate(
          input_z,
          i) != QValidator::Acceptable ||
        float_validator->validate(
          input_roll,
          i) != QValidator::Acceptable ||
        float_validator->validate(
          input_pitch,
          i) != QValidator::Acceptable ||
        float_validator->validate(input_yaw, i) != QValidator::Acceptable)
      {
        ui->robot_desc_error->setText(
          " <font color='red'>Type Error: Robot XYZ and RPY need to be floats</font>");
        num_errors++;
      } else {
        gui_environment.environment.robot_vector[0].origin.is_origin = true;
        gui_environment.environment.robot_vector[0].origin.x = input_x.toFloat();
        gui_environment.environment.robot_vector[0].origin.y = input_y.toFloat();
        gui_environment.environment.robot_vector[0].origin.z = input_z.toFloat();

        gui_environment.environment.robot_vector[0].origin.roll = input_roll.toFloat();
        gui_environment.environment.robot_vector[0].origin.pitch = input_pitch.toFloat();
        gui_environment.environment.robot_vector[0].origin.yaw = input_yaw.toFloat();
      }
    }
    if (num_errors == 0) {
      ui->robot_desc_error->setText(" <font color='green'> No Errors </font>");
    }
  } else {
    if (ui->ee_x->text().isEmpty() || ui->ee_y->text().isEmpty() || ui->ee_z->text().isEmpty() ||
      ui->ee_roll->text().isEmpty() || ui->ee_pitch->text().isEmpty() ||
      ui->ee_yaw->text().isEmpty())
    {
      ui->ee_desc_error->setText(
        " <font color='red'>Error: XYZ or RPY values for"
        " End Effector not completely filled.</font>");
      num_errors++;
    } else {
      int i = 0;
      auto float_validator = new QDoubleValidator();
      QString input_x = ui->ee_x->text();
      QString input_y = ui->ee_y->text();
      QString input_z = ui->ee_z->text();

      QString input_roll = ui->ee_roll->text();
      QString input_pitch = ui->ee_pitch->text();
      QString input_yaw = ui->ee_yaw->text();

      if (float_validator->validate(
          input_x,
          i) != QValidator::Acceptable ||
        float_validator->validate(
          input_y,
          i) != QValidator::Acceptable ||
        float_validator->validate(
          input_z,
          i) != QValidator::Acceptable ||
        float_validator->validate(
          input_roll,
          i) != QValidator::Acceptable ||
        float_validator->validate(
          input_pitch,
          i) != QValidator::Acceptable ||
        float_validator->validate(input_yaw, i) != QValidator::Acceptable)
      {
        ui->ee_desc_error->setText(
          " <font color='red'>Type Error: End Effector XYZ and RPY need to be floats</font>");
        num_errors++;
      } else {
        gui_environment.environment.ee_vector[0].origin.is_origin = true;
        gui_environment.environment.ee_vector[0].origin.x = input_x.toFloat();
        gui_environment.environment.ee_vector[0].origin.y = input_y.toFloat();
        gui_environment.environment.ee_vector[0].origin.z = input_z.toFloat();

        gui_environment.environment.ee_vector[0].origin.roll = input_roll.toFloat();
        gui_environment.environment.ee_vector[0].origin.pitch = input_pitch.toFloat();
        gui_environment.environment.ee_vector[0].origin.yaw = input_yaw.toFloat();
      }
    }

    if (num_errors == 0) {
      ui->ee_desc_error->setText(" <font color='green'> No Errors </font>");
    }
  }
  return num_errors;
}

void NewScene::on_robot_x_textChanged(const QString & arg1)
{
  ErrorCheckOrigin(0);
}

void NewScene::on_robot_y_textChanged(const QString & arg1)
{
  ErrorCheckOrigin(0);
}

void NewScene::on_robot_z_textChanged(const QString & arg1)
{
  ErrorCheckOrigin(0);
}

void NewScene::on_robot_roll_textChanged(const QString & arg1)
{
  ErrorCheckOrigin(0);
}

void NewScene::on_robot_pitch_textChanged(const QString & arg1)
{
  ErrorCheckOrigin(0);
}

void NewScene::on_robot_yaw_textChanged(const QString & arg1)
{
  ErrorCheckOrigin(0);
}

void NewScene::on_ee_x_textChanged(const QString & arg1)
{
  ErrorCheckOrigin(1);
}

void NewScene::on_ee_y_textChanged(const QString & arg1)
{
  ErrorCheckOrigin(1);
}

void NewScene::on_ee_z_textChanged(const QString & arg1)
{
  ErrorCheckOrigin(1);
}

void NewScene::on_ee_roll_textChanged(const QString & arg1)
{
  ErrorCheckOrigin(1);
}

void NewScene::on_ee_pitch_textChanged(const QString & arg1)
{
  ErrorCheckOrigin(1);
}

void NewScene::on_ee_yaw_textChanged(const QString & arg1)
{
  ErrorCheckOrigin(1);
}

void NewScene::on_enable_robot_stateChanged(int arg1)
{
  if (arg1 == 0) {
    ui->robot_desc_label->setDisabled(true);
    ui->load_robot_desc->setDisabled(true);
    ui->robot_desc_error->setDisabled(true);
    ui->robot_desc_error->setText("Robot Disabled");
    ui->robot_desc_filepath->setDisabled(true);
    ui->robot_desc_filepath->clear();
    gui_environment.robot_loaded = false;
    gui_environment.environment.robot_vector.clear();

    ui->robot_x->setDisabled(true);
    ui->robot_x_label->setDisabled(true);
    ui->robot_y->setDisabled(true);
    ui->robot_y_label->setDisabled(true);
    ui->robot_z->setDisabled(true);
    ui->robot_z_label->setDisabled(true);
    ui->robot_roll->setDisabled(true);
    ui->robot_roll_label->setDisabled(true);
    ui->robot_pitch->setDisabled(true);
    ui->robot_pitch_label->setDisabled(true);
    ui->robot_yaw->setDisabled(true);
    ui->robot_yaw_label->setDisabled(true);

  } else {
    ui->robot_desc_label->setDisabled(false);
    ui->load_robot_desc->setDisabled(false);
    ui->robot_desc_error->setDisabled(false);
    ui->robot_desc_error->setText("Please Load the Robot Description Folder");
    ui->robot_desc_filepath->setDisabled(false);
  }
}

void NewScene::on_enable_ee_stateChanged(int arg1)
{
  if (arg1 == 0) {
    ui->ee_desc_error->setDisabled(true);
    ui->ee_desc_error->setText("End Effector Disabled");
    ui->ee_desc_filepath->setDisabled(true);
    ui->ee_desc_filepath->clear();
    ui->load_ee_desc->setDisabled(true);
    ui->ee_desc_label->setDisabled(true);
    gui_environment.ee_loaded = false;
    gui_environment.environment.ee_vector.clear();

    ui->ee_x->setDisabled(true);
    ui->ee_x_label->setDisabled(true);
    ui->ee_y->setDisabled(true);
    ui->ee_y_label->setDisabled(true);
    ui->ee_z->setDisabled(true);
    ui->ee_z_label->setDisabled(true);
    ui->ee_roll->setDisabled(true);
    ui->ee_roll_label->setDisabled(true);
    ui->ee_pitch->setDisabled(true);
    ui->ee_pitch_label->setDisabled(true);
    ui->ee_yaw->setDisabled(true);
    ui->ee_yaw_label->setDisabled(true);
  } else {
    ui->ee_desc_error->setDisabled(false);
    ui->ee_desc_error->setText("Please Load the End Effector Description Folder");
    ui->ee_desc_filepath->setDisabled(false);
    ui->load_ee_desc->setDisabled(false);
    ui->ee_desc_label->setDisabled(false);
  }
}

void NewScene::on_create_environment_clicked()
{
  int errors = 0;
  ui->errorlist->clear();
  if (gui_environment.robot_loaded) {
    if (ErrorCheckOrigin(0) != 0) {
      ui->errorlist->append("Robot Origin error: Cannot Generate YAML file");
      errors++;
    }
  }
  if (gui_environment.ee_loaded) {
    if (ErrorCheckOrigin(1) != 0) {
      ui->errorlist->append("End Effector Origin error: Cannot Generate YAML file");
      errors++;
    }
  }
  if (errors == 0) {
    success = true;
    this->close();
  }
}

void NewScene::LoadEnvironment(GUIEnvironment input_environment)
{
  if (input_environment.robot_loaded) {
    on_enable_robot_stateChanged(1);

    if (input_environment.environment.robot_vector.size() != 0) {
      ui->robot_desc_label->setText(
        QString::fromStdString(
          input_environment.environment.
          robot_vector[0].filepath));
      ui->robot_x->setText(QString::number(input_environment.environment.robot_vector[0].origin.x));
      ui->robot_y->setText(QString::number(input_environment.environment.robot_vector[0].origin.y));
      ui->robot_z->setText(QString::number(input_environment.environment.robot_vector[0].origin.z));
      ui->robot_roll->setText(
        QString::number(
          input_environment.environment.robot_vector[0].origin.
          roll));
      ui->robot_pitch->setText(
        QString::number(
          input_environment.environment.robot_vector[0].origin.
          pitch));
      ui->robot_yaw->setText(
        QString::number(
          input_environment.environment.robot_vector[0].origin.
          yaw));
    }
  } else {
    on_enable_robot_stateChanged(0);
  }
  if (input_environment.ee_loaded) {
    on_enable_ee_stateChanged(1);
    if (input_environment.environment.ee_vector.size() != 0) {
      ui->ee_desc_label->setText(
        QString::fromStdString(
          input_environment.environment.ee_vector[0].
          filepath));
      ui->ee_x->setText(QString::number(input_environment.environment.ee_vector[0].origin.x));
      ui->ee_y->setText(QString::number(input_environment.environment.ee_vector[0].origin.y));
      ui->ee_z->setText(QString::number(input_environment.environment.ee_vector[0].origin.z));
      ui->ee_roll->setText(
        QString::number(input_environment.environment.ee_vector[0].origin.roll));
      ui->ee_pitch->setText(
        QString::number(input_environment.environment.ee_vector[0].origin.pitch));
      ui->ee_yaw->setText(QString::number(input_environment.environment.ee_vector[0].origin.yaw));
    }
  } else {
    on_enable_ee_stateChanged(0);
  }

  if (input_environment.environment.object_vector.size() != 0) {
    for (int i = 0; i < input_environment.object_names.size(); i++) {
      ui->object_list->addItem(QString::fromStdString(input_environment.object_names[i]));
    }
  }
  if (input_environment.environment.ext_joint_vector.size() != 0) {
    if (input_environment.parent_objects.size() == input_environment.child_objects.size() &&
      input_environment.parent_objects.size() == input_environment.ext_joint_names.size())
    {
      for (int j = 0; j < input_environment.ext_joint_names.size(); j++) {
        ui->ext_joint_list->addItem(QString::fromStdString(input_environment.ext_joint_names[j]));
        ui->child_obj_list->addItem(QString::fromStdString(input_environment.child_objects[j]));
        ui->child_link_list->addItem(
          QString::fromStdString(
            input_environment.environment.
            ext_joint_vector[j].child_link));
        ui->parent_obj_list->addItem(QString::fromStdString(input_environment.parent_objects[j]));
        ui->parent_link_list->addItem(
          QString::fromStdString(
            input_environment.environment.
            ext_joint_vector[j].parent_link));
      }
    }
  }
  gui_environment = input_environment;
}
