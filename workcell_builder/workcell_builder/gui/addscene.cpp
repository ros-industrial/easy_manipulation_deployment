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

#include <QFileDialog>
#include <QKeyEvent>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>

#include "gui/addscene.h"
#include "gui/ui_addscene.h"
#include "gui/addobject.h"
#include "gui/addexternaljoint.h"
#include "gui/addrobot.h"
#include "gui/addendeffector.h"
#include "gui/loadobjects.h"


AddScene::AddScene(QWidget * parent)
: QDialog(parent),
  ui(new Ui::AddScene)
{
  ui->setupUi(this);
  world_link.name = "world";
  on_include_ee_stateChanged(0);
  on_include_robot_stateChanged(0);
  success = false;

  scenes_path = boost::filesystem::current_path();
  boost::filesystem::current_path(scenes_path.branch_path());
  boost::filesystem::current_path("assets");
  assets_path = boost::filesystem::current_path();
  boost::filesystem::current_path(scenes_path);
}

AddScene::~AddScene()
{
  delete ui;
}

void AddScene::on_add_object_clicked()
{
  AddObject object_window;
  object_window.setWindowTitle("Add New Environmental Object");
  object_window.setModal(true);
  object_window.workcell_path = workcell_path;
  object_window.exec();

  if (object_window.success) {
    ui->object_list->addItem(QString::fromStdString(object_window.object.name));
    ui->object_list_2->addItem(QString::fromStdString(object_window.object.name));
    ui->parent_obj->addItem("");
    ui->parent_link->addItem("");
    scene.object_vector.push_back(object_window.object);
  }
  boost::filesystem::current_path(scenes_path);
}

void AddScene::on_object_list_itemDoubleClicked(QListWidgetItem * item)
{
  int pos = ui->object_list->row(item);
  AddObject object_window;
  object_window.setModal(true);
  object_window.setWindowTitle("Edit Object");
  object_window.LoadObject(scene.object_vector[pos]);
  object_window.workcell_path = workcell_path;

  std::vector<std::string> available_object_names;
  for (int i = 0; i < static_cast<int>(scene.object_vector.size()); i++) {
    if (pos != i) {
      available_object_names.push_back(scene.object_vector[i].name);
    }
  }
  object_window.available_object_names = available_object_names;
  object_window.object = scene.object_vector[pos];
  object_window.exec();
  if (object_window.success) {
    scene.object_vector[pos] = object_window.object;
    item->setText(QString::fromStdString(object_window.object.name));
    ui->object_list_2->item(pos)->setText(QString::fromStdString(object_window.object.name));
  }
  boost::filesystem::current_path(scenes_path);
}

void AddScene::on_include_robot_stateChanged(int arg1)
{
  if (arg1 == 0) {
    ui->robot_label->setDisabled(true);
    ui->robot_brand->clear();
    ui->robot_brand->setDisabled(true);
    ui->robot_brand_label->setDisabled(true);
    ui->robot_model->clear();
    ui->robot_model->setDisabled(true);
    ui->robot_model_label->setDisabled(true);
    ui->add_robot->setDisabled(true);
    ui->robot_connected->clear();
    ui->robot_connected->setDisabled(true);
    ui->remove_robot->setDisabled(true);
    ui->remove_ee->setDisabled(true);
    ui->robot_ee_check->clear();
    ui->robot_ee_check->setDisabled(true);

    ui->include_ee->stateChanged(0);
    ui->include_ee->setDisabled(true);
  } else {
    ui->robot_label->setDisabled(false);
    ui->robot_brand->setDisabled(false);
    ui->robot_brand_label->setDisabled(false);
    ui->robot_model->setDisabled(false);
    ui->robot_model_label->setDisabled(false);
    ui->add_robot->setDisabled(false);
    ui->robot_connected->setDisabled(false);
    ui->remove_robot->setDisabled(false);
    ui->robot_model->setText("<font color='red'> No model specified </font>");
    ui->robot_brand->setText("<font color='red'> No brand specified </font>");
    ui->robot_connected->setText("<font color='red'> Robot not conencted to world </font>");

    if (ui->include_ee->isChecked()) {
      ui->robot_ee_check->setText("<font color='red'> Robot and ee not linked </font>");
      ui->remove_ee->setDisabled(false);
      ui->robot_ee_check->setDisabled(false);
    }
  }
}

void AddScene::on_include_ee_stateChanged(int arg1)
{
  if (arg1 == 0) {
    ui->ee_label->setDisabled(true);
    ui->ee_brand->clear();
    ui->ee_brand->setDisabled(true);
    ui->ee_brand_label->setDisabled(true);
    ui->ee_model->clear();
    ui->ee_model->setDisabled(true);
    ui->ee_model_label->setDisabled(true);
    ui->add_ee->setDisabled(true);
    ui->remove_ee->setDisabled(true);
    ui->robot_ee_check->clear();
    ui->robot_ee_check->setDisabled(true);
  } else {
    ui->ee_label->setDisabled(false);
    ui->ee_brand->setDisabled(false);
    ui->ee_brand_label->setDisabled(false);
    ui->ee_model->setDisabled(false);
    ui->ee_model_label->setDisabled(false);
    ui->add_ee->setDisabled(false);
    ui->ee_model->setText("<font color='red'> No model specified </font>");
    ui->ee_brand->setText("<font color='red'> No brand specified </font>");
    if (ui->include_robot->isChecked()) {
      ui->robot_ee_check->setText("<font color='red'> Robot and ee not linked </font>");
      ui->remove_ee->setDisabled(false);
      ui->robot_ee_check->setDisabled(false);
    }
  }
}

void AddScene::on_object_list_2_itemDoubleClicked(QListWidgetItem * item)
{
  int pos = ui->object_list_2->row(item);  // Get position of selected object
  AddExternalJoint ext_object_window;
  ext_object_window.target_object = scene.object_vector[pos];  // load target object to window
  ext_object_window.available_objects.clear();
  ext_object_window.all_objects = scene.object_vector;  // Load all objects in scene

  std::vector<int> excluded_positions;
  excluded_positions.push_back(pos);   // You cannot be your own parent
  // Iterate through every object
  for (int object = 0; object < static_cast<int>(scene.object_vector.size()); object++) {
    if (object != pos) {  // dont have to check curr object
      std::vector<int> temp_excluded;
      int curr_obj = object;
      bool has_parent = true;       // current object has a parent that is not world
      bool has_pos = false;        // variable to check if target object is a ancestor of any object
      int counter = 0;       // counter to prevent inf while loops
      while (has_parent) {
        if (scene.object_vector[curr_obj].ext_joint.parent_obj_pos >= 0) {
          has_parent = true;
          temp_excluded.push_back(curr_obj);
          // The parent of the checked object is the target object
          if (scene.object_vector[curr_obj].ext_joint.parent_obj_pos == pos) {
            has_pos = true;             // This set of exclusion vector is relevant to target object
            break;
          }
          // Check the parent of the checked object
          curr_obj = scene.object_vector[curr_obj].ext_joint.parent_obj_pos;
        } else {
          has_parent = false;           // No parents, exit while loop
        }
        counter++;
        // In case some issues happen, exit while loop on max cases
        if (counter > static_cast<int>(scene.object_vector.size())) {
          break;
        }
      }
      // Add temp exclusions to excluded positions for this target
      if (has_pos) {
        excluded_positions.insert(
          excluded_positions.end(), temp_excluded.begin(),
          temp_excluded.end());
      }
    }
  }
  // Check the parent of each object, and the parent of the parent,
  // all the way to the top parent.
  //
  // If at any point the parent of the object is the target,
  // reject all objects including the curr object

  sort(excluded_positions.begin(), excluded_positions.end() );
  excluded_positions.erase(
    unique(
      excluded_positions.begin(),
      excluded_positions.end() ), excluded_positions.end() );

  std::vector<int> available_objects_index;
  for (int i = 0; i < static_cast<int>(scene.object_vector.size()); i++) {
    bool included = true;
    for (int i3 = 0; i3 < static_cast<int>(excluded_positions.size()); i3++) {
      if (i == excluded_positions[i3]) {
        included = false;
        break;
      }
    }
    if (included) {
      ext_object_window.available_objects.push_back(scene.object_vector[i]);
      available_objects_index.push_back(i);
    }
  }
  ext_object_window.setWindowTitle("Add New External Joint");
  ext_object_window.setModal(true);

  // If it is an empty ext joint
  if (ui->parent_obj->item(pos)->text().toStdString().compare("") == 0 &&
    ui->parent_link->item(pos)->text().toStdString().compare("") == 0)
  {
    ext_object_window.LoadNewExternalJoint();
  } else {  // If not load the current ext joint
    ext_object_window.LoadExternalJoint();
  }
  ext_object_window.exec();
  if (ext_object_window.success) {
    scene.object_vector[pos] = ext_object_window.target_object;     // Copy over object from window
    if (scene.object_vector[pos].ext_joint.parent_obj_pos >= 0 &&
      scene.object_vector[pos].ext_joint.parent_link_pos >= 0)
    {
      ui->parent_obj->item(pos)->setText(
        QString::fromStdString(
          scene.object_vector[scene.
          object_vector[pos].ext_joint.parent_obj_pos].name));
      ui->parent_link->item(pos)->setText(
        QString::fromStdString(
          scene.object_vector[scene.
          object_vector[pos].ext_joint.parent_obj_pos].link_vector[scene.object_vector[pos].
          ext_joint.
          parent_link_pos].name));
    } else {
      ui->parent_obj->item(pos)->setText(QString::fromStdString("world"));
      ui->parent_link->item(pos)->setText(QString::fromStdString("world"));
    }
  }
  boost::filesystem::current_path(scenes_path);
}

void AddScene::on_delete_object_clicked()
{
  auto list = ui->object_list->selectionModel()->selectedIndexes();
  if (list.size() != 0) {
    QListWidgetItem * item = ui->object_list->selectedItems().first();
    int pos = ui->object_list->row(item);

    std::string deleted_object = item->text().toStdString();     // Check deleted object name
    // Delete the parent objects if the objects are deleted.
    // Check the parent obejects of the external joints
    for (int i = 0; i < static_cast<int>(scene.object_vector.size()); i++) {
      if (scene.object_vector[i].ext_joint.parent_obj_pos == pos) {
        scene.object_vector[i].ext_joint.parent_obj_pos = -1;
        scene.object_vector[i].ext_joint.parent_link_pos = -1;
        ui->parent_obj->item(i)->setText(QString::fromStdString(""));
        ui->parent_link->item(i)->setText(QString::fromStdString(""));
      }

      if (scene.object_vector[i].ext_joint.parent_obj_pos > pos) {
        scene.object_vector[i].ext_joint.parent_obj_pos--;
      }
    }
    if (scene.object_vector.size() > 1) {
      scene.object_vector.erase(scene.object_vector.begin() + list[0].row());
    } else {
      scene.object_vector.clear();
    }
    delete ui->object_list_2->takeItem(ui->object_list->row(item));
    delete ui->parent_obj->takeItem(ui->object_list->row(item));
    delete ui->parent_link->takeItem(ui->object_list->row(item));
    delete ui->object_list->takeItem(ui->object_list->row(item));
  }
  boost::filesystem::current_path(scenes_path);
}

void AddScene::on_add_robot_clicked()
{
  boost::filesystem::current_path(scenes_path);
  AddRobot robot_window;
  int num_robots_loaded = robot_window.LoadAvailableRobots();
  if (num_robots_loaded <= 0) {
    ui->robot_brand->setText(
      "<font color='orange'> No robot detected in the workcell folder.</font>");
    ui->robot_model->setText(
      "<font color='orange'> No robot detected in the workcell folder. </font>");
  } else {
    if (scene.robot_loaded) {
      if (scene.robot_vector.size() > 0) {
        robot_window.LoadExistingRobot(scene.robot_vector[0]);
      }
    }
    robot_window.setWindowTitle("Load a Robot");
    robot_window.setModal(true);
    robot_window.exec();
    if (robot_window.success) {
      scene.robot_vector.clear();       // for now only one robot
      scene.robot_vector.push_back(robot_window.robot);
      scene.robot_loaded = true;

      std::string robot_model_output = "<font color='green'>" + robot_window.robot.name + "</font>";
      ui->robot_model->setText(QString::fromStdString(robot_model_output));
      std::string robot_brand_output = "<font color='green'>" + robot_window.robot.brand +
        "</font>";
      ui->robot_brand->setText(QString::fromStdString(robot_brand_output));
      ui->robot_connected->setText("<font color='green'> Robot Loaded! </font>");
      ui->add_robot->setText(QString::fromStdString("Edit Robot"));
      if (scene.ee_loaded) {
      } else {
        ui->include_ee->stateChanged(0);
        ui->include_ee->setDisabled(false);
      }
    }
  }
  boost::filesystem::current_path(scenes_path);
}

void AddScene::on_add_ee_clicked()
{
  AddEndEffector ee_window;
  int num_ee = ee_window.LoadAvailableEE(scene.robot_vector[0]);
  if (num_ee <= 0) {
    ui->ee_brand->setText("<font color='orange'> No ee detected in the workcell folder.</font>");
    ui->ee_model->setText("<font color='orange'> No ee detected in the workcell folder. </font>");
  } else {
    if (scene.ee_loaded) {
      if (scene.ee_vector.size() > 0) {
        ee_window.LoadExistingEE(scene.ee_vector[0]);
      }
    }
    ee_window.setWindowTitle("Load End Effector");
    ee_window.setModal(true);
    ee_window.exec();
    if (ee_window.success) {
      scene.ee_vector.clear();       // for now only one robot
      scene.ee_vector.push_back(ee_window.ee);
      scene.ee_loaded = true;

      std::string ee_model_output = "<font color='green'>" + ee_window.ee.name + "</font>";
      ui->ee_model->setText(QString::fromStdString(ee_model_output));
      std::string ee_brand_output = "<font color='green'>" + ee_window.ee.brand + "</font>";
      ui->ee_brand->setText(QString::fromStdString(ee_brand_output));
      ui->robot_ee_check->setText("<font color='green'> Robot and EE connected! </font>");
      ui->add_ee->setText(QString::fromStdString("Edit End Effector"));
    }
  }


  boost::filesystem::current_path(scenes_path);
}

void AddScene::on_remove_robot_clicked()
{
  scene.robot_vector.clear();
  scene.robot_loaded = false;
  ui->include_robot->stateChanged(1);
  ui->include_ee->stateChanged(0);
  ui->include_ee->setChecked(false);
  ui->include_ee->setDisabled(true);
  ui->add_robot->setText(QString::fromStdString("Add Robot"));
}

void AddScene::on_remove_ee_clicked()
{
  scene.ee_vector.clear();
  scene.ee_loaded = false;
  ui->include_ee->stateChanged(1);
  ui->add_ee->setText(QString::fromStdString("Add End Effector"));
}

void AddScene::on_ok_clicked()
{
  ui->scene_errors->clear();
  if (CheckRobot() && CheckEE() && CheckExtJoint() && CheckSceneName()) {
    scene.name = ui->scene_name->text().toStdString();
    success = true;
    scene.loaded = true;
    this->close();
  } else {
    success = false;
  }
}

void AddScene::on_exit_clicked()
{
  success = false;
  this->close();
}

bool AddScene::CheckRobot()
{
  if (ui->include_robot->isChecked()) {
    if (scene.robot_loaded) {
      if (scene.robot_vector.size() == 0) {
        ui->scene_errors->append(
          "<font color='red'> FATAL ERROR. Robot was loaded but no robot found </font>");
        // Something is wrong, robot should not be loaded if there is no robot in vector
        return false;
      }
    } else {
      ui->scene_errors->append("<font color='red'> No Robot Loaded.</font>");
      // If Robot is checked, should load robot if not disable it
      return false;
    }
  } else {
    ui->scene_errors->append("<font color='red'> No Robot Loaded.</font>");
    return false;
  }
  boost::filesystem::current_path(scenes_path);
  return true;
}

bool AddScene::CheckEE()
{
  if (ui->include_ee->isChecked()) {
    if (scene.ee_loaded) {
      if (scene.ee_vector.size() == 0) {
        ui->scene_errors->append(
          "<font color='red'> FATAL ERROR."
          " End Effector was loaded but no end effector found </font>");
        // Something is wrong, ee should not be loaded if there is no ee in vector
        return false;
      }
    } else {
      ui->scene_errors->append(
        "<font color='red'> No End Effector Loaded."
        " Uncheck End effector option if you dont require one. </font>");
      // If include ee is checked, should load ee if not disable it
      return false;
    }
  } else {
    scene.ee_loaded = false;
    scene.ee_vector.clear();
  }
  boost::filesystem::current_path(scenes_path);
  return true;
}

bool AddScene::CheckExtJoint()
{
  // Check that every object is connected to something
  for (int i = 0; i < static_cast<int>(scene.object_vector.size()); i++) {
    if (ui->parent_obj->item(i)->text().toStdString().compare("") == 0 ||
      ui->parent_link->item(i)->text().toStdString().compare("") == 0)
    {
      ui->scene_errors->append(
        QString::fromStdString(
          "<font color='red'> ERROR. Object " +
          scene.object_vector[i].name +
          " is not conencted to anything. Please set an external joint for the object </font>"));

      return false;
    }
  }
  return true;
}

bool AddScene::CheckSceneName()
{
  std::string initial_name = ui->scene_name->text().toStdString();
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
    ui->scene_name->setText(QString::fromStdString(final_name));
    return true;
  } else {
    ui->scene_errors->append("<font color='red'> Please enter a scene name. </font>");
    return false;
  }
}

void AddScene::LoadScene(Scene scene_input)
{
  boost::filesystem::current_path(scenes_path);
  scene = scene_input;
  // Load Scene Name
  ui->scene_name->setText(QString::fromStdString(scene.name));
  // Load Objects in tables
  for (int object = 0; object < static_cast<int>(scene.object_vector.size()); object++) {
    LoadObject(scene.object_vector[object]);
  }

  // Load Robots
  if (scene.robot_loaded) {
    ui->add_robot->setText(QString::fromStdString("Edit Robot"));
    LoadRobot(scene.robot_vector[0]);
    // Load EE
    if (scene.ee_loaded) {
      ui->include_ee->setChecked(true);
      LoadEE(scene.ee_vector[0]);
    }
  }
}
void AddScene::LoadRobot(Robot robot)
{
  ui->include_robot->setChecked(true);
  std::string robot_model_output = "<font color='green'>" + robot.name + "</font>";
  ui->robot_model->setText(QString::fromStdString(robot_model_output));
  std::string robot_brand_output = "<font color='green'>" + robot.brand + "</font>";
  ui->robot_brand->setText(QString::fromStdString(robot_brand_output));
  ui->robot_connected->setText("<font color='green'> Robot Loaded! </font>");
  ui->include_ee->stateChanged(0);
  ui->include_ee->setDisabled(false);
}

void AddScene::LoadEE(EndEffector ee)
{
  std::string ee_model_output = "<font color='green'>" + ee.name + "</font>";
  ui->ee_model->setText(QString::fromStdString(ee_model_output));
  std::string ee_brand_output = "<font color='green'>" + ee.brand + "</font>";
  ui->ee_brand->setText(QString::fromStdString(ee_brand_output));
  ui->robot_ee_check->setText("<font color='green'> Robot and EE connected! </font>");
  ui->add_ee->setText(QString::fromStdString("Edit End Effector"));
}

void AddScene::LoadObject(Object object)
{
  int parent_pos = object.ext_joint.parent_obj_pos;
  int parent_link_pos = object.ext_joint.parent_link_pos;
  ui->object_list->addItem(QString::fromStdString(object.name));
  ui->object_list_2->addItem(QString::fromStdString(object.name));
  if (parent_pos < 0) {
    ui->parent_obj->addItem(QString::fromStdString("world"));
  } else {
    ui->parent_obj->addItem(QString::fromStdString(scene.object_vector[parent_pos].name));
  }
  if (parent_link_pos < 0) {
    ui->parent_link->addItem(QString::fromStdString("world"));
  } else {
    ui->parent_link->addItem(
      QString::fromStdString(
        scene.object_vector[parent_pos].link_vector[
          parent_link_pos].name));
  }
}
// Load objects in the object tables

void AddScene::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}

void AddScene::on_load_object_clicked()
{
  std::vector<std::string> available_object_names;
  for (int i = 0; i < static_cast<int>(scene.object_vector.size()); i++) {
    available_object_names.push_back(scene.object_vector[i].name);
  }

  boost::filesystem::current_path(assets_path);
  LoadObjects load_obj_window;
  load_obj_window.current_object_names = available_object_names;
  load_obj_window.setWindowTitle("Load Existing Objects");
  load_obj_window.setModal(true);
  load_obj_window.exec();
  if (load_obj_window.success) {
    ui->object_list->addItem(QString::fromStdString(load_obj_window.chosen_object.name));
    ui->object_list_2->addItem(QString::fromStdString(load_obj_window.chosen_object.name));
    ui->parent_obj->addItem("");
    ui->parent_link->addItem("");
    scene.object_vector.push_back(load_obj_window.chosen_object);
  }
  boost::filesystem::current_path(scenes_path);
}
