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

#include "gui/addendeffector.h"
#include <QString>
#include <QKeyEvent>
#include <boost/filesystem.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "gui/ui_addendeffector.h"
#include "attributes/robot.h"

AddEndEffector::AddEndEffector(QWidget * parent)
: QDialog(parent),
  ui(new Ui::AddEndEffector)
{
  ui->setupUi(this);
  for (int i = 0; i < static_cast<int>(supported_types.size()); i++) {
    ui->ee_type->addItem(QString::fromStdString(supported_types[i]));
  }
  on_include_origin_stateChanged(0);
  success = false;
}


int AddEndEffector::LoadAvailableEE(Robot robot)
{
  original_path = boost::filesystem::current_path();
  ui->parent_object->setText(QString::fromStdString(robot.name));
  ui->parent_link->setText(QString::fromStdString(robot.ee_link));
  boost::filesystem::current_path(boost::filesystem::current_path().branch_path());  //
  boost::filesystem::current_path("assets");
  boost::filesystem::current_path("end_effectors");
  available_brands.clear();
  available_ee.clear();
  if (boost::filesystem::is_empty(boost::filesystem::current_path())) {
    return 0;

  } else {
    for (auto & filepath :
      boost::filesystem::directory_iterator(boost::filesystem::current_path()))
    {
      std::string temp_brand;
      std::vector<EndEffector> brand_ee_vector;

      // Iterate brand
      for (auto it = filepath.path().string().crbegin(); it != filepath.path().string().crend();
        ++it)
      {
        if (*it != '/') {
          temp_brand = std::string(1, *it) + temp_brand;
        } else {
          boost::filesystem::current_path(temp_brand);
          std::vector<std::string> moveit_configs;
          std::vector<std::string> descriptions;
          for (auto & filepath_model :
            boost::filesystem::directory_iterator(boost::filesystem::current_path()))
          {
            bool is_moveit_config = false;
            bool is_description = false;
            std::string temp_model;
            int char_count = 0;

            for (auto it = filepath_model.path().string().crbegin();
              it != filepath_model.path().string().crend(); ++it)
            {
              if (*it != '/') {
                temp_model = std::string(1, *it) + temp_model;
                char_count++;
                if (char_count == 11) {
                  if (temp_model.compare("description") == 0) {
                    is_description = true;
                  }
                }
                if (char_count == 13) {
                  if (temp_model.compare("moveit_config") == 0) {
                    is_moveit_config = true;
                  }
                }
              } else {
                break;
              }
            }
            if (is_description) {
              descriptions.push_back(temp_model.substr(0, temp_model.size() - 12));
            }
            if (is_moveit_config) {
              moveit_configs.push_back(temp_model.substr(0, temp_model.size() - 14));
            }
          }
          if (moveit_configs.size() < descriptions.size()) {
            for (int i = 0; i < static_cast<int>(descriptions.size()); i++) {
              if (std::find(
                  moveit_configs.begin(), moveit_configs.end(),
                  descriptions[i]) != moveit_configs.end())
              {
                brand_ee_vector.push_back(LoadEE(descriptions[i] + "_description", temp_brand));
              }
            }
          } else {
            for (int i = 0; i < static_cast<int>(moveit_configs.size()); i++) {
              if (std::find(
                  descriptions.begin(), descriptions.end(),
                  moveit_configs[i]) != descriptions.end())
              {
                brand_ee_vector.push_back(LoadEE(moveit_configs[i] + "_description", temp_brand));
              }
            }
          }
          boost::filesystem::current_path(boost::filesystem::current_path().branch_path());
          break;
        }
      }
      if (!brand_ee_vector.empty()) {
        available_ee.push_back(brand_ee_vector);
        available_brands.push_back(temp_brand);
      }
    }
    for (int i3 = 0; i3 < static_cast<int>(available_brands.size()); i3++) {
      ui->ee_brand->addItem(QString::fromStdString(available_brands[i3]));
    }
    ui->ee_brand->setCurrentIndex(0);
    on_ee_brand_currentIndexChanged(0);
    ui->ee_model->setCurrentIndex(0);
    on_ee_model_currentIndexChanged(0);
    ui->ee_links->setCurrentIndex(0);
    int total_ee = 0;
    for (int i = 0; i < static_cast<int>(available_brands.size()); i++) {
      for (int j = 0; j < static_cast<int>(available_ee[i].size()); j++) {
        total_ee++;
      }
    }
    return total_ee;
  }
  return 0;
}

void AddEndEffector::LoadExistingEE(EndEffector ee_input)
{
  ee = ee_input;

  //  Populate Origin
  if (ee.origin.is_origin) {
    ui->include_origin->setChecked(true);
    on_include_origin_stateChanged(2);
    if (ee.origin.x >= 0) {ui->x->setText(QString::number(ee.origin.x));}
    if (ee.origin.y >= 0) {ui->y->setText(QString::number(ee.origin.y));}
    if (ee.origin.z >= 0) {ui->z->setText(QString::number(ee.origin.z));}
    if (ee.origin.roll >= 0) {ui->roll->setText(QString::number(ee.origin.roll));}
    if (ee.origin.pitch >= 0) {ui->pitch->setText(QString::number(ee.origin.pitch));}
    if (ee.origin.yaw >= 0) {ui->yaw->setText(QString::number(ee.origin.yaw));}
  } else {
    ui->include_origin->stateChanged(0);
  }

  //  Populate Brand
  for (int brand = 0; brand < static_cast<int>(available_brands.size()); brand++) {
    if (ee.brand.compare(available_brands[brand]) == 0) {
      ui->ee_brand->setCurrentIndex(brand);
      for (int model = 0; model < static_cast<int>(available_ee[brand].size()); model++) {
        if (ee.name.compare(available_ee[brand][model].name) == 0) {
          ui->ee_model->setCurrentIndex(model);
          for (int link = 0;
            link < static_cast<int>(available_ee[brand][model].ee_links.size()); link++)
          {
            if (ee.base_link.compare(available_ee[brand][model].ee_links[link]) == 0) {
              ui->ee_links->setCurrentIndex(link);
              break;
            }
          }
          break;
        }
      }
      break;
    }
  }
}

void AddEndEffector::on_ee_brand_currentIndexChanged(int index)
{
  bool oldState = ui->ee_model->blockSignals(true);
  ui->ee_model->clear();
  for (int i = 0; i < static_cast<int>(available_ee[index].size()); i++) {
    ui->ee_model->addItem(QString::fromStdString(available_ee[index][i].name));
  }
  ui->ee_model->blockSignals(oldState);
  ui->ee_model->setCurrentIndex(0);
  on_ee_model_currentIndexChanged(0);
}

void AddEndEffector::on_ee_model_currentIndexChanged(int index)
{
  bool oldState = ui->ee_links->blockSignals(true);
  ui->ee_links->clear();
  if (ui->ee_model->currentIndex() >= 0 && ui->ee_brand->currentIndex() >= 0) {
    for (int i = 0;
      i <
      static_cast<int>(available_ee[ui->ee_brand->
      currentIndex()][ui->ee_model->currentIndex()].ee_links.size());
      i++)
    {
      ui->ee_links->addItem(
        QString::fromStdString(
          available_ee[ui->ee_brand->currentIndex()][index]
          .ee_links[i]));
    }
  }
  ui->ee_links->blockSignals(oldState);
}

void AddEndEffector::on_ee_type_currentIndexChanged(int index)
{
  bool oldState1 = ui->attribute_1->blockSignals(true);
  bool oldState2 = ui->attribute_2->blockSignals(true);
  ui->attribute_1->clear();
  ui->attribute_2->clear();
  if (supported_attributes[index].size() == 1) {
    ui->attribute_1_label->setText("Fingers:");
    for (int i = 0; i < static_cast<int>(supported_attributes[index][0].size()); i++) {
      ui->attribute_1->addItem(QString::number(supported_attributes[index][0][i]));
    }
    ui->attribute_2->hide();
    ui->attribute_2_label->hide();
  } else if (supported_attributes[index].size() == 2) {
    ui->attribute_2->show();
    ui->attribute_2_label->show();
    ui->attribute_1_label->setText("Suction array Width");
    ui->attribute_2_label->setText("Suction array Height");
    for (int i = 0; i < static_cast<int>(supported_attributes[index][0].size()); i++) {
      ui->attribute_1->addItem(QString::number(supported_attributes[index][0][i]));
    }
    for (int i = 0; i < static_cast<int>(supported_attributes[index][1].size()); i++) {
      ui->attribute_2->addItem(QString::number(supported_attributes[index][1][i]));
    }
  }
  ui->attribute_1->blockSignals(oldState1);
  ui->attribute_2->blockSignals(oldState2);
}

EndEffector AddEndEffector::LoadEE(std::string file, std::string brand)
{
  EndEffector temp_ee;
  temp_ee.filepath = boost::filesystem::current_path().string() + "/" + file;
  boost::filesystem::current_path(file);
  boost::filesystem::current_path("urdf");
  temp_ee.brand = brand;
  temp_ee.name = file.erase(file.length() - 12);
  temp_ee.ee_links = GetLinks(temp_ee.name + "_gripper.urdf.xacro");
  boost::filesystem::current_path(boost::filesystem::current_path().branch_path().branch_path());
  return temp_ee;
}
std::vector<std::string> AddEndEffector::GetLinks(std::string filename)
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
    if (temp_string.compare("<link name=") == 0) {
      remove_if(line.begin(), line.end(), isspace);
      line.erase(0, line.find("\"") + 1);
      line.erase(line.begin() + line.find("\""), line.end());
      line.erase(0, line.find("}") + 1);
      links.push_back(line);
    }
  }

  return links;
}

void AddEndEffector::on_include_origin_stateChanged(int arg1)
{
  if (arg1 == 0) {
    //  No Origin
    ui->x->setDisabled(true);
    ui->x->setText(QString::fromStdString("0"));
    ui->x_label->setDisabled(true);
    ui->y->setDisabled(true);
    ui->y->setText(QString::fromStdString("0"));
    ui->y_label->setDisabled(true);
    ui->z->setDisabled(true);
    ui->z->setText(QString::fromStdString("0"));
    ui->z_label->setDisabled(true);
    ui->roll->setDisabled(true);
    ui->roll->setText(QString::fromStdString("0"));
    ui->roll_label->setDisabled(true);
    ui->pitch->setDisabled(true);
    ui->pitch->setText(QString::fromStdString("0"));
    ui->pitch_label->setDisabled(true);
    ui->yaw->setDisabled(true);
    ui->yaw->setText(QString::fromStdString("0"));
    ui->yaw_label->setDisabled(true);
    ui->origin_label->setDisabled(true);
    ui->position_label->setDisabled(true);
    ui->orientation_label->setDisabled(true);
  } else {
    //  Have origin
    ui->x->setDisabled(false);
    ui->x->clear();
    ui->x_label->setDisabled(false);
    ui->y->setDisabled(false);
    ui->y->clear();
    ui->y_label->setDisabled(false);
    ui->z->setDisabled(false);
    ui->z->clear();
    ui->z_label->setDisabled(false);
    ui->roll->setDisabled(false);
    ui->roll->clear();
    ui->roll_label->setDisabled(false);
    ui->pitch->setDisabled(false);
    ui->pitch->clear();
    ui->pitch_label->setDisabled(false);
    ui->yaw->setDisabled(false);
    ui->yaw->clear();
    ui->yaw_label->setDisabled(false);
    ui->origin_label->setDisabled(false);
    ui->position_label->setDisabled(false);
    ui->orientation_label->setDisabled(false);
  }
}

int AddEndEffector::ErrorCheckOrigin()
{
  int num_errors = 0;
  if (ui->include_origin->isChecked()) {
    ee.origin.is_origin = true;
    bool all_empty_xyz =
      (ui->x->text().isEmpty() && ui->y->text().isEmpty() && ui->z->text().isEmpty());
    bool all_full_xyz =
      (!ui->x->text().isEmpty() && !ui->y->text().isEmpty() && !ui->z->text().isEmpty());

    bool all_empty_rpy =
      (ui->roll->text().isEmpty() && ui->pitch->text().isEmpty() && ui->yaw->text().isEmpty());
    bool all_full_rpy =
      (!ui->roll->text().isEmpty() && !ui->pitch->text().isEmpty() && !ui->yaw->text().isEmpty());
    if (all_empty_rpy && all_empty_xyz) {
      ui->errorlist->
      append(
        "<font color='red'> All Origin Fields are empty."
        "Uncheck the Origin selection to disable the option. \n </font>");
      num_errors++;
    } else {
      if (!all_full_xyz && !all_empty_xyz) {
        ui->errorlist->
        append(
          "<font color='red'>XYZ values not complete."
          " Leave it all blank for default values. \n </font>");
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
            ui->errorlist->
            append("<font color='red'> Type Error: XYZ need to be floats \n </font>");
            num_errors++;
          } else {
            ee.origin.x = input_x.toFloat();
            ee.origin.y = input_y.toFloat();
            ee.origin.z = input_z.toFloat();
          }
        }
      }
      if (!all_full_rpy && !all_empty_rpy) {
        ui->errorlist->
        append(
          "<font color='red'> RPY values not complete."
          " Leave it all blank for default values </font>");
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
            ui->errorlist->append(" <font color='red'> Type Error: RPY need to be floats </font>");
            num_errors++;
          } else {
            ee.origin.roll = ui->roll->text().toFloat();
            ee.origin.pitch = ui->pitch->text().toFloat();
            ee.origin.yaw = ui->yaw->text().toFloat();
          }
        }
      }
    }
  } else {
    ee.origin.disableOrigin();
  }
  return num_errors;
}

AddEndEffector::~AddEndEffector()
{
  boost::filesystem::current_path(original_path);
  delete ui;
}

void AddEndEffector::on_ok_clicked()
{
  if (ui->ee_brand->currentIndex() < 0) {
    ui->errorlist->append(QString::fromStdString("EE Brand Error"));
    return;
  }
  if (ui->ee_model->currentIndex() < 0) {
    ui->errorlist->append(QString::fromStdString("EE model Error"));
    return;
  }
  if (ui->ee_links->currentIndex() < 0) {
    ui->errorlist->append(QString::fromStdString("EE Link Error"));
    return;
  }

  ee = available_ee[ui->ee_brand->currentIndex()][ui->ee_model->currentIndex()];
  ui->errorlist->clear();
  if (ErrorCheckOrigin() == 0) {
    ee.ee_type = ui->ee_type->currentText().toStdString();
    ee.attribute_1 = ui->attribute_1->currentText().toInt();

    if (ee.ee_type.compare("suction")) {
      ee.attribute_2 = ui->attribute_2->currentText().toInt();
    }
    ee.base_link = ui->ee_links->currentText().toStdString();
    ee.robot_link = ui->parent_link->text().toStdString();
    success = true;
    this->close();
  }
}

void AddEndEffector::on_exit_clicked()
{
  success = false;
  this->close();
}

void AddEndEffector::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}
