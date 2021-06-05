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

#include "gui/addrobot.h"

#include <boost/filesystem.hpp>
#include <QKeyEvent>
#include <iostream>
#include <sstream>
#include <string>

#include "gui/ui_addrobot.h"


AddRobot::AddRobot(QWidget * parent)
: QDialog(parent),
  ui(new Ui::AddRobot)
{
  ui->setupUi(this);
  on_include_origin_stateChanged(0);
  success = false;
}

AddRobot::~AddRobot()
{
  boost::filesystem::current_path(original_path);
  delete ui;
}
void AddRobot::LoadExistingRobot(Robot robot_input)
{
  robot = robot_input;
  // Populate Origin
  if (robot.origin.is_origin) {
    ui->include_origin->setChecked(true);
    on_include_origin_stateChanged(2);
    if (robot.origin.x >= 0) {ui->x_2->setText(QString::number(robot.origin.x));}
    if (robot.origin.y >= 0) {ui->y_2->setText(QString::number(robot.origin.y));}
    if (robot.origin.z >= 0) {ui->z_2->setText(QString::number(robot.origin.z));}
    if (robot.origin.roll >= 0) {ui->roll_2->setText(QString::number(robot.origin.roll));}
    if (robot.origin.pitch >= 0) {ui->pitch_2->setText(QString::number(robot.origin.pitch));}
    if (robot.origin.yaw >= 0) {ui->yaw_2->setText(QString::number(robot.origin.yaw));}
  } else {
    ui->include_origin->stateChanged(0);
  }

  // Populate Brand
  for (int brand = 0; brand < static_cast<int>(available_brands.size()); brand++) {
    if (robot.brand.compare(available_brands[brand]) == 0) {
      ui->robot_brand->setCurrentIndex(brand);
      for (int model = 0; model < static_cast<int>(available_robots[brand].size()); model++) {
        if (robot.name.compare(available_robots[brand][model].name) == 0) {
          ui->robot_model->setCurrentIndex(model);
          for (int link = 0; link <
            static_cast<int>(available_robots[brand][model].robot_links.size());
            link++)
          {
            if (robot.base_link.compare(available_robots[brand][model].robot_links[link]) == 0) {
              ui->robot_links->setCurrentIndex(link);
            }
            if (robot.ee_link.compare(available_robots[brand][model].robot_links[link]) == 0) {
              ui->robot_ee_link->setCurrentIndex(link);
            }
          }
          break;
        }
      }
      break;
    }
  }
}

int AddRobot::LoadAvailableRobots()
{
  original_path = boost::filesystem::current_path();
  boost::filesystem::current_path(boost::filesystem::current_path().branch_path());  //
  boost::filesystem::current_path("assets");
  boost::filesystem::current_path("robots");
  available_brands.clear();
  available_robots.clear();
  if (boost::filesystem::is_empty(boost::filesystem::current_path())) {
    return 0;
  } else {
    boost::filesystem::path temp_path(boost::filesystem::current_path());
    for (auto & filepath :
      boost::filesystem::directory_iterator(boost::filesystem::current_path()))
    {
      std::string temp_brand;
      std::vector<Robot> brand_robot_vector;
      for (auto it = filepath.path().string().crbegin(); it != filepath.path().string().crend();
        ++it)
      {
        if (*it != '/') {
          temp_brand = std::string(1, *it) + temp_brand;
        } else {
          boost::filesystem::current_path(temp_brand);
          if (temp_brand.compare("universal_robot") == 0) {
            std::vector<std::string> valid_robots;
            for (auto & filepath :
              boost::filesystem::directory_iterator(boost::filesystem::current_path()))
            {
              std::string temp_model;
              bool is_moveit = false;
              int char_count = 0;
              for (auto it = filepath.path().string().crbegin();
                it != filepath.path().string().crend(); ++it)
              {
                if (*it != '/') {
                  temp_model = std::string(1, *it) + temp_model;
                  char_count++;
                  if (char_count == 13) {
                    if (temp_model.compare("moveit_config") == 0) {
                      is_moveit = true;
                    }
                  }
                }
                if (*it == '/') {
                  break;
                }
              }
              if (is_moveit) {
                valid_robots.push_back(temp_model.substr(0, temp_model.size() - 14));
              }
            }
            brand_robot_vector = LoadUR(valid_robots);
          } else {
            std::vector<std::string> moveit_configs;
            std::vector<std::string> descriptions;
            for (auto & filepath :
              boost::filesystem::directory_iterator(boost::filesystem::current_path()))
            {
              bool is_description = false;
              bool is_moveit = false;

              std::string temp_model;
              int char_count = 0;
              // Only _descrtiption
              for (auto it = filepath.path().string().crbegin();
                it != filepath.path().string().crend(); ++it)
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
                      is_moveit = true;
                    }
                  }
                } else {
                  break;
                }
              }

              if (is_description) {
                descriptions.push_back(temp_model.substr(0, temp_model.size() - 12));
              }
              if (is_moveit) {
                moveit_configs.push_back(temp_model.substr(0, temp_model.size() - 14));
              }
            }
            boost::filesystem::current_path(boost::filesystem::current_path().branch_path());

            if (moveit_configs.size() < descriptions.size()) {
              for (int i = 0; i < static_cast<int>(descriptions.size()); i++) {
                if (std::find(
                    moveit_configs.begin(), moveit_configs.end(),
                    descriptions[i]) != moveit_configs.end())
                {
                  brand_robot_vector.push_back(
                    LoadRobot(
                      descriptions[i] + "_description",
                      temp_brand));
                }
              }
            } else {
              for (int i = 0; i < static_cast<int>(moveit_configs.size()); i++) {
                if (std::find(
                    descriptions.begin(), descriptions.end(),
                    moveit_configs[i]) != descriptions.end())
                {
                  brand_robot_vector.push_back(
                    LoadRobot(
                      moveit_configs[i] + "_description",
                      temp_brand));
                }
              }
            }
          }
          break;
        }
      }
      if (!brand_robot_vector.empty()) {
        available_robots.push_back(brand_robot_vector);
        available_brands.push_back(temp_brand);
      }
      boost::filesystem::current_path(temp_path);
    }
    for (int i3 = 0; i3 < static_cast<int>(available_brands.size()); i3++) {
      ui->robot_brand->addItem(QString::fromStdString(available_brands[i3]));
    }

    ui->robot_brand->setCurrentIndex(0);
    on_robot_brand_currentIndexChanged(0);
    ui->robot_model->setCurrentText(0);
    on_robot_model_currentIndexChanged(0);
    ui->robot_links->setCurrentIndex(0);
    ui->robot_ee_link->setCurrentIndex(0);
    int total_count = 0;
    for (int i = 0; i < static_cast<int>(available_brands.size()); i++) {
      for (int i2 = 0; i2 < static_cast<int>(available_robots[i].size()); i2++) {
        total_count++;
      }
    }
    return total_count;
  }
  return 0;
}

Robot AddRobot::LoadRobot(std::string file, std::string brand)
{
  boost::filesystem::path temp_path(boost::filesystem::current_path());
  Robot temp_robot;
  temp_robot.filepath = boost::filesystem::current_path().string() + "/" + file;
  boost::filesystem::current_path(brand);
  boost::filesystem::current_path(file);
  boost::filesystem::current_path("urdf");
  temp_robot.brand = brand;
  temp_robot.name = file.erase(file.length() - 12);
  temp_robot.robot_links = GetLinks(temp_robot.name + ".urdf.xacro");
  boost::filesystem::current_path(temp_path);
  return temp_robot;
}

std::vector<Robot> AddRobot::LoadUR(std::vector<std::string> robot_list)
// load ur robots based on the way it is structured
{
  boost::filesystem::current_path("ur_description");
  boost::filesystem::current_path("urdf");

  std::vector<Robot> ur_robot_vector;
  for (int i = 0; i < static_cast<int>(robot_list.size()); i++) {
    Robot temp_robot;
    temp_robot.brand = "universal_robot";
    temp_robot.name = robot_list[i];
    temp_robot.robot_links = GetLinks(temp_robot.name + ".urdf.xacro");
    ur_robot_vector.push_back(temp_robot);
  }
  return ur_robot_vector;
}

std::vector<std::string> AddRobot::GetLinks(std::string filename)
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

void AddRobot::on_robot_brand_currentIndexChanged(int index)
{
  bool oldState = ui->robot_model->blockSignals(true);
  ui->robot_model->clear();
  for (int i = 0; i < static_cast<int>(available_robots[index].size()); i++) {
    ui->robot_model->addItem(QString::fromStdString(available_robots[index][i].name));
  }
  ui->robot_model->blockSignals(oldState);
  ui->robot_model->setCurrentIndex(0);
  on_robot_model_currentIndexChanged(0);
}


void AddRobot::on_robot_model_currentIndexChanged(int index)
{
  bool oldState = ui->robot_links->blockSignals(true);
  bool oldState2 = ui->robot_ee_link->blockSignals(true);
  ui->robot_links->clear();
  ui->robot_ee_link->clear();
  for (int i = 0;
    i <
    static_cast<int>(available_robots[ui->robot_brand->currentIndex()][ui->robot_model->currentIndex()  // NOLINT
    ].
    robot_links
    .size()); i++)
  {
    ui->robot_links->addItem(
      QString::fromStdString(
        available_robots[ui->robot_brand->currentIndex()
        ][index].robot_links[i]));
    ui->robot_ee_link->addItem(
      QString::fromStdString(
        available_robots[ui->robot_brand->currentIndex()
        ][index].robot_links[i]));
  }
  ui->robot_links->blockSignals(oldState);
  ui->robot_links->setCurrentIndex(0);
  ui->robot_ee_link->blockSignals(oldState2);
  ui->robot_ee_link->setCurrentIndex(0);
}


void AddRobot::on_include_origin_stateChanged(int arg1)
{
  if (arg1 == 0) {
    // No Origin
    ui->x_2->setDisabled(true);
    ui->x_2->setText(QString::fromStdString("0"));
    ui->x_label_2->setDisabled(true);
    ui->y_2->setDisabled(true);
    ui->y_2->setText(QString::fromStdString("0"));
    ui->y_label_2->setDisabled(true);
    ui->z_2->setDisabled(true);
    ui->z_2->setText(QString::fromStdString("0"));
    ui->z_label_2->setDisabled(true);
    ui->roll_2->setDisabled(true);
    ui->roll_2->setText(QString::fromStdString("0"));
    ui->roll_label_2->setDisabled(true);
    ui->pitch_2->setDisabled(true);
    ui->pitch_2->setText(QString::fromStdString("0"));
    ui->pitch_label_2->setDisabled(true);
    ui->yaw_2->setDisabled(true);
    ui->yaw_2->setText(QString::fromStdString("0"));
    ui->yaw_label_2->setDisabled(true);
    ui->origin_label_2->setDisabled(true);
    ui->position_label_2->setDisabled(true);
    ui->orientation_label_2->setDisabled(true);
  } else {
    // Have origin
    ui->x_2->setDisabled(false);
    ui->x_2->clear();
    ui->x_label_2->setDisabled(false);
    ui->y_2->setDisabled(false);
    ui->y_2->clear();
    ui->y_label_2->setDisabled(false);
    ui->z_2->setDisabled(false);
    ui->z_2->clear();
    ui->z_label_2->setDisabled(false);
    ui->roll_2->setDisabled(false);
    ui->roll_2->clear();
    ui->roll_label_2->setDisabled(false);
    ui->pitch_2->setDisabled(false);
    ui->pitch_2->clear();
    ui->pitch_label_2->setDisabled(false);
    ui->yaw_2->setDisabled(false);
    ui->yaw_2->clear();
    ui->yaw_label_2->setDisabled(false);
    ui->origin_label_2->setDisabled(false);
    ui->position_label_2->setDisabled(false);
    ui->orientation_label_2->setDisabled(false);
  }
}

void AddRobot::on_exit_2_clicked()
{
  success = false;
  this->close();
}

void AddRobot::on_ok_2_clicked()
{
  if (ui->robot_brand->currentIndex() < 0 || ui->robot_model->currentIndex() < 0) {
    ui->errorlist_2->append(" <font color='red'> Invalid Robot Brand or Robot Model \n </font>");
  } else {
    robot = available_robots[ui->robot_brand->currentIndex()][ui->robot_model->currentIndex()];
    ui->errorlist_2->clear();
    if (ErrorCheckOrigin() == 0) {
      if (ui->robot_links->currentIndex() < 0) {
        ui->errorlist_2->append(" <font color='red'> Invalid Base Link \n </font>");
        return;
      }
      if (ui->robot_ee_link->currentIndex() < 0) {
        ui->errorlist_2->append(" <font color='red'> Invalid EE Link \n </font>");
        return;
      }
      robot.base_link = ui->robot_links->currentText().toStdString();
      robot.ee_link = ui->robot_ee_link->currentText().toStdString();
      success = true;
      this->close();
    }
  }
}

int AddRobot::ErrorCheckOrigin()
{
  int num_errors = 0;
  if (ui->include_origin->isChecked()) {
    robot.origin.is_origin = true;
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
        " <font color='red'> All Origin Fields are empty."
        " Uncheck the Origin selection to disable the option. \n </font>");
      num_errors++;
    } else {
      if (!all_full_xyz && !all_empty_xyz) {
        ui->errorlist_2->append(
          "<font color='red'> XYZ values not complete."
          " Leave it all blank for default values. \n </font>");
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
            ui->errorlist_2->append(
              "<font color='red'> Type Error: XYZ need to be floats \n </font>");
            num_errors++;
          } else {
            robot.origin.x = input_x.toFloat();
            robot.origin.y = input_y.toFloat();
            robot.origin.z = input_z.toFloat();
          }
        }
      }
      if (!all_full_rpy && !all_empty_rpy) {
        ui->errorlist_2->append(
          "<font color='red'> RPY values not complete."
          " Leave it all blank for default values </font>");
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
            ui->errorlist_2->append(
              " <font color='red'> Type Error: RPY need"
              " to be floats </font>");
            num_errors++;
          } else {
            robot.origin.roll = ui->roll_2->text().toFloat();
            robot.origin.pitch = ui->pitch_2->text().toFloat();
            robot.origin.yaw = ui->yaw_2->text().toFloat();
          }
        }
      }
    }
  } else {
    robot.origin.disableOrigin();
  }
  return num_errors;
}

void AddRobot::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}
