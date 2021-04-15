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

#include <QKeyEvent>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdio>

#include "gui/ui_scene_select.h"
#include "gui/scene_select.h"

#include "gui/replacewarning.h"
#include "gui/addscene.h"

#include "yaml_parser/generate_yaml.h"
#include "include/armhand_xacro_parser.h"
#include "include/file_functions.h"
#include "include/object_package_parser.h"
#include "include/object_xacro_parser.h"
#include "include/scene_check.h"
#include "include/scene_parser.h"
#include "include/scene_xacro_parser.h"


SceneSelect::SceneSelect(QWidget * parent)
: QDialog(parent),
  ui(new Ui::SceneSelect)
{
  scenes_path = boost::filesystem::current_path();
  boost::filesystem::current_path(boost::filesystem::current_path().branch_path());
  workcell_path = boost::filesystem::current_path();
  try {
    boost::filesystem::current_path("easy_manipulation_deployment/workcell_builder/examples");
  } catch (boost::filesystem::filesystem_error const & e) {
    std::cerr << e.what() << '\n';
  }
  example_path = boost::filesystem::current_path();

  boost::filesystem::current_path(workcell_path);
  boost::filesystem::current_path("assets");
  assets_path = boost::filesystem::current_path();
  boost::filesystem::current_path(scenes_path);
  ui->setupUi(this);
}

SceneSelect::~SceneSelect()
{
  delete ui;
}
void SceneSelect::load_workcell(Workcell workcell_input)
{
  workcell = workcell_input;
  refresh_scenes(0);
}
void SceneSelect::on_add_scene_clicked()
{
  boost::filesystem::current_path(scenes_path);

  AddScene scene_window;
  scene_window.setWindowTitle("Create New Scene");
  scene_window.setModal(true);
  scene_window.workcell_path = workcell_path;
  scene_window.exec();
  if (scene_window.success) {
    boost::filesystem::current_path(scenes_path);
    workcell.scene_vector.push_back(scene_window.scene);
    generate_scene_package(scenes_path, scene_window.scene.name, workcell.ros_ver);
    refresh_scenes(workcell.scene_vector.size() - 1);
  }
  boost::filesystem::current_path(scenes_path);
}

void SceneSelect::generate_scene_package(
  boost::filesystem::path scene_filepath,
  std::string scene_name, int ros_ver)
{
  boost::filesystem::current_path(scene_filepath);
  if (!boost::filesystem::exists(scene_name)) {
    boost::filesystem::create_directory(scene_name);
  }
  boost::filesystem::current_path(scene_name);
  if (!boost::filesystem::exists("urdf")) {
    boost::filesystem::create_directory("urdf");
  }
  boost::filesystem::path workcell_path(scene_filepath.branch_path());
  generate_cmakelists(workcell_path, scene_name, ros_ver);
  generate_package_xml(workcell_path, scene_name, ros_ver);
  boost::filesystem::current_path(scene_filepath);
}

void SceneSelect::generate_scene_files(Scene scene)
{
  // generate environment.urdf.xacro
  boost::filesystem::current_path(workcell_path.string() + "/scenes/" + scene.name + "/urdf");
  generate_scene_xacro(scene);
  if (scene.robot_loaded && scene.ee_loaded) {
    generate_armhand_xacro(scene.robot_vector[0], scene.ee_vector[0], scene.name);
  }
  if (scene.robot_loaded && !scene.ee_loaded) {  // no ee
    generate_armhand_xacro(scene.robot_vector[0], scene.name);
  }
  if (!scene.robot_loaded && !scene.ee_loaded) {  // no robot and ee
    generate_armhand_xacro(scene.name);
  }
  boost::filesystem::path launch_path(
    workcell_path.string() + "/easy_manipulation_deployment/workcell_builder/examples/ros" +
    std::to_string(workcell.ros_ver) + "/launch");
  boost::filesystem::path target_path(workcell_path.string() + "/scenes/" + scene.name + "/launch");
  copyDir(launch_path, target_path);
  boost::filesystem::current_path(target_path);

  if (workcell.ros_ver == 1) {  // ROS1
    find_replace("scene.launch", "scene_interim.launch", "scene_name", scene.name);
    find_replace("scene_interim.launch", "scene.launch", "robot_name", scene.robot_vector[0].name);
    find_replace("demo.launch", "demo_interim.launch", "scene_name", scene.name);
    find_replace("demo_interim.launch", "demo.launch", "robot_name", scene.robot_vector[0].name);
  } else if (workcell.ros_ver == 2) {  // ROS2
    find_replace("demo.launch.py", "demo_interim.launch.py", "scene_name", scene.name);
    find_replace(
      "demo_interim.launch.py", "demo_interim2.launch.py", "base_link_name",
      scene.robot_vector[0].base_link);
    find_replace(
      "demo_interim2.launch.py", "demo.launch.py", "moveit_config_name",
      scene.robot_vector[0].name + "_moveit_config");
  }
}
void SceneSelect::refresh_scenes(int latest_scene)
{
  if (latest_scene < 0) {latest_scene = 0;}
  bool oldState = ui->scene_list->blockSignals(true);
  ui->scene_list->clear();  // Clear the dropdown menu
  if (workcell.scene_vector.size() > 0) {  // There are scenes in the workcell
    ui->scene_list->setDisabled(false);     // Enable the dropdown menu
    for (int scene = 0; scene < static_cast<int>(workcell.scene_vector.size()); scene++) {
      ui->scene_list->addItem(QString::fromStdString(workcell.scene_vector[scene].name));
    }
    ui->scene_list->setCurrentIndex(latest_scene);     // Display the latest scene the user created
    on_scene_list_currentIndexChanged(latest_scene);
    ui->edit_scene->setDisabled(false);
    ui->delete_scene->setDisabled(false);
    ui->generate_yaml->setDisabled(false);
    ui->generate_files->setDisabled(false);
  } else {  // no scenes
    ui->scene_list->setDisabled(true);
    ui->generate_yaml->setDisabled(true);
    ui->generate_files->setDisabled(true);
    ui->edit_scene->setDisabled(true);
    ui->delete_scene->setDisabled(true);
    ui->error_workcell->append("<font color='red'> No scenes available. </font>");
  }
  ui->scene_list->blockSignals(oldState);
}
void SceneSelect::on_delete_scene_clicked()
{
  ReplaceWarning replace_window;
  replace_window.setWindowTitle("Edit Scene");
  replace_window.set_label("Warning: Scene folders with all files will be deleted. Continue?");
  replace_window.setModal(true);
  replace_window.exec();

  if (replace_window.decision) {  // user allows for scene folder deletion
    ui->error_workcell->clear();

    if (ui->scene_list->currentIndex() >= 0) {
      bool oldState = ui->scene_list->blockSignals(true);
      delete_folder(scenes_path, workcell.scene_vector[ui->scene_list->currentIndex()].name);
      workcell.scene_vector.erase(workcell.scene_vector.begin() + ui->scene_list->currentIndex());
      if (workcell.scene_vector.size() > 0) {
        refresh_scenes(0);
      } else {
        refresh_scenes(-1);
      }
      ui->scene_list->blockSignals(oldState);
    } else {
      ui->error_workcell->append("<font color='red'> No scene to delete! </font>");
    }
  }
}
void SceneSelect::on_edit_scene_clicked()
{
  ui->error_workcell->clear();
  boost::filesystem::current_path(scenes_path);
  if (ui->scene_list->currentIndex() >= 0) {  // Make sure that there are scenes to select
    Scene curr_scene = workcell.scene_vector[ui->scene_list->currentIndex()];
    if (!curr_scene.loaded) {
      if (!load_scene_from_yaml(&curr_scene)) {
        // if scene.loaded is not true, generate scene from yaml
        ui->error_workcell->append("<font color='red'> Could not load scene from YAML </font>");
        return;
      }
    }
    // Scene loaded
    AddScene scene_window;
    scene_window.LoadScene(curr_scene);
    scene_window.setWindowTitle("Edit Scene");
    scene_window.setModal(true);
    scene_window.exec();
    if (scene_window.success) {
      boost::filesystem::current_path(scenes_path);
      if (CheckSceneEqual(scene_window.scene, curr_scene)) {
        refresh_scenes(ui->scene_list->currentIndex());
      } else {  // Scene was edited
        boost::filesystem::current_path(scenes_path);
        boost::filesystem::path scene_yaml_path(
          scenes_path.string() + "/" + scene_window.scene.name);
        if (boost::filesystem::exists(scene_window.scene.name)) {     // Scene name nvr change
          // Replace the current environment yaml
          GenerateYAML::generate_yaml(
            scene_window.scene,
            scene_yaml_path.string(), scenes_path, assets_path);
          generate_scene_files(scene_window.scene);

        } else {
          // Delete previous scene folder
          delete_folder(scenes_path, curr_scene.name);
          // Generate new folder
          generate_scene_package(scenes_path, scene_window.scene.name, workcell.ros_ver);
          GenerateYAML::generate_yaml(
            scene_window.scene,
            scene_yaml_path.string(), scenes_path, assets_path);
          generate_scene_files(scene_window.scene);
        }
        workcell.scene_vector[ui->scene_list->currentIndex()] = scene_window.scene;
        ui->error_workcell->append(
          "<font color='orange'> Warning: Scene has been edited. Previous yaml file removed."
          " Click \" Generate YAML \" again to Generate YAML. </font>");
        refresh_scenes(ui->scene_list->currentIndex());
      }
    } else {
      refresh_scenes(ui->scene_list->currentIndex());
    }
  } else {
    ui->error_workcell->append("<font color='red'> No scene to edit! </font>");
  }
  boost::filesystem::current_path(scenes_path);
}
void SceneSelect::on_generate_yaml_clicked()
{
  ui->error_workcell->clear();
  boost::filesystem::current_path(scenes_path);
  if (ui->scene_list->currentIndex() >= 0) {  // Make sure that there are scenes to select
    boost::filesystem::path scene_yaml_path(
      scenes_path.string() + "/" + workcell.scene_vector[ui->scene_list->currentIndex()].name);
    Scene target_scene = workcell.scene_vector[ui->scene_list->currentIndex()];
    if (!target_scene.loaded) {  // No scene currently loaded
      if (check_yaml()) {    // If yaml file is in folder,
                             // it might get replaced by new scene configuration
        ui->error_workcell->append(
          "<font color='orange'> ERROR: No changes were made to existing scene, "
          "so environment yaml remains the same </font>");
        return;
      } else {   // No yaml in scene folder, no loaded scene from created
        ui->error_workcell->append(
          "<font color='red'> ERROR: No Existing YAML file found, "
          "and no new scene generated. </font>");
        return;
      }
    } else {
      if (check_yaml()) {    // If yaml file is in folder, it might get replaced by new config
        ReplaceWarning replace_window;
        replace_window.setWindowTitle("Edit Scene");
        replace_window.set_label(
          "Warning: Environment yaml currently exists. "
          "Current environment yaml will be replaced. Continue?");
        replace_window.setModal(true);
        replace_window.exec();
        if (replace_window.decision) {      // user allows for replacing of current yaml file
          GenerateYAML::generate_yaml(
            target_scene,
            scene_yaml_path.string(), scenes_path, assets_path);
          ui->error_workcell->clear();
          ui->error_workcell->append("<font color='green'> YAML Generated. </font>");
        }
      } else {   // currently no yaml file, add one to scene folder
        GenerateYAML::generate_yaml(
          target_scene, scene_yaml_path.string(), scenes_path,
          assets_path);
        ui->error_workcell->clear();
        ui->error_workcell->append("<font color='green'> YAML Generated. </font>");
      }
    }
  } else {
    ui->error_workcell->append("<font color='red'> No scene to generate yaml file </font>");
  }
  check_scene();
}
bool SceneSelect::check_yaml()  // Check if scene package has a yaml file to use.
{
  boost::filesystem::current_path(scenes_path);  // in scenes folder
  boost::filesystem::current_path((ui->scene_list->currentText()).toStdString());
  if (!boost::filesystem::exists("environment.yaml")) {
    return false;
  } else {
    ui->edit_scene->setDisabled(false);
  }
  return true;
}
bool SceneSelect::check_scene()
{
  ui->error_workcell->clear();

  bool has_yaml = check_yaml();
  bool files_loaded_proper = check_files();
  if (has_yaml) {
    ui->error_workcell->append(
      "<font color='green'>[Scene Status] "
      "Environment YAML present </font>");
  } else {
    ui->error_workcell->append(
      "<font color='orange'>[Scene Status] Environment YAML not present </font>");
  }

  if (files_loaded_proper) {
    ui->error_workcell->append(
      "<font color='green'>[Scene Status] All files generated properly </font>");
  }

  if (has_yaml && files_loaded_proper) {
    ui->error_workcell->append(
      "<font color='green'>[Scene Status] Scene generation complete."
      " You may exit this application </font>");
  }
  if (!has_yaml && files_loaded_proper) {
    ui->error_workcell->append(
      "<font color='orange'>[Scene Status] Scene generation complete,"
      " but without environment yaml you cannot edit this scene after exit. </font>");
  }
  ui->exit->setDisabled(false);
  boost::filesystem::current_path(scenes_path);
  return true;
}
bool SceneSelect::check_files()
{
  boost::filesystem::current_path(scenes_path);  // in scenes folder
  boost::filesystem::current_path((ui->scene_list->currentText()).toStdString());
  if (!boost::filesystem::exists("launch") || !boost::filesystem::exists("urdf") ||
    !boost::filesystem::exists("CMakeLists.txt") || !boost::filesystem::exists("package.xml"))
  {
    ui->error_workcell->append(
      "<font color='red'>[Scene Status] ERROR: Files not generated properly </font>");

    if (!boost::filesystem::exists("launch")) {
      ui->error_workcell->append(
        "<font color='red'>[Scene Status] ERROR: launch folder missing </font>");
    }
    if (!boost::filesystem::exists("urdf")) {
      ui->error_workcell->append(
        "<font color='red'>[Scene Status] ERROR: urdf folder missing </font>");
    }
    if (!boost::filesystem::exists("CMakeLists.txt")) {
      ui->error_workcell->append(
        "<font color='red'>[Scene Status] ERROR: CMakeLists.txt missing </font>");
    }
    if (!boost::filesystem::exists("package.xml")) {
      ui->error_workcell->append(
        "<font color='red'>[Scene Status] ERROR: Package.xml missing </font>");
    }
    boost::filesystem::current_path(scenes_path);
    return false;
  } else {
    boost::filesystem::current_path("launch");
    if (!boost::filesystem::exists("demo.rviz") || !boost::filesystem::exists("demo.launch.py")) {
      ui->error_workcell->append(
        "<font color='red'>[Scene Status] ERROR: Files not generated properly </font>");
      if (!boost::filesystem::exists("demo.rviz")) {
        ui->error_workcell->append(
          "<font color='red'>[Scene Status] ERROR: demo.rviz missing </font>");
      }
      if (!boost::filesystem::exists("demo.launch.py")) {
        ui->error_workcell->append(
          "<font color='red'>[Scene Status] ERROR: demo.launch.py missing </font>");
      }
      boost::filesystem::current_path(scenes_path);
      return false;
    }

    boost::filesystem::current_path(boost::filesystem::current_path().branch_path());
    boost::filesystem::current_path("urdf");
    if (!boost::filesystem::exists("arm_hand.srdf.xacro") ||
      !boost::filesystem::exists("scene.urdf.xacro"))
    {
      ui->error_workcell->append(
        "<font color='red'>[Scene Status] ERROR: Files not generated properly </font>");
      if (!boost::filesystem::exists("arm_hand.srdf.xacro")) {
        ui->error_workcell->append(
          "<font color='red'>[Scene Status] ERROR: arm_hand.srdf.xacro missing </font>");
      }
      if (!boost::filesystem::exists("scene.urdf.xacro")) {
        ui->error_workcell->append(
          "<font color='red'>[Scene Status] ERROR: scene.urdf.xacro missing </font>");
      }
      boost::filesystem::current_path(scenes_path);
      return false;
    }
  }
  return true;
}
void SceneSelect::on_scene_list_currentIndexChanged(int index)
{
  ui->error_workcell->clear();
  check_scene();
}
void SceneSelect::on_generate_files_clicked()
{
  ui->error_workcell->clear();
  boost::filesystem::current_path(scenes_path);   // Scene folder
  if (ui->scene_list->currentIndex() >= 0) {  // Make sure that there are scenes to select
    Scene curr_scene = workcell.scene_vector[ui->scene_list->currentIndex()];
    if (!curr_scene.loaded) {
      if (!load_scene_from_yaml(&curr_scene)) {
        ui->error_workcell->append("<font color='red'> Could not load scene from YAML </font>");
        return;
      }
    }
    // Generate all environment object packages
    for (Object object : curr_scene.object_vector) {
      // Generate the folders and CMakeLists + Package xmls
      generate_object_package(workcell_path, object, workcell.ros_ver);
      // Generate urdf xacro for object
      boost::filesystem::current_path(
        workcell_path.string() + "/assets/environment/" + object.name + "_description/urdf");
      make_object_xacro(object);
      boost::filesystem::current_path(scenes_path);
    }
    generate_scene_files(curr_scene);
  } else {
    ui->error_workcell->append("<font color='red'> No scene to generate files from </font>");
  }
  check_scene();
}
bool SceneSelect::load_scene_from_yaml(Scene * input_scene)
{
  boost::filesystem::current_path(scenes_path);  // Go back to scene
  try {
    boost::filesystem::current_path(input_scene->name);
  } catch (boost::filesystem::filesystem_error const & e) {
    std::cerr << e.what() << '\n';
    return false;
  }
  YAML::Node yaml;
  // Load Yaml File.
  try {
    yaml = YAML::LoadFile("environment.yaml");
    // std::ifstream f("environment.yaml");
    //    if (f.is_open())
    //        std::cout << f.rdbuf() << std::endl;
  } catch (YAML::BadFile & error) {
    ui->error_workcell->append(
      "<font color='red'> Something went wrong with the yaml file."
      " Please Generate it again </font>");
    return false;
  }

  // TODO(Glenn): add error catch if trying to create directory with same name,
  // or maybe add a number to it, eg table1, table2.
  YAML::Node objects;
  YAML::Node ext_joints;
  bool has_objects;


  for (YAML::iterator it = yaml.begin(); it != yaml.end(); ++it) {
    std::string key = it->first.as<std::string>();
    if (key.compare("robot") == 0) {
      Robot robot;
      SceneParser::LoadRobotFromYAML(&robot, it->second);
      input_scene->robot_loaded = true;
      input_scene->robot_vector.clear();
      input_scene->robot_vector.push_back(robot);
    }
    if (key.compare("end_effector") == 0) {
      EndEffector ee;
      SceneParser::LoadEEFromYAML(&ee, it->second);
      input_scene->ee_loaded = true;
      input_scene->ee_vector.clear();
      input_scene->ee_vector.push_back(ee);
    }
    if (key.compare("objects") == 0) {
      objects = it->second;
      has_objects = true;
    }
    if (key.compare("external joints") == 0) {
      ext_joints = it->second;
    }
  }

  if (has_objects) {  // We need to do this because the object field needs to load before
                      // the ext joint field, and it currently has a random load order
    for (YAML::iterator objects_it = objects.begin(); objects_it != objects.end(); ++objects_it) {
      Object temp_object;
      temp_object.name = objects_it->first.as<std::string>();
      YAML::Node ext_joint;
      temp_object.ext_joint.child_object = objects_it->first.as<std::string>();
      for (YAML::iterator in_object_it = objects_it->second.begin();
        in_object_it != objects_it->second.end(); ++in_object_it)
      {
        if (in_object_it->first.as<std::string>().compare("links") == 0) {
          std::vector<Link> temp_link_vector;
          SceneParser::LoadLinksFromYAML(&temp_link_vector, in_object_it->second);
          temp_object.link_vector = temp_link_vector;
        }
        if (in_object_it->first.as<std::string>().compare("joints") == 0) {
          std::vector<Joint> temp_joint_vector;
          YAML::Node joints = in_object_it->second;
          SceneParser::LoadJointsFromYAML(
            &temp_joint_vector, temp_object.link_vector,
            in_object_it->second);
          temp_object.joint_vector = temp_joint_vector;
        }
        if (in_object_it->first.as<std::string>().compare(temp_object.name + "_base_joint") == 0) {
          temp_object.ext_joint.name = in_object_it->first.as<std::string>();
          ext_joint = in_object_it->second;
        }
      }

      for (YAML::iterator in_ext_joint_it = ext_joint.begin(); in_ext_joint_it != ext_joint.end();
        ++in_ext_joint_it)
      {
        if (in_ext_joint_it->first.as<std::string>().compare("ext_joint_type") == 0) {
          temp_object.ext_joint.type = in_ext_joint_it->second.as<std::string>();
        }
        if (in_ext_joint_it->first.as<std::string>().compare("child_link") == 0) {
          // Get Child Link pos
          std::string child_link = in_ext_joint_it->second.as<std::string>();
          for (int i = 0; i < static_cast<int>(temp_object.link_vector.size()); i++) {
            if (child_link.compare(temp_object.link_vector[i].name) == 0) {
              temp_object.ext_joint.child_link_pos = i;
              break;
            }
          }
        }
      }
      input_scene->object_vector.push_back(temp_object);
    }

    int counter = 0;
    for (YAML::iterator ext_joints_it = ext_joints.begin(); ext_joints_it != ext_joints.end();
      ++ext_joints_it)
    {
      input_scene->object_vector[counter].ext_joint.origin.is_origin = false;
      input_scene->object_vector[counter].ext_joint.axis.is_axis = false;
      YAML::Node in_ext_joints = ext_joints_it->second;
      for (YAML::iterator in_ext_joints_it = in_ext_joints.begin();
        in_ext_joints_it != in_ext_joints.end(); ++in_ext_joints_it)
      {
        if (in_ext_joints_it->first.as<std::string>().compare("parent object") == 0) {
          std::string parent_object = in_ext_joints_it->second.as<std::string>();
          if (parent_object.compare("world") == 0) {       // if world pos is -1
            input_scene->object_vector[counter].ext_joint.parent_obj_pos = -1;
          } else {
            for (int i = 0; i < static_cast<int>(input_scene->object_vector.size()); i++) {
              if (parent_object.compare(input_scene->object_vector[i].name) == 0) {
                input_scene->object_vector[counter].ext_joint.parent_obj_pos = i;
              }
            }
          }
        }
        if (in_ext_joints_it->first.as<std::string>().compare("parent link") == 0) {
          // Get Parent link pos
          std::string parent_link = in_ext_joints_it->second.as<std::string>();
          int parent_obj_pos = input_scene->object_vector[counter].ext_joint.parent_obj_pos;
          if (parent_obj_pos >= 0) {
            for (int i = 0;
              i < static_cast<int>(input_scene->object_vector[parent_obj_pos].link_vector.size());
              i++)
            {
              if (parent_link.compare(
                  input_scene->object_vector[parent_obj_pos].
                  link_vector[i].name)
                ==
                0)
              {
                input_scene->object_vector[counter].ext_joint.parent_link_pos = i;
                break;
              }
            }
          }
        }
        if (in_ext_joints_it->first.as<std::string>().compare("origin") == 0) {
          input_scene->object_vector[counter].ext_joint.origin.is_origin = true;
          SceneParser::LoadOriginFromYAML(
            &(input_scene->object_vector[counter].ext_joint.origin),
            in_ext_joints_it->second);
        }
        if (in_ext_joints_it->first.as<std::string>().compare("axis") == 0) {
          input_scene->object_vector[counter].ext_joint.axis.is_axis = true;
          SceneParser::LoadAxisFromYAML(
            &(input_scene->object_vector[counter].ext_joint.axis),
            in_ext_joints_it->second);
        }
      }
      counter++;
    }
  }

  input_scene->loaded = true;
  boost::filesystem::current_path(scenes_path);
  return true;
}
void SceneSelect::on_back_clicked()
{
  if (workcell.scene_vector[ui->scene_list->currentIndex()].loaded && !check_yaml()) {
    ReplaceWarning replace_window;
    replace_window.setWindowTitle("Edit Scene");
    replace_window.set_label(
      "Warning: Currently loaded scene is not saved."
      " All progress will be lost. Generate yaml file before going back.");
    replace_window.setModal(true);
    replace_window.exec();

    if (replace_window.decision) {
      this->close();
    }
  }
}

void SceneSelect::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}

void SceneSelect::on_exit_clicked()
{
  QApplication::quit();
}
