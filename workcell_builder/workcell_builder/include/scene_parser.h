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

#ifndef SCENE_PARSER_H_
#define SCENE_PARSER_H_


#include <boost/filesystem.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "attributes/workcell.h"

class SceneParser
{
public:
  static bool LoadRobotFromYAML(Robot * input_robot, YAML::Node yaml_robot)
  {
    for (YAML::iterator robot_it = yaml_robot.begin(); robot_it != yaml_robot.end(); ++robot_it) {
      if (robot_it->first.as < std::string > ().compare("name") == 0) {
        input_robot->name = robot_it->second.as < std::string > ();
      }
      if (robot_it->first.as < std::string > ().compare("brand") == 0) {
        input_robot->brand = robot_it->second.as < std::string > ();
      }
      if (robot_it->first.as < std::string > ().compare("filepath") == 0) {
        input_robot->filepath = robot_it->second.as < std::string > ();
      }
      if (robot_it->first.as < std::string > ().compare("base_link") == 0) {
        input_robot->base_link = robot_it->second.as < std::string > ();
      }
      if (robot_it->first.as < std::string > ().compare("ee_link") == 0) {
        input_robot->ee_link = robot_it->second.as < std::string > ();
      }
      if (robot_it->first.as < std::string > ().compare("origin") == 0) {
        input_robot->origin.is_origin = true;
        YAML::Node robot_node = robot_it->second;
        for (YAML::iterator origin_it = robot_node.begin(); origin_it != robot_node.end();
          ++origin_it)
        {
          if (origin_it->first.as < std::string > ().compare("x") == 0) {
            // input_robot->origin.x = std::stof(origin_it->second.as<std::string>());
            input_robot->origin.x = origin_it->second.as < float > ();
          }
          if (origin_it->first.as < std::string > ().compare("y") == 0) {
            input_robot->origin.y = origin_it->second.as < float > ();
          }
          if (origin_it->first.as < std::string > ().compare("z") == 0) {
            input_robot->origin.z = origin_it->second.as < float > ();
          }
          if (origin_it->first.as < std::string > ().compare("roll") == 0) {
            input_robot->origin.roll = origin_it->second.as < float > ();
          }
          if (origin_it->first.as < std::string > ().compare("pitch") == 0) {
            input_robot->origin.pitch = origin_it->second.as < float > ();
          }
          if (origin_it->first.as < std::string > ().compare("yaw") == 0) {
            input_robot->origin.yaw = origin_it->second.as < float > ();
          }
        }
      }

      if (robot_it->first.as < std::string > ().compare("links") == 0) {
        YAML::Node robot_links_ = robot_it->second;
        std::vector < std::string > temp_vec;
        for (YAML::iterator robot_links_it = robot_links_.begin();
          robot_links_it != robot_links_.end(); ++robot_links_it)
        {
          temp_vec.push_back(robot_links_it->as < std::string > ());
        }
        input_robot->robot_links = temp_vec;
      }
    }
    return true;
  }
  static bool LoadEEFromYAML(EndEffector * input_ee, YAML::Node yaml_ee)
  {
    input_ee->origin.is_origin = false;
    for (YAML::iterator ee_it = yaml_ee.begin(); ee_it != yaml_ee.end(); ++ee_it) {
      if (ee_it->first.as < std::string > ().compare("name") == 0) {
        input_ee->name = ee_it->second.as < std::string > ();
      }
      if (ee_it->first.as < std::string > ().compare("brand") == 0) {
        input_ee->brand = ee_it->second.as < std::string > ();
      }
      if (ee_it->first.as < std::string > ().compare("base_link") == 0) {
        input_ee->base_link = ee_it->second.as < std::string > ();
      }
      if (ee_it->first.as < std::string > ().compare("filepath") == 0) {
        input_ee->filepath = ee_it->second.as < std::string > ();
      }
      if (ee_it->first.as < std::string > ().compare("robot_link") == 0) {
        input_ee->robot_link = ee_it->second.as < std::string > ();
      }
      if (ee_it->first.as < std::string > ().compare("ee_type") == 0) {
        input_ee->ee_type = ee_it->second.as < std::string > ();
      }

      if (ee_it->first.as < std::string > ().compare("attributes") == 0) {
        YAML::Node attributes = ee_it->second;
        for (YAML::iterator attributes_it = attributes.begin(); attributes_it != attributes.end();
          ++attributes_it)
        {
          if (attributes_it->first.as < std::string > ().compare("fingers") == 0) {
            input_ee->attribute_1 = attributes_it->second.as < int > ();
          }
          if (attributes_it->first.as < std::string > ().compare("array_width") == 0) {
            input_ee->attribute_1 = attributes_it->second.as < int > ();
          }
          if (attributes_it->first.as < std::string > ().compare("array_height") == 0) {
            input_ee->attribute_2 = attributes_it->second.as < int > ();
          }
        }
      }
      if (ee_it->first.as < std::string > ().compare("origin") == 0) {
        input_ee->origin.is_origin = true;
        YAML::Node ee_node1 = ee_it->second;
        for (YAML::iterator origin_it2 = ee_node1.begin(); origin_it2 != ee_node1.end();
          ++origin_it2)
        {
          if (origin_it2->first.as < std::string > ().compare("x") == 0) {
            input_ee->origin.x = origin_it2->second.as < float > ();
          }
          if (origin_it2->first.as < std::string > ().compare("y") == 0) {
            input_ee->origin.y = origin_it2->second.as < float > ();
          }
          if (origin_it2->first.as < std::string > ().compare("z") == 0) {
            input_ee->origin.z = origin_it2->second.as < float > ();
          }
          if (origin_it2->first.as < std::string > ().compare("roll") == 0) {
            input_ee->origin.roll = origin_it2->second.as < float > ();
          }
          if (origin_it2->first.as < std::string > ().compare("pitch") == 0) {
            input_ee->origin.pitch = origin_it2->second.as < float > ();
          }
          if (origin_it2->first.as < std::string > ().compare("yaw") == 0) {
            input_ee->origin.yaw = origin_it2->second.as < float > ();
          }
        }
      }
      if (ee_it->first.as < std::string > ().compare("links") == 0) {
        YAML::Node ee_links_ = ee_it->second;
        std::vector < std::string > temp_vec;

        for (YAML::iterator ee_links_it = ee_links_.begin(); ee_links_it != ee_links_.end();
          ++ee_links_it)
        {
          temp_vec.push_back(ee_links_it->as < std::string > ());
        }
        input_ee->ee_links = temp_vec;
      }
    }
    return true;
  }
  static bool LoadLinksFromYAML(std::vector < Link > * input_vec, YAML::Node yaml_link)
  {
    for (YAML::iterator links_it = yaml_link.begin(); links_it != yaml_link.end(); ++links_it) {
      Link temp_link;
      temp_link.is_visual = false;
      temp_link.is_inertial = false;
      temp_link.is_collision = false;
      if (links_it->first.as < std::string > ().compare("None") == 0) {
        temp_link.name = "";
      } else {
        temp_link.name = links_it->first.as < std::string > ();
      }
      for (YAML::iterator in_links_it = links_it->second.begin();
        in_links_it != links_it->second.end(); ++in_links_it)
      {
        if (in_links_it->first.as < std::string > ().compare("visual") == 0) {
          Visual temp_visual;
          temp_link.is_visual = true;
          LoadVisualFromYAML(&temp_visual, in_links_it->second);
          temp_link.visual_vector.push_back(temp_visual);
        }

        if (in_links_it->first.as < std::string > ().compare("collision") == 0) {
          Collision temp_collision;
          temp_link.is_collision = true;
          LoadCollisionFromYAML(&temp_collision, in_links_it->second);
          temp_link.collision_vector.push_back(temp_collision);
        }
        if (in_links_it->first.as < std::string > ().compare("inertial") == 0) {
          Inertial temp_inertial;
          temp_link.is_inertial = true;
          LoadInertialFromYAML(&temp_inertial, in_links_it->second);
          temp_link.inertial_vector.push_back(temp_inertial);
        }
      }
      input_vec->push_back(temp_link);
    }
    return true;
  }
  static bool LoadJointsFromYAML(
    std::vector < Joint > * input_vec, std::vector < Link > link_vec,
    YAML::Node yaml_joint)
  {
    for (YAML::iterator joint_it = yaml_joint.begin(); joint_it != yaml_joint.end(); ++joint_it) {
      Joint temp_joint;
      temp_joint.origin.is_origin = false;
      temp_joint.axis.is_axis = false;
      if (joint_it->first.as < std::string > ().compare("None") == 0) {
        temp_joint.name = "";
      } else {
        temp_joint.name = joint_it->first.as < std::string > ();
      }
      for (YAML::iterator in_joints_it = joint_it->second.begin();
        in_joints_it != joint_it->second.end(); ++in_joints_it)
      {
        if (in_joints_it->first.as < std::string > ().compare("type") == 0) {
          temp_joint.type = in_joints_it->second.as < std::string > ();
        }
        if (in_joints_it->first.as < std::string > ().compare("parent") == 0) {
          for (int i = 0; i < static_cast < int > (link_vec.size()); i++) {
            if (link_vec[i].name.compare(in_joints_it->second.as < std::string > ()) == 0) {
              temp_joint.parent_link = link_vec[i];
            }
          }
        }
        if (in_joints_it->first.as < std::string > ().compare("child") == 0) {
          for (int i = 0; i < static_cast < int > (link_vec.size()); i++) {
            if (link_vec[i].name.compare(in_joints_it->second.as < std::string > ()) == 0) {
              temp_joint.child_link = link_vec[i];
            }
          }
        }
        if (in_joints_it->first.as < std::string > ().compare("origin") == 0) {
          LoadOriginFromYAML(&(temp_joint.origin), in_joints_it->second);
        }
        if (in_joints_it->first.as < std::string > ().compare("axis") == 0) {
          LoadAxisFromYAML(&(temp_joint.axis), in_joints_it->second);
        }
      }
      input_vec->push_back(temp_joint);
    }
    return true;
  }
  static bool LoadVisualFromYAML(Visual * input_visual, YAML::Node yaml_visual)
  {
    input_visual->origin.is_origin = false;
    input_visual->is_origin = false;
    for (YAML::iterator visual_it = yaml_visual.begin(); visual_it != yaml_visual.end();
      ++visual_it)
    {
      if (visual_it->first.as < std::string > ().compare("name") == 0) {
        if (visual_it->second.as < std::string > ().compare("None") == 0 ||
          visual_it->second.as < std::string > ().compare("") == 0)
        {
          input_visual->name = "None";
        } else {
          input_visual->name = visual_it->second.as < std::string > ();
        }
      }
      if (visual_it->first.as < std::string > ().compare("origin") == 0) {
        input_visual->origin.is_origin = true;
        input_visual->is_origin = true;
        LoadOriginFromYAML(&(input_visual->origin), visual_it->second);
      }
      if (visual_it->first.as < std::string > ().compare("geometry") == 0) {
        LoadGeometryFromYAML(&(input_visual->geometry), visual_it->second);
      }
      if (visual_it->first.as < std::string > ().compare("material") == 0) {
        input_visual->is_material = true;
        LoadMaterialFromYAML(&(input_visual->material), visual_it->second);
      }
    }
    return true;
  }
  static bool LoadCollisionFromYAML(Collision * input_collision, YAML::Node yaml_collision)
  {
    input_collision->origin.is_origin = false;
    for (YAML::iterator collision_it = yaml_collision.begin(); collision_it != yaml_collision.end();
      ++collision_it)
    {
      if (collision_it->first.as < std::string > ().compare("name") == 0) {
        if (collision_it->second.as < std::string > ().compare("None") == 0 ||
          collision_it->second.as < std::string > ().compare("") == 0)
        {
          input_collision->name = "None";
        } else {
          input_collision->name = yaml_collision["name"].as < std::string > ();
        }
      }
      if (collision_it->first.as < std::string > ().compare("origin") == 0) {
        input_collision->origin.is_origin = true;
        LoadOriginFromYAML(&(input_collision->origin), collision_it->second);
      }
      if (collision_it->first.as < std::string > ().compare("geometry") == 0) {
        LoadGeometryFromYAML(&(input_collision->geometry), collision_it->second);
      }
    }
    return true;
  }
  static bool LoadInertialFromYAML(Inertial * input_inertial, YAML::Node yaml_inertial)
  {
    for (YAML::iterator inertial_it = yaml_inertial.begin(); inertial_it != yaml_inertial.end();
      ++inertial_it)
    {
      if (inertial_it->first.as < std::string > ().compare("mass") == 0) {
        input_inertial->mass = inertial_it->second.as < float > ();
      }
      if (inertial_it->first.as < std::string > ().compare("inertia") == 0) {
        std::string ixx, ixy, ixz, iyy, iyz, izz;
        for (YAML::iterator inertia_it = inertial_it->second.begin();
          inertia_it != inertial_it->second.end(); ++inertia_it)
        {
          if (inertia_it->first.as < std::string > ().compare("ixx") == 0) {
            input_inertial->ixx = std::stof(inertia_it->second.as < std::string > ());
          }
          if (inertia_it->first.as < std::string > ().compare("ixy") == 0) {
            input_inertial->ixy = std::stof(inertia_it->second.as < std::string > ());
          }
          if (inertia_it->first.as < std::string > ().compare("ixz") == 0) {
            input_inertial->ixz = std::stof(inertia_it->second.as < std::string > ());
          }

          if (inertia_it->first.as < std::string > ().compare("iyy") == 0) {
            input_inertial->iyy = std::stof(inertia_it->second.as < std::string > ());
          }
          if (inertia_it->first.as < std::string > ().compare("iyz") == 0) {
            input_inertial->iyz = std::stof(inertia_it->second.as < std::string > ());
          }
          if (inertia_it->first.as < std::string > ().compare("izz") == 0) {
            input_inertial->izz = std::stof(inertia_it->second.as < std::string > ());
          }
        }
      }
      if (inertial_it->first.as < std::string > ().compare("origin") == 0) {
        // input_inertial->is_origin = true;
        LoadOriginFromYAML(&(input_inertial->origin), inertial_it->second);
      }
    }
    return true;
  }
  static bool LoadGeometryFromYAML(Geometry * input_geometry, YAML::Node yaml_geometry)
  {
    for (YAML::iterator geometry_it = yaml_geometry.begin(); geometry_it != yaml_geometry.end();
      ++geometry_it)
    {
      if (geometry_it->first.as < std::string > ().compare("filepath") == 0) {
        input_geometry->disableShape();
        input_geometry->filepath = geometry_it->second.as < std::string > ();
        input_geometry->filepath_new = geometry_it->second.as < std::string > ();
      }
      if (geometry_it->first.as < std::string > ().compare("scale") == 0) {
        YAML::Node scale = geometry_it->second;
        for (YAML::iterator scale_it = scale.begin(); scale_it != scale.end(); ++scale_it) {
          if (scale_it->first.as < std::string > ().compare("scale_x") == 0) {
            input_geometry->scale_x = scale_it->second.as < float > ();
          }
          if (scale_it->first.as < std::string > ().compare("scale_y") == 0) {
            input_geometry->scale_y = scale_it->second.as < float > ();
          }
          if (scale_it->first.as < std::string > ().compare("scale_z") == 0) {
            input_geometry->scale_z = scale_it->second.as < float > ();
          }
        }
      }
      if (geometry_it->first.as < std::string > ().compare("shape") == 0) {
        input_geometry->disableSTL();
        input_geometry->shape = geometry_it->second.as < std::string > ();
      }
      if (geometry_it->first.as < std::string > ().compare("length") == 0) {
        input_geometry->length = geometry_it->second.as < float > ();
      }
      if (geometry_it->first.as < std::string > ().compare("breadth") == 0) {
        input_geometry->breadth = geometry_it->second.as < float > ();
      }
      if (geometry_it->first.as < std::string > ().compare("height") == 0) {
        input_geometry->height = geometry_it->second.as < float > ();
      }
      if (geometry_it->first.as < std::string > ().compare("radius") == 0) {
        input_geometry->radius = geometry_it->second.as < float > ();
      }
    }
    return true;
  }
  static bool LoadMaterialFromYAML(Material * input_material, YAML::Node yaml_material)
  {
    input_material->is_material = true;
    input_material->is_texture = false;
    for (YAML::iterator material_it = yaml_material.begin(); material_it != yaml_material.end();
      ++material_it)
    {
      if (material_it->first.as < std::string > ().compare("name") == 0) {
        input_material->material_name = material_it->second.as < std::string > ();
      }
      if (material_it->first.as < std::string > ().compare("filepath") == 0) {
        input_material->disableColor();
        input_material->filepath = material_it->second.as < std::string > ();
      }
      if (material_it->first.as < std::string > ().compare("r") == 0) {
        input_material->r = material_it->second.as < float > ();
      }
      if (material_it->first.as < std::string > ().compare("g") == 0) {
        input_material->g = material_it->second.as < float > ();
      }
      if (material_it->first.as < std::string > ().compare("b") == 0) {
        input_material->b = material_it->second.as < float > ();
      }
      if (material_it->first.as < std::string > ().compare("a") == 0) {
        input_material->a = material_it->second.as < float > ();
      }
    }
    return true;
  }
  static bool LoadOriginFromYAML(Origin * input_origin, YAML::Node yaml_origin)
  {
    input_origin->is_origin = true;
    for (YAML::iterator origin_it = yaml_origin.begin(); origin_it != yaml_origin.end();
      ++origin_it)
    {
      if (origin_it->first.as < std::string > ().compare("x") == 0) {
        input_origin->x = origin_it->second.as < float > ();
      }

      if (origin_it->first.as < std::string > ().compare("y") == 0) {
        input_origin->y = origin_it->second.as < float > ();
      }

      if (origin_it->first.as < std::string > ().compare("z") == 0) {
        input_origin->z = origin_it->second.as < float > ();
      }

      if (origin_it->first.as < std::string > ().compare("roll") == 0) {
        input_origin->roll = origin_it->second.as < float > ();
      }

      if (origin_it->first.as < std::string > ().compare("pitch") == 0) {
        input_origin->pitch = origin_it->second.as < float > ();
      }

      if (origin_it->first.as < std::string > ().compare("yaw") == 0) {
        input_origin->yaw = origin_it->second.as < float > ();
      }
    }
    return true;
  }
  static bool LoadAxisFromYAML(Axis * input_axis, YAML::Node yaml_axis)
  {
    input_axis->is_axis = true;
    for (YAML::iterator axis_it = yaml_axis.begin(); axis_it != yaml_axis.end(); ++axis_it) {
      if (axis_it->first.as < std::string > ().compare("x") == 0) {
        input_axis->x = axis_it->second.as < float > ();
      }

      if (axis_it->first.as < std::string > ().compare("y") == 0) {
        input_axis->y = axis_it->second.as < float > ();
      }

      if (axis_it->first.as < std::string > ().compare("z") == 0) {
        input_axis->z = axis_it->second.as < float > ();
      }
    }
    return true;
  }
};

#endif  // SCENE_PARSER_H_
