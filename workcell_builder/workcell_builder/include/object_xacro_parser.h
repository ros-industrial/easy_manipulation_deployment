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


#ifndef OBJECT_XACRO_PARSER_H_
#define OBJECT_XACRO_PARSER_H_


#include <iostream>
#include <fstream>
#include <string>

#include "attributes/workcell.h"


void StartXacro(std::ofstream * MyFile, std::string name);
void NewObject(std::ofstream * MyFile, Object object);
void NewLink(std::ofstream * MyFile, Link link, std::string object_name);
void NewVisual(std::ofstream * MyFile, Visual visual, std::string object_name);
void NewGeometry(std::ofstream * MyFile, Geometry geometry, std::string object_name);
void NewMaterial(std::ofstream * MyFile, Material material);
void NewCollision(std::ofstream * MyFile, Collision collision, std::string object_name);
void NewInertial(std::ofstream * MyFile, Inertial inertial);
void NewJoint(std::ofstream * MyFile, Joint joint);
void NewExtJoint(std::ofstream * MyFile, ExternalJoint joint, Object object);
void NewAxis(std::ofstream * MyFile, Axis axis);
void NewOrigin(std::ofstream * MyFile, Origin origin, std::string space);
void EndXacro(std::ofstream * MyFile);

void make_object_xacro(Object object)  // Assumes that we are currently in the urdf folders
{
  std::ofstream MyFile(object.name + ".urdf.xacro");
  NewObject(&MyFile, object);
}

void StartXacro(std::ofstream * MyFile, std::string name)
{
  // Create the xacro file

  *MyFile << "<?xml version=\"1.0\" ?> \n";
  *MyFile << "<robot xmlns:xacro=\"http://wiki.ros.org/xacro\"> \n";
  *MyFile << "<xacro:macro name=\"" + name + "\"" + " params=\"parent *origin prefix\"> \n";
}

void NewObject(std::ofstream * MyFile, Object object)
{
  StartXacro(MyFile, object.name);
  for (Link link : object.link_vector) {
    NewLink(MyFile, link, object.name);
  }
  for (Joint joint : object.joint_vector) {
    NewJoint(MyFile, joint);
  }
  NewExtJoint(MyFile, object.ext_joint, object);
  EndXacro(MyFile);
}

void NewLink(std::ofstream * MyFile, Link link, std::string object_name)
{
  *MyFile << "\t<link name=\"" + link.name + "_${prefix}\">\n";
  if (link.is_visual) {
    NewVisual(MyFile, link.visual_vector[0], object_name);
  }
  if (link.is_collision) {
    NewCollision(MyFile, link.collision_vector[0], object_name);
  }
  if (link.is_inertial) {
    NewInertial(MyFile, link.inertial_vector[0]);
  }
  *MyFile << "\t</link>\n";
}

void NewVisual(std::ofstream * MyFile, Visual visual, std::string object_name)
{
  if (visual.name.find_first_not_of(' ') == std::string::npos) {  // No name
    *MyFile << "\t  <visual>\n";
  } else {
    *MyFile << "\t  <visual name=\"" + visual.name + "_${prefix}\">\n";
  }
  if (visual.is_origin) {
    NewOrigin(MyFile, visual.origin, "\t    ");
  }
  NewGeometry(MyFile, visual.geometry, object_name);
  if (visual.is_material) {
    NewMaterial(MyFile, visual.material);
  }
  *MyFile << "\t  </visual>\n";
}
void NewGeometry(std::ofstream * MyFile, Geometry geometry, std::string object_name)
{
  *MyFile << "\t    <geometry>\n";
  if (geometry.is_stl) {
    std::string object_filename;
    for (auto it = geometry.filepath.crbegin(); it != geometry.filepath.crend(); ++it) {
      if (*it != '/') {
        object_filename = std::string(1, *it) + object_filename;
      } else {
        break;
      }
    }
    std::string filepath = "\t      <mesh filename=\"package://" + object_name +
      "_description/meshes/visual/" + object_filename + "\"";
    std::string scale_output = " scale=\"" + std::to_string(geometry.scale_x) + " " +
      std::to_string(geometry.scale_y) + " " + std::to_string(geometry.scale_z) + "\"/>\n";
    *MyFile << (filepath + scale_output);
  } else {  // use shape
    if (geometry.shape.compare("Sphere") == 0) {
      *MyFile << "\t      <sphere radius=\"" + std::to_string(geometry.radius) + "\"/>\n";
    } else if (geometry.shape.compare("Box") == 0) {
      *MyFile << "\t      <box size=\"" + std::to_string(geometry.length) + " " + std::to_string(
        geometry.breadth) + " " + std::to_string(geometry.height) + "\"/>\n";
    } else if (geometry.shape.compare("Cylinder") == 0) {
      *MyFile << "\t      <cylinder radius=\"" + std::to_string(geometry.radius) + "\" length=\"" +
        std::to_string(geometry.height) + "\"/>\n";
    }
  }
  *MyFile << "\t    </geometry>\n";
}
void NewMaterial(std::ofstream * MyFile, Material material)
{
  *MyFile << "\t    <material name=\"" + material.material_name + "\">\n";
  if (material.is_texture) {
    *MyFile << "\t      <texture filename=\"" + material.filepath + "\"/>\n";
  } else {
    *MyFile << "\t      <color rgba=\"" + std::to_string(material.r) + " " + std::to_string(
      material.g) + " " + std::to_string(material.b) + " " + std::to_string(material.a) + "\"/>\n";
  }
  *MyFile << "\t    </material>\n";
}
void NewCollision(std::ofstream * MyFile, Collision collision, std::string object_name)
{
  if (collision.name.find_first_not_of(' ') == std::string::npos) {  // No name
    *MyFile << "\t  <collision>\n";
  } else {
    *MyFile << "\t  <collision name=\"" + collision.name + "_${prefix}\">\n";
  }
  if (collision.origin.is_origin) {
    NewOrigin(MyFile, collision.origin, "\t    ");
  }
  NewGeometry(MyFile, collision.geometry, object_name);
  *MyFile << "\t  </collision>\n";
}
void NewInertial(std::ofstream * MyFile, Inertial inertial)
{
  *MyFile << "\t  <inertial>\n";
  if (inertial.origin.is_origin) {
    NewOrigin(MyFile, inertial.origin, "\t    ");
  }
  *MyFile << "\t   <mass value=\"" + std::to_string(inertial.mass) + "\"/>\n";
  *MyFile << "\t   <inertia ixx=\"" + std::to_string(inertial.ixx) + "\" ixy=\"" + std::to_string(
    inertial.ixy) + "\" ixz=\"" + std::to_string(inertial.ixz) + "\" iyy=\"" + std::to_string(
    inertial.iyy) + "\" iyz=\"" + std::to_string(inertial.iyz) + "\" izz=\"" + std::to_string(
    inertial.izz) + "\"/>\n";
  *MyFile << "\t  </inertial>\n";
}
void NewJoint(std::ofstream * MyFile, Joint joint)
{
  *MyFile << "\t<joint name=\"" + joint.name + "_${prefix}\" type=\"" + joint.type + "\">\n";
  *MyFile << "\t  <parent link=\"" + joint.parent_link.name + "_${prefix}\"/>\n";
  *MyFile << "\t  <child link=\"" + joint.child_link.name + "_${prefix}\"/>\n";
  if (joint.origin.is_origin) {
    NewOrigin(MyFile, joint.origin, "\t    ");
  }
  *MyFile << "\t</joint>\n";
}
void NewExtJoint(std::ofstream * MyFile, ExternalJoint joint, Object object)
{
  *MyFile << "\t<joint name=\"" + joint.name + "_${prefix}\" type=\"" + joint.type + "\">\n";
  *MyFile << "\t  <xacro:insert_block name=\"origin\" />\n";
  *MyFile << "\t  <parent link=\"${parent}\"/>\n";
  *MyFile << "\t  <child link=\"" + object.link_vector[joint.child_link_pos].name +
    "_${prefix}\"/>\n";
  *MyFile << "\t</joint>\n";
}
void NewAxis(std::ofstream * MyFile, Axis axis)
{
  *MyFile << "\t   <axis xyz=\"" + std::to_string(axis.x) + " " + std::to_string(axis.y) + " " +
    std::to_string(axis.z) + "\"/>\n";
}
void NewOrigin(std::ofstream * MyFile, Origin origin, std::string space)
{
  *MyFile << space + "<origin xyz=\"" + std::to_string(origin.x) + " " + std::to_string(origin.y) +
    " " + std::to_string(origin.z) + "\"" + " rpy=\"" + std::to_string(origin.roll) + " " +
    std::to_string(origin.pitch) + " " + std::to_string(origin.yaw) + "\"/>\n";
}
void EndXacro(std::ofstream * MyFile)
{
  *MyFile << "</xacro:macro>\n";
  *MyFile << "</robot> \n";
}

#endif  // OBJECT_XACRO_PARSER_H_
