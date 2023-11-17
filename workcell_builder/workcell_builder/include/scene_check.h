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


#ifndef SCENE_CHECK_H_
#define SCENE_CHECK_H_

#include <iostream>

#include "attributes/workcell.h"


bool CheckSceneEqual(Scene scene_1, Scene scene_2);
bool CheckRobotEqual(Robot robot_1, Robot robot_2);
bool CheckEEEqual(EndEffector ee_1, EndEffector ee_2);
bool CheckObjectEqual(Object object_1, Object object_2);
bool CheckLinkEqual(Link link_1, Link link_2);
bool CheckVisualEqual(Visual visual_1, Visual visual_2);
bool CheckCollisionEqual(Collision collision_1, Collision collision_2);
bool CheckInertiaEqual(Inertial inertial_1, Inertial inertial_2);
bool CheckJointEqual(Joint joint_1, Joint joint_2);
bool CheckOriginEqual(Origin origin_1, Origin origin_2);
bool CheckAxisEqual(Axis axis_1, Axis axis_2);
bool CheckGeometryEqual(Geometry geometry_1, Geometry geometry_2);
bool CheckMaterialEqual(Material material_1, Material material_2);
bool CheckExtJointEqual(ExternalJoint extjoint_1, ExternalJoint extjoint_2);


bool CheckSceneEqual(Scene scene_1, Scene scene_2)
{
  if (scene_1.name.compare(scene_2.name) != 0) {
    std::cout << "name not identical" << std::endl;
    return false;
  }
  if (scene_1.filepath.compare(scene_2.filepath) != 0) {
    std::cout << "filepath not identical" << std::endl;
    return false;
  }
  if (scene_1.ee_loaded != scene_2.ee_loaded) {
    std::cout << "ee_loaded not identical" << std::endl;
    return false;
  }
  if (scene_1.robot_loaded != scene_2.robot_loaded) {
    std::cout << "robot_loaded not identical" << std::endl;
    return false;
  }
  if (scene_1.loaded != scene_2.loaded) {
    std::cout << "loaded identical" << std::endl;
    return false;
  }
  if (static_cast < int > (scene_1.robot_vector.size()) !=
    static_cast < int > (scene_2.robot_vector.size()))
  {
    std::cout << "robot_vector size not identical" << std::endl;
    return false;
  }
  if (static_cast < int > (scene_1.object_vector.size()) !=
    static_cast < int > (scene_2.object_vector.size()))
  {
    std::cout << "object_vector size not identical" << std::endl;
    return false;
  }
  if (static_cast < int > (scene_1.ee_vector.size()) !=
    static_cast < int > (scene_2.ee_vector.size()))
  {
    std::cout << "ee_vector size not identical" << std::endl;
    return false;
  }
  if (static_cast < int > (scene_1.parent_objects.size()) !=
    static_cast < int > (scene_2.parent_objects.size()))
  {
    std::cout << "parent_objects size not identical" << std::endl;
    return false;
  }
  if (static_cast < int > (scene_1.child_objects.size()) !=
    static_cast < int > (scene_2.child_objects.size()))
  {
    std::cout << "child_objects size not identical" << std::endl;
    return false;
  }
  if (scene_1.object_vector.size() > 0) {
    for (int i = 0; i < static_cast < int > (scene_1.object_vector.size()); i++) {
      if (!CheckObjectEqual(scene_1.object_vector[i], scene_2.object_vector[i])) {
        std::cout << "objects not identical" << std::endl;
        return false;
      }
    }
  }

  if (scene_1.robot_vector.size() > 0) {
    for (int i = 0; i < static_cast < int > (scene_1.robot_vector.size()); i++) {
      if (!CheckRobotEqual(scene_1.robot_vector[i], scene_2.robot_vector[i])) {
        std::cout << "robots not identical" << std::endl;
        return false;
      }
    }
  }
  if (scene_1.ee_vector.size() > 0) {
    for (int i = 0; i < static_cast < int > (scene_1.ee_vector.size()); i++) {
      if (!CheckEEEqual(scene_1.ee_vector[i], scene_2.ee_vector[i])) {
        std::cout << "ee not identical" << std::endl;
        return false;
      }
    }
  }
  if (scene_1.parent_objects.size() > 0) {
    for (int i = 0; i < static_cast < int > (scene_1.parent_objects.size()); i++) {
      if (scene_1.parent_objects[i].compare(scene_2.parent_objects[i]) != 0) {
        std::cout << "parent objects not identical" << std::endl;
        return false;
      }
    }
  }
  if (scene_1.child_objects.size() > 0) {
    for (int i = 0; i < static_cast < int > (scene_1.child_objects.size()); i++) {
      if (scene_1.child_objects[i].compare(scene_2.child_objects[i]) != 0) {
        std::cout << "child objects not identical" << std::endl;
        return false;
      }
    }
  }
  std::cout << "scene identical" << std::endl;
  return true;
}
bool CheckRobotEqual(Robot robot_1, Robot robot_2)
{
  if (robot_1.name.compare(robot_2.name) != 0) {
    return false;
  }
  if (robot_1.brand.compare(robot_2.brand) != 0) {
    return false;
  }
  if (robot_1.base_link.compare(robot_2.base_link) != 0) {
    return false;
  }
  if (robot_1.ee_link.compare(robot_2.ee_link) != 0) {
    return false;
  }
  if (robot_1.filepath.compare(robot_2.filepath) != 0) {
    return false;
  }
  if (robot_1.parent_link.compare(robot_2.parent_link) != 0) {
    return false;
  }
  if (robot_1.parent_robot_joint_type.compare(robot_2.parent_robot_joint_type) != 0) {
    return false;
  }
  if (!CheckOriginEqual(robot_1.origin, robot_2.origin)) {
    return false;
  }
  return true;
}
bool CheckEEEqual(EndEffector ee_1, EndEffector ee_2)
{
  if (ee_1.name.compare(ee_2.name) != 0) {
    return false;
  }
  if (ee_1.brand.compare(ee_2.brand) != 0) {
    return false;
  }
  if (ee_1.filepath.compare(ee_2.filepath) != 0) {
    return false;
  }
  if (ee_1.base_link.compare(ee_2.base_link) != 0) {
    return false;
  }
  if (ee_1.robot_link.compare(ee_2.robot_link) != 0) {
    return false;
  }
  if (ee_1.ee_type.compare(ee_2.ee_type) != 0) {
    return false;
  }
  if (ee_1.ee_type.compare("suction") == 0) {
    if (ee_1.attribute_1 != ee_2.attribute_1) {
      return false;
    }
    if (ee_1.attribute_2 != ee_2.attribute_2) {
      return false;
    }
  }
  if (ee_1.ee_type.compare("finger") == 0) {
    if (ee_1.attribute_1 != ee_2.attribute_1) {
      return false;
    }
  }
  if (!CheckOriginEqual(ee_1.origin, ee_2.origin)) {
    return false;
  }
  return true;
}
bool CheckObjectEqual(Object object_1, Object object_2)
{
  if (object_1.name.compare(object_2.name) != 0) {
    return false;
  }
  if (static_cast < int > (object_1.link_vector.size()) !=
    static_cast < int > (object_2.link_vector.size()))
  {
    return false;
  }
  if (static_cast < int > (object_1.joint_vector.size()) !=
    static_cast < int > (object_2.joint_vector.size()))
  {
    return false;
  }

  for (int i = 0; i < static_cast < int > (object_1.link_vector.size()); i++) {
    if (!CheckLinkEqual(object_1.link_vector[i], object_2.link_vector[i])) {
      return false;
    }
  }

  for (int i = 0; i < static_cast < int > (object_1.joint_vector.size()); i++) {
    if (!CheckJointEqual(object_1.joint_vector[i], object_2.joint_vector[i])) {
      return false;
    }
  }
  if (!CheckExtJointEqual(object_1.ext_joint, object_2.ext_joint)) {
    return false;
  }
  return true;
}
bool CheckExtJointEqual(ExternalJoint extjoint_1, ExternalJoint extjoint_2)
{
  if (extjoint_1.name.compare(extjoint_2.name) != 0) {
    return false;
  }
  if (extjoint_1.child_object.compare(extjoint_2.child_object) != 0) {
    return false;
  }
  if (extjoint_1.type.compare(extjoint_2.type) != 0) {
    return false;
  }
  if (CheckAxisEqual(extjoint_1.axis, extjoint_2.axis)) {
    return false;
  }
  if (CheckOriginEqual(extjoint_1.origin, extjoint_2.origin)) {
    return false;
  }

  if (extjoint_1.child_link_pos != extjoint_2.child_link_pos) {
    return false;
  }
  if (extjoint_1.parent_obj_pos != extjoint_2.parent_obj_pos) {
    return false;
  }
  if (extjoint_1.parent_link_pos != extjoint_2.parent_link_pos) {
    return false;
  }
  return true;
}
bool CheckLinkEqual(Link link_1, Link link_2)
{
  if (link_1.name.compare(link_2.name) != 0) {
    return false;
  }
  if (link_1.is_visual != link_2.is_visual) {
    return false;
  }
  if (link_1.is_collision != link_2.is_collision) {
    return false;
  }
  if (link_1.is_inertial != link_2.is_inertial) {
    return false;
  }
  if (static_cast < int > (link_1.visual_vector.size()) !=
    static_cast < int > (link_2.visual_vector.size()))
  {
    return false;
  }
  if (static_cast < int > (link_1.collision_vector.size()) !=
    static_cast < int > (link_2.collision_vector.size()))
  {
    return false;
  }
  if (static_cast < int > (link_1.inertial_vector.size()) !=
    static_cast < int > (link_2.inertial_vector.size()))
  {
    return false;
  }
  if (link_1.visual_vector.size() > 0) {
    for (int i = 0; i < static_cast < int > (link_1.visual_vector.size()); i++) {
      if (!CheckVisualEqual(link_1.visual_vector[i], link_2.visual_vector[i])) {
        return false;
      }
    }
  }
  if (link_1.collision_vector.size() > 0) {
    for (int i = 0; i < static_cast < int > (link_1.collision_vector.size()); i++) {
      if (!CheckCollisionEqual(link_1.collision_vector[i], link_2.collision_vector[i])) {
        return false;
      }
    }
  }
  if (link_1.inertial_vector.size() > 0) {
    for (int i = 0; i < static_cast < int > (link_1.inertial_vector.size()); i++) {
      if (!CheckInertiaEqual(link_1.inertial_vector[i], link_2.inertial_vector[i])) {
        return false;
      }
    }
  }
  return true;
}
bool CheckVisualEqual(Visual visual_1, Visual visual_2)
{
  if (visual_1.name.compare(visual_2.name) != 0) {
    return false;
  }
  if (CheckOriginEqual(visual_1.origin, visual_2.origin)) {
    return false;
  }
  if (CheckGeometryEqual(visual_1.geometry, visual_2.geometry)) {
    return false;
  }
  if (CheckMaterialEqual(visual_1.material, visual_2.material)) {
    return false;
  }
  if (visual_1.is_origin != visual_2.is_origin) {
    return false;
  }
  if (visual_1.is_material != visual_2.is_material) {
    return false;
  }
  return true;
}
bool CheckCollisionEqual(Collision collision_1, Collision collision_2)
{
  if (collision_1.name.compare(collision_2.name) != 0) {
    return false;
  }
  if (!CheckGeometryEqual(collision_1.geometry, collision_2.geometry)) {
    return false;
  }
  if (!CheckOriginEqual(collision_1.origin, collision_2.origin)) {
    return false;
  }
  return true;
}
bool CheckInertiaEqual(Inertial inertial_1, Inertial inertial_2)
{
  if (!CheckOriginEqual(inertial_1.origin, inertial_2.origin)) {
    return false;
  }
  if (inertial_1.mass != inertial_2.mass || inertial_1.ixx != inertial_2.ixx ||
    inertial_1.ixy != inertial_2.ixy || inertial_1.ixz != inertial_2.ixz ||
    inertial_1.iyy != inertial_2.iyy || inertial_1.iyz != inertial_2.iyz ||
    inertial_1.izz != inertial_2.izz)
  {
    return false;
  }
  return true;
}
bool CheckJointEqual(Joint joint_1, Joint joint_2)
{
  if (joint_1.name.compare(joint_2.name) != 0) {
    return false;
  }
  if (joint_1.type.compare(joint_2.type) != 0) {
    return false;
  }
  if (!CheckAxisEqual(joint_1.axis, joint_2.axis)) {
    return false;
  }
  if (!CheckOriginEqual(joint_1.origin, joint_2.origin)) {
    return false;
  }
  if (!CheckLinkEqual(joint_1.parent_link, joint_2.parent_link)) {
    return false;
  }
  if (!CheckLinkEqual(joint_1.child_link, joint_2.child_link)) {
    return false;
  }
  return true;
}
bool CheckOriginEqual(Origin origin_1, Origin origin_2)
{
  if (origin_1.x != origin_2.x || origin_1.y != origin_2.y || origin_1.z != origin_2.z ||
    origin_1.roll != origin_2.roll || origin_1.pitch != origin_2.pitch ||
    origin_1.yaw != origin_2.yaw)
  {
    return false;
  }
  if (origin_1.is_origin != origin_2.is_origin) {
    return false;
  }
  return true;
}
bool CheckAxisEqual(Axis axis_1, Axis axis_2)
{
  if (axis_1.x != axis_2.x || axis_1.y != axis_2.y || axis_1.z != axis_2.z) {
    return false;
  }
  if (axis_1.is_axis != axis_2.is_axis) {
    return false;
  }
  return true;
}
bool CheckGeometryEqual(Geometry geometry_1, Geometry geometry_2)
{
  if (geometry_1.is_stl != geometry_2.is_stl) {
    return false;
  } else {
    if (geometry_1.is_stl) {
      if (geometry_1.filepath.compare(geometry_2.filepath) != 0) {
        return false;
      }
      if (geometry_1.scale_x != geometry_2.scale_x || geometry_1.scale_y != geometry_2.scale_y ||
        geometry_1.scale_z != geometry_2.scale_z)
      {
        return false;
      }
    } else {
      if (geometry_1.shape.compare(geometry_2.shape) != 0) {
        return false;
      }
      if (geometry_1.shape.compare("Sphere") == 0) {
        if (geometry_1.radius != geometry_2.radius) {
          return false;
        }
      } else if (geometry_1.shape.compare("Box") == 0) {
        if (geometry_1.length != geometry_2.length) {
          return false;
        }
        if (geometry_1.breadth != geometry_2.breadth) {
          return false;
        }
        if (geometry_1.height != geometry_2.height) {
          return false;
        }

      } else if (geometry_1.shape.compare("Cylinder") == 0) {
        if (geometry_1.radius != geometry_2.radius) {
          return false;
        }
        if (geometry_1.height != geometry_2.height) {
          return false;
        }
      }
    }
  }
  return true;
}
bool CheckMaterialEqual(Material material_1, Material material_2)
{
  if (material_1.is_material != material_2.is_material) {
    return false;
  }

  if (material_1.is_material) {
    if (material_1.is_texture != material_2.is_texture) {
      return false;
    }

    if (material_1.is_texture) {
      if (material_1.filepath.compare(material_2.filepath) != 0) {
        return false;
      }
    } else {
      if (material_1.a != material_2.a || material_1.r != material_2.r ||
        material_1.g != material_2.g || material_1.b != material_2.b)
      {
        return false;
      }
    }
  }
  return true;
}

#endif  // SCENE_CHECK_H_
