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


#ifndef PRINT_FUNCTIONS_H_
#define PRINT_FUNCTIONS_H_

#include <iostream>

#include "attributes/workcell.h"


class PrintFunctions
{
//    void PrintScene(Scene scene);
//    void PrintRobot(Robot robot);
//    void PrintEE(EndEffector ee);
//    void PrintObject(Object object);
//    void PrintLink(Link link);
//    void PrintVisual(Visual visual);
//    void PrintCollision(Collision collision);
//    void PrintInertial(Inertial inertial);
//    void PrintJoint(Joint joint);
//    void PrintOrigin(Origin origin);
//    void PrintAxis(Axis axis);
//    void PrintGeometry(Geometry geometry);
//    void PrintMaterial(Material material);
  void PrintScene(Scene scene)
  {
    std::cout << "================ Print Scene ================" << std::endl;
    std::cout << "Scene Name: " << scene.name << std::endl;
    std::cout << "Filepath: " << scene.filepath << std::endl;

    std::cout << "~~~~~~~~~ Robots ~~~~~~~~~" << std::endl;

    if (scene.robot_loaded) {
      std::cout << "Robot Present in scene" << std::endl;
      for (int i = 0; i < static_cast < int > (scene.robot_vector.size()); i++) {
        PrintRobot(scene.robot_vector[i]);
      }
    } else {
      std::cout << "Robot Not in scene" << std::endl;
    }
    std::cout << "~~~~~~~~~ End Effectors ~~~~~~~~~" << std::endl;
    if (scene.ee_loaded) {
      std::cout << "End Effector Present in scene" << std::endl;
      for (int i = 0; i < static_cast < int > (scene.ee_vector.size()); i++) {
        PrintEE(scene.ee_vector[i]);
      }
    } else {
      std::cout << "End Effector Not in scene" << std::endl;
    }
    std::cout << "~~~~~~~~~ Objects ~~~~~~~~~" << std::endl;
    if (scene.object_vector.size() > 0) {
      std::cout << scene.object_vector.size() << " Objects in scene" << std::endl;
      for (int i = 0; i < static_cast < int > (scene.object_vector.size()); i++) {
        PrintObject(scene.object_vector[i]);
      }
    } else {
      std::cout << " No objects in scene" << std::endl;
    }
    std::cout << "================ End Scene ================" << std::endl;
  }
  void PrintRobot(Robot robot)
  {
    std::cout << " ================ Print Robot ================" << std::endl;
    std::cout << " Name: " << robot.name << std::endl;
    std::cout << " Brand: " << robot.brand << std::endl;
    std::cout << " filepath: " << robot.filepath << std::endl;
    std::cout << " base_link: " << robot.base_link << std::endl;
    std::cout << " ee_link: " << robot.ee_link << std::endl;
    std::cout << " ================ Links ================" << std::endl;
    for (int i = 0; i < static_cast < int > (robot.robot_links.size()); i++) {
      std::cout << " - " << robot.robot_links[i] << std::endl;
    }
    PrintOrigin(robot.origin);
  }
  void PrintEE(EndEffector ee)
  {
    std::cout << " ================ Print EE ================" << std::endl;
    std::cout << " Name: " << ee.name << std::endl;
    std::cout << " Brand: " << ee.brand << std::endl;
    std::cout << " filepath: " << ee.filepath << std::endl;
    std::cout << " base_link: " << ee.base_link << std::endl;
    std::cout << " robot_link: " << ee.robot_link << std::endl;
    std::cout << " Type: " << ee.ee_type << std::endl;
    if (ee.ee_type.compare("finger") == 0) {
      std::cout << " Number of Fingers: " << ee.attribute_1 << std::endl;
    }
    if (ee.ee_type.compare("suction") == 0) {
      std::cout << " Suction Width: " << ee.attribute_1 << std::endl;
      std::cout << " Suction Height: " << ee.attribute_2 << std::endl;
    }
    std::cout << " ================ Links ================" << std::endl;
    for (int i = 0; i < static_cast < int > (ee.ee_links.size()); i++) {
      std::cout << " - " << ee.ee_links[i] << std::endl;
    }
    PrintOrigin(ee.origin);
  }
  void PrintObject(Object object)
  {
    std::cout << "================ Print Object ================" << std::endl;
    std::cout << " Object name: " << object.name << std::endl;
    std::cout << "================ Print Links ================" << std::endl;
    for (int link = 0; link < static_cast < int > (object.link_vector.size()); link++) {
      PrintLink(object.link_vector[link]);
    }
    for (int joint = 0; joint < static_cast < int > (object.joint_vector.size()); joint++) {
      PrintJoint(object.joint_vector[joint]);
    }
    std::cout << "================ Print External Joints ================" << std::endl;
    std::cout << "Name: " << object.ext_joint.name << std::endl;
    std::cout << "Type: " << object.ext_joint.type << std::endl;
    std::cout << "Child Object: " << object.ext_joint.child_object << std::endl;
    std::cout << "Child Link Pos: " << object.ext_joint.child_link_pos << std::endl;
    std::cout << "Parent Object Pos: " << object.ext_joint.parent_obj_pos << std::endl;
    std::cout << "Parent Link Pos: " << object.ext_joint.parent_link_pos << std::endl;
    if (object.ext_joint.axis.is_axis) {
      PrintAxis(object.ext_joint.axis);
    } else {
      std::cout << "No Axis" << std::endl;
    }
    if (object.ext_joint.origin.is_origin) {
      PrintOrigin(object.ext_joint.origin);
    } else {
      std::cout << "No origin" << std::endl;
    }
  }
  void PrintLink(Link link)
  {
    std::cout << " ================ Link ================" << std::endl;
    std::cout << " Name: " << link.name << std::endl;
    if (link.is_visual) {
      std::cout << " Loading Visual" << std::endl;
      for (int visual = 0; visual < static_cast < int > (link.visual_vector.size()); visual++) {
        PrintVisual(link.visual_vector[visual]);
      }
    } else {
      std::cout << " No Visual" << std::endl;
    }
    if (link.is_collision) {
      std::cout << " Loading Collision" << std::endl;
      for (int collision = 0; collision < static_cast < int > (link.collision_vector.size());
        collision++)
      {
        PrintCollision(link.collision_vector[collision]);
      }
    } else {
      std::cout << " No Collision" << std::endl;
    }
    if (link.is_inertial) {
      std::cout << " Loading Inertial" << std::endl;
      for (int inertial = 0; inertial < static_cast < int > (link.inertial_vector.size());
        inertial++)
      {
        PrintInertial(link.inertial_vector[inertial]);
      }
    } else {
      std::cout << " No Inertial" << std::endl;
    }
  }
  void PrintVisual(Visual visual)
  {
    std::cout << " ================ Visual ================" << std::endl;
    std::cout << " Name: " << visual.name << std::endl;
    PrintGeometry(visual.geometry);
    if (visual.is_origin) {
      std::cout << " Origin Present" << std::endl;
      PrintOrigin(visual.origin);
    } else {
      std::cout << " No Origin Present" << std::endl;
    }
    if (visual.is_material) {
      std::cout << " Material Present" << std::endl;
      PrintMaterial(visual.material);
    } else {
      std::cout << " No Material Present" << std::endl;
    }
  }
  void PrintCollision(Collision collision)
  {
    std::cout << " ================ Print Collision ================" << std::endl;
    std::cout << " Name: " << collision.name << std::endl;
    PrintGeometry(collision.geometry);
    if (collision.origin.is_origin) {
      std::cout << " Origin Present" << std::endl;
      PrintOrigin(collision.origin);
    } else {
      std::cout << " No Origin Present" << std::endl;
    }
  }
  void PrintInertial(Inertial inertial)
  {
    std::cout << " ============== Print Inertial ==============" << std::endl;
    std::cout << " mass: " << inertial.mass << std::endl;
    std::cout << " ixx: " << inertial.ixx << std::endl;
    std::cout << " ixy: " << inertial.ixy << std::endl;
    std::cout << " ixz: " << inertial.ixz << std::endl;
    std::cout << " iyy: " << inertial.iyy << std::endl;
    std::cout << " iyz: " << inertial.iyz << std::endl;
    std::cout << " izz: " << inertial.izz << std::endl;
    if (inertial.origin.is_origin) {
      std::cout << " Origin Present" << std::endl;
      PrintOrigin(inertial.origin);
    } else {
      std::cout << " No Origin Present" << std::endl;
    }
  }
  void PrintJoint(Joint joint)
  {
    std::cout << " ================ Print Joint ================" << std::endl;
    std::cout << " Name: " << joint.name << std::endl;
    std::cout << " Type: " << joint.type << std::endl;
    std::cout << " Parent Link: " << std::endl;
    PrintLink(joint.parent_link);
    std::cout << " Child Link: " << std::endl;
    PrintLink(joint.child_link);

    if (joint.origin.is_origin) {
      std::cout << " Origin Present" << std::endl;
      PrintOrigin(joint.origin);
    } else {
      std::cout << " No Origin Present" << std::endl;
    }

    if (joint.axis.is_axis) {
      std::cout << " Axis Present" << std::endl;
      PrintAxis(joint.axis);
    } else {
      std::cout << " Axis Present" << std::endl;
    }
  }
  void PrintGeometry(Geometry geometry)
  {
    std::cout << " ================ Print Geometry ================" << std::endl;

    if (geometry.is_stl) {
      std::cout << " Use STL File " << std::endl;
      std::cout << " Filepath: " << geometry.filepath << std::endl;
      std::cout << " Scale x: " << geometry.scale_x << std::endl;
      std::cout << " Scale y: " << geometry.scale_y << std::endl;
      std::cout << " Scale z: " << geometry.scale_z << std::endl;
    } else {
      std::cout << " Use shape " << std::endl;
      std::cout << " Shape: " << geometry.shape << std::endl;
      std::cout << " length: " << geometry.length << std::endl;
      std::cout << " breadth: " << geometry.breadth << std::endl;
      std::cout << " height: " << geometry.height << std::endl;
      std::cout << " radius: " << geometry.radius << std::endl;
    }
  }


  void PrintMaterial(Material material)
  {
    std::cout << " ================ Print Material ================" << std::endl;
    if (material.is_material) {
      std::cout << " Name: " << material.material_name << std::endl;
      if (material.is_texture) {
        std::cout << " Use texture file" << std::endl;
        std::cout << " Filepath: " << material.filepath << std::endl;
      } else {
        std::cout << " Use rgba" << std::endl;
        std::cout << " r: " << material.r << std::endl;
        std::cout << " g: " << material.g << std::endl;
        std::cout << " b: " << material.b << std::endl;
        std::cout << " a: " << material.a << std::endl;
      }
    }
  }


  void PrintOrigin(Origin origin)
  {
    if (origin.is_origin) {
      std::cout << " ================ Origin ================" << std::endl;
      std::cout << " X: " << origin.x << std::endl;
      std::cout << " Y: " << origin.y << std::endl;
      std::cout << " Z: " << origin.z << std::endl << std::endl;
      std::cout << " R: " << origin.roll << std::endl;
      std::cout << " P: " << origin.pitch << std::endl;
      std::cout << " Y: " << origin.yaw << std::endl;
    }
  }


  void PrintAxis(Axis axis)
  {
    std::cout << " ================ Print Axis ================" << std::endl;
    if (axis.is_axis) {
      std::cout << " X: " << axis.x << std::endl;
      std::cout << " Y: " << axis.y << std::endl;
      std::cout << " Z: " << axis.z << std::endl;
    }
  }
};


#endif   // PRINT_FUNCTIONS_H_
