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


#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include "object.h"
#include "external_joint.h"
#include "robot.h"
#include "end_effector.h"
#include "environment.h"

class Scene
{
public:

    std::vector<Object> object_vector;
    //std::vector<ExternalJoint> ext_joint_vector;
    std::vector<Robot> robot_vector;
    std::vector<EndEffector> ee_vector;
    std::vector<std::string> parent_objects;            // Cannot be empty
    std::vector<std::string> child_objects;             // Cannot be empty
    std::string name;
    std::string filepath;
    bool robot_loaded;
    bool ee_loaded;
    bool loaded;
    Scene()
    {
        robot_loaded = false;
        ee_loaded = false;
        //loaded = false;
    }

};


#endif // GUI_ENVIRONMENT_H
