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


#ifndef JOINT_PARSER_H
#define JOINT_PARSER_H

#include "yaml-cpp/yaml.h"

#include "origin_parser.h"
#include "string"
#include "axis_parser.h"
#include "joint.h"

class JointParser
{
public:
    static void generate_joint(YAML::Emitter** out, Joint joint)
    {
        // **out<<YAML::Key << "name";
        **out<<YAML::Key << joint.name;
        **out<<YAML::Value;
        **out<<YAML::BeginMap;
        **out<<YAML::Key << "type";
        **out<<YAML::Value << joint.type;
        **out<<YAML::Key << "parent";
        **out<<YAML::Value << joint.parent_link.name;
        **out<<YAML::Key << "child";
        **out<<YAML::Value << joint.child_link.name;

        if(joint.origin.is_origin || joint.axis.is_axis)
        {
            if(joint.origin.is_origin)
            {
                OriginParser::generate_origin(&out, joint.origin);
            }

            if(joint.axis.is_axis)
            {
                AxisParser::generate_axis(&out, joint.axis);
            }
        }
        **out<<YAML::EndMap;

    }
};

#endif // JOINT_PARSER_H
