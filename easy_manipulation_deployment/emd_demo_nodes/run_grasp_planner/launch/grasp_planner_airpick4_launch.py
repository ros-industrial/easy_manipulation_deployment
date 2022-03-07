# Copyright 2020 Advanced Remanufacturing and Technology Centre
# Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ld = LaunchDescription()
    config = get_package_share_directory('run_grasp_planner') + '/config/params_airpick4.yaml'
    print(config)
    node = Node(
        package='run_grasp_planner',
        name='grasp_planning_node',
        executable='demo_node',
        output='screen',
        # prefix=['gdb -ex=r --args'],
        # prefix=['valgrind --leak-check=full'],
        # prefix=['valgrind'],
        parameters=[config]
    )
    # ld.add_action(node)
    return LaunchDescription([node])
