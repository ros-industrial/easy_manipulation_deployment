# Copyright 2020 ROS Industrial Consortium Asia Pacific
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

import os
import tempfile
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

scene_pkg = 'ur5_2f_test'
robot_base_link = 'base_link'


def to_urdf(xacro_path, urdf_path=None):
    # If no URDF path is given, use a temporary file
    if urdf_path is None:
        urdf_path = tempfile.mktemp(prefix='%s_' % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path)
    # open the output file
    print(urdf_path)
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path  # Return path to the urdf file


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)  # get package filepath
    absolute_file_path = os.path.join(package_path, file_path)
    temp_urdf_filepath = absolute_file_path.replace('.xacro', '')
    absolute_file_path = to_urdf(absolute_file_path, temp_urdf_filepath)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # moveit_cpp.yaml is passed by filename for now since it's node specific
    grasp_execution_yaml_file_name = (get_package_share_directory('grasp_execution') +
                                      '/config/grasp_execution.yaml')

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file(scene_pkg, 'urdf/scene.urdf.xacro')
    robot_description = {'robot_description': robot_description_config}

    robot_description_semantic_config = load_file(scene_pkg, 'urdf/arm_hand.srdf.xacro')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('ur5_moveit_config', 'config/kinematics.yaml')

    ompl_planning_pipeline_config = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ('default_planner_request_adapters/AddTimeOptimalParameterization '
                                 'default_planner_request_adapters/FixWorkspaceBounds '
                                 'default_planner_request_adapters/FixStartStateBounds '
                                 'default_planner_request_adapters/FixStartStateCollision '
                                 'default_planner_request_adapters/FixStartStatePathConstraints'),
            'start_state_max_bounds_error': 0.1}
        }

    ompl_planning_yaml = load_yaml('ur5_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.01}

    controllers_yaml = load_yaml('grasp_execution', 'config/fake_controllers.yaml')
    fake_controller = {
        'moveit_fake_controller_manager': controllers_yaml,
        'moveit_controller_manager': 'moveit_fake_controller_manager/MoveItFakeControllerManager'
        }

    # MoveItCpp demo executable
    grasp_execution_demo_node = Node(
        name='grasp_execution_node',
        package='grasp_execution',
        # TODO(henningkayser): add debug argument
        # prefix='xterm -e gdb --args',
        executable='demo_node',
        output='screen',
        parameters=[grasp_execution_yaml_file_name,
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    ompl_planning_pipeline_config,
                    trajectory_execution,
                    fake_controller]
        )

    # RViz
    rviz_config_file = (get_package_share_directory('grasp_execution') +
                        '/config/grasp_execution.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description,
                    robot_description_semantic]
        )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
        )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[os.path.join(get_package_share_directory(scene_pkg),
                   'urdf/scene.urdf')],
        output='log',
        parameters=[{'source_list': ["/fake_controller_joint_states"]},
                    {'zeros': load_yaml('grasp_execution', 'config/start_positions.yaml')}
                    ]
        )

    return LaunchDescription([
        robot_state_publisher,
        rviz_node,
        joint_state_publisher,
        grasp_execution_demo_node
        ])
