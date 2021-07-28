# Copyright 2021 ROS Industrial Consortium Asia Pacific
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import xacro
import yaml

scene_pkg = 'ur5_2f_test'
robot_base_link = 'base_link'


def to_urdf(xacro_path, urdf_path=None, mappings=None):
    # If no URDF path is given, use a temporary file
    if urdf_path is None:
        urdf_path = tempfile.mktemp(prefix='%s_' % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=mappings)
    # open the output file
    print(urdf_path)
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path  # Return path to the urdf file


def load_file(package_name, file_path, mappings=None):
    package_path = get_package_share_directory(package_name)  # get package filepath
    absolute_file_path = os.path.join(package_path, file_path)
    temp_urdf_filepath = absolute_file_path.replace('.xacro', '')
    absolute_file_path = to_urdf(absolute_file_path, temp_urdf_filepath, mappings)

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
    # Initial position mapping
    initial_position_path = (get_package_share_directory('grasp_execution') +
                             '/config/start_positions.yaml')
    initial_position_mappings = {
        'initial_positions_file': initial_position_path}

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file(scene_pkg, 'urdf/scene.urdf.xacro',
                                         initial_position_mappings)
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

    controllers_yaml = load_yaml('grasp_execution', 'config/controllers.yaml')
    moveit_controller = {
        'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        }

    # Octomap Integration
    octomap_config = {'octomap_frame': '/camera_color_optical_frame',
                      'octomap_resolution': 0.04,
                      'max_range': 2.0}

    octomap_updater_config = load_yaml('grasp_execution', 'config/sensors_3d.yaml')

    # moveit_cpp.yaml is passed by filename for now since it's node specific
    dynamic_safety_yaml_file_name = (get_package_share_directory('grasp_execution') +
                                     '/config/dynamic_safety_demo.yaml')

    # Manipulator config
    workcell_context_yaml = os.path.join(
        get_package_share_directory('grasp_execution'), 'config', 'workcell_context.yaml')
    workcell_context = {'workcell_context': workcell_context_yaml}

    # MoveItCpp demo executable
    dynamic_safety_demo_node = Node(
        name='dynamic_safety_demo_node',
        package='grasp_execution',
        # TODO(henningkayser): add debug argument
        # prefix='xterm -e gdb --args',
        executable='dynamic_safety_demo_node',
        output='screen',
        parameters=[dynamic_safety_yaml_file_name,
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    ompl_planning_pipeline_config,
                    trajectory_execution,
                    octomap_config,
                    octomap_updater_config,
                    workcell_context,
                    moveit_controller]
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

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur5_moveit_config"),
        "config",
        "ur5_ros_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Load controllers
    load_controllers = []
    for controller in ["ur5_arm_controller", "joint_state_controller"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription([
        robot_state_publisher,
        rviz_node,
        dynamic_safety_demo_node,
        ros2_control_node,
        ]
        + load_controllers
    )
