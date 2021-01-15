## Copyright 2020 ROS Industrial Consortium Asia Pacific
##
## Licensed under the Apache License, Version 2.0 (the "License");
## you may not use this file except in compliance with the License.
## You may obtain a copy of the License at
##
##     http://www.apache.org/licenses/LICENSE-2.0
##
## Unless required by applicable law or agreed to in writing, software
## distributed under the License is distributed on an "AS IS" BASIS,
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
## See the License for the specific language governing permissions and
## limitations under the License.

import os
import yaml
import xacro
import tempfile
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

scene_pkg = 'ur5_test'
robot_base_link = 'base_link'
robot_moveit_pkg = 'ur5_moveit_config'

def to_urdf(xacro_path, urdf_path=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * urdf_path -- the path to the urdf file
    """
    # If no URDF path is given, use a temporary file
    if urdf_path is None:
        urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path  # Return path to the urdf file

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name) #get package filepath
    absolute_file_path = os.path.join(package_path, file_path)
    temp_urdf_filepath = absolute_file_path.replace('.xacro','')
    absolute_file_path = to_urdf(absolute_file_path,temp_urdf_filepath)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        print(package_path)
        print(absolute_file_path)
        return None


def generate_launch_description():

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file(scene_pkg, 'urdf/scene.urdf.xacro')
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file(scene_pkg, 'urdf/arm_hand.srdf.xacro')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml(robot_moveit_pkg , 'config/kinematics.yaml')
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }

    ompl_planning_pipeline_config = { 'ompl' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }

    ompl_planning_yaml = load_yaml(robot_moveit_pkg, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # RViz
    rviz_config_file = get_package_share_directory(scene_pkg) + "/launch/demo.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic])
    # Publish base link TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', robot_base_link])

    return LaunchDescription([ static_tf, rviz_node])
