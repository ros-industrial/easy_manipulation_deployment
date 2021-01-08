.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _workcell_initialization:

Workcell Initialization
========================================================

Folder structure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is (Highly) recommended to select the same workspace location as the workspace that you lodaded this package in.

From the main window, choose your workspace location. Ensure that the folder selected is the **main workspace folder**, i.e catkin_ws or colcon_ws

.. image:: ./images/workcell_loaded.png
   :scale: 100%
   :align: center

Once you see the confirmation message that the workcell is loaded, you can then check that folder using your file explorer and you will see the various folders required to store your assets created. Choose the ROS version and distribution required and click next to be directed to the scene select window

Folder Structure for Assets
-----------------------------

This package requires a standardized folder structure in order for the workcell builder to function well. This serves as good practice as well for users to store their files in a logical and standardized format. The following is how your folder should be structured

.. code-block:: bash

   |--workcell_ws
   ___|--src
   ______|--scenes
   ______|--assets
   _________|--robots
   ____________|--robot_brand
   _______________|--robot_model_description
   _______________|--robot_model_moveit_config     
   _________|--end_effectors
   ____________|--end_effector_brand
   _______________|--end_effector_model_description
   _______________|--end_effector_model_moveit_config    
   _________|--environment_objects
   ____________|--environment_objects_description

The rest of the documentation will highlight how it should be populated.

Generating Moveit Config packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

One key feature of this package is to generate workcell simulations that is compatible with path planning frameworks. The current version of the workcell builder is designed to be compatible with Moveit, a popular open source motion planning framework

The moveit configuration packages are required if you want to control your robot with the moveit (and the grasp execution component of easy_manipulation_deployment). It is recommended to use the `Moveit Setup Assistant <http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html/>`_ it is recommended to use this to generate the package rather than to do it yourself. Below are some existing moveit_config folders

`UR Robots <https://github.com/ros-industrial/universal_robot/>`_

`ABB Robots <https://github.com/ros-industrial/abb/>`_


**FOR ROS 2**
-------------

Note that the setup wizard only generates ROS1 packages for now, so if you are using ROS2, **please convert the moveit_config packages to ROS2 before starting**. 

ROS2 Examples (ur5_moveit_config)

CMakeLists.txt:

.. code-block:: bash

   cmake_minimum_required(VERSION 3.10.2)
   project(ur5_moveit_config)
   find_package(ament_cmake REQUIRED)

   install(DIRECTORY launch DESTINATION "share/${PROJECT_NAME}")
   install(DIRECTORY config DESTINATION "share/${PROJECT_NAME}")
   ament_package()
   
package.xml

.. code-block:: bash

   <?xml version="1.0"?>
   <package format="3">
     <name>ur5_moveit_config</name>
     <version>0.6.4</version>
     <description>Resources used for MoveIt! testing</description>

     <author email="isucan@willowgarage.edu">Ioan Sucan</author>
     <author email="acorn@willowgarage.edu">Acorn Pooley</author>

     <maintainer email="dave@dav.ee">Dave Coleman</maintainer>

     <license>BSD</license>
     <url type="website">http://moveit.ros.org</url>
     <url type="bugtracker">https://github.com/ros-planning/moveit-resources/issues</url>
     <url type="repository">https://github.com/ros-planning/moveit-resources</url>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <exec_depend>joint_state_publisher</exec_depend>
     <exec_depend>robot_state_publisher</exec_depend>

     <export>
         <build_type>ament_cmake</build_type>
     </export>
   </package>

Uploading Relevant Assets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before generating a scene, you need to make sure you have the assets you need for the scene, especially for the robot and end effector. 

Robots
--------------------------

For increased reusability and ease of visualization, we will create separate folders for separate vendors of Robots. For example, we will create a folder to store UR robots

.. code-block:: bash

   $ cd workcell_ws/src/assets/robots

   $ mkdir universal_robot



Copy over the moveit_config folder and description folders of the relevant robot models you want to add, ensuring that the folder names and file names follow the naming :ref:`Conventions`


End Effector
--------------------------

Simlarly for End Effectors, we will create a separate folder for each End Effector Vendor. For example, we will create a folder to store Robotiq Grippers

.. code-block:: bash

   $ cd workcell_ws/src/assets/end_effectors

   $ mkdir robotiq

Copy over the moveit_config folder and description folders of the relevant end_effector models you want to add, ensuring that the folder names and file names follow the naming :ref:`Conventions`

Environment Objects
--------------------------

For objects that is part of the environment that will be used as static collision objects, it should be stored in the **workcell_ws/src/assets/environment folder**. 



***Current version of the GUI does not support loading of existing environment objects. For simple environment objects, consider creating a copy of the environment objects with the gui instead**

Next step: :ref:`Create A Scene`
