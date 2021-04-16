.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _workcell_builder_example_initial:

Before running the GUI
=========================
**The following instructions provides an example of how you can incorporate new robots and end effector ROS1 packages into this package. There are currently a few robots and end-effectors included in the package, so if you do not need to add any more of these packages, skip forward to :** :ref:`workcell_builder_example_gui`

Downloading Robot and End effector resources
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Assuming that you have followed the :ref:`download_instructions` and have successfully installed the workcell builder, remove all folders in the workcell_ws/src/assets/robots , workcell_ws/src/assets/end_effectors , and workcell_ws/src/assets/environment_objects folders.

Your resulting ROS2 workspace should look like this 

.. code-block:: bash

   |--workcell_ws
   ___|--src
   ______|-- ....other folders
   ______|--scenes
   ______|--assets
   _________|--robots
   _________|--end_effectors
   _________|--environment_objects

**Universal Robot**
   
Next we will get the ur_description and ur5_moveit config folders from the `ROS-Industrial Universal Robots repository <https://github.com/ros-industrial/universal_robot/tree/kinetic-devel>`_ . For this example, we can use `the kinetic-devel` branch

Clone the repository in the :code:`assets/robots` folder. For this example we only require the :code:`ur_description` and :code:`ur5_moveit_config` folders, thus we remove the other folders for now.

**Robotiq End Effector**

Next we will get the robotiq_85_description and robotiq_85_moveit_config folders from the `Robotiq gripper repository <https://github.com/StanleyInnovation/robotiq_85_gripper>`_

Clone the repository in the :code:`assets/robots` folder. For this example we only require the :code:`robotiq_85_description` and :code:`robotiq_85_moveit_config` folders, thus we remove the other folders for now.

 Your workspace should look like this.

.. code-block:: bash

   |--workcell_ws
   ___|--src
   ______|-- ....other folders
   ______|--scenes
   ______|--assets
   _________|--robots
   ____________|--universal_robot
   _______________|--ur5_moveit_config
   _______________|--ur_description
   _________|--end_effectors
   ____________|--robotiq_85_gripper
   _______________|--robotiq_85_description
   _______________|--robotiq_85_moveit_config
   _________|--environment_objects


Edit CMakelists.txt and package.xml
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As this example will be run on ROS2 Foxy, we will need to make some changes to the CMakelists and package.xml

**Universal Robot**

In the :code:`/assets/robots/ur_description/CMakeLists.txt`, replace the contents with the following: 

.. code-block:: bash

   cmake_minimum_required(VERSION 3.10.2)
   project(ur_description)
   find_package(ament_cmake REQUIRED)

   install(DIRECTORY meshes DESTINATION "share/${PROJECT_NAME}")
   install(DIRECTORY urdf DESTINATION "share/${PROJECT_NAME}")
   ament_package()

In the :code:`/assets/robots/ur_description/package.xml`, replace the contents with the following: 

.. code-block:: bash

   <?xml version="1.0"?>
   <package format="3">
     <name>ur_description</name>
     <version>1.2.7</version>
     <description>
         URDF description for Universal UR5/10 robot arms
     </description>

     <author>Wim Meeussen</author>
     <author>Kelsey Hawkins</author>
     <author>Mathias Ludtke</author>
     <author>Felix Messmer</author>
     <maintainer email="g.a.vanderhoorn@tudelft.nl">G.A. vd. Hoorn</maintainer>
     <maintainer email="miguel.prada@tecnalia.com">Miguel Prada Sarasola</maintainer>
     <maintainer email="nhg@ipa.fhg.de">Nadia Hammoudeh Garcia</maintainer>

     <license>BSD</license>

     <url type="website">http://ros.org/wiki/ur_description</url>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <exec_depend>joint_state_publisher</exec_depend>
     <exec_depend>robot_state_publisher</exec_depend>
     <exec_depend>rviz</exec_depend>
     <exec_depend>urdf</exec_depend>
     <exec_depend>xacro</exec_depend>

     <export>
         <build_type>ament_cmake</build_type>
     </export>
   </package>
   
In the :code:`/assets/robots/ur5_moveit_config/CMakeLists.txt`, replace the contents with the following: 

.. code-block:: bash

   cmake_minimum_required(VERSION 3.10.2)
   project(ur5_moveit_config)
   find_package(ament_cmake REQUIRED)


   install(DIRECTORY config DESTINATION "share/${PROJECT_NAME}")
   install(DIRECTORY launch DESTINATION "share/${PROJECT_NAME}")
   install(DIRECTORY tests DESTINATION "share/${PROJECT_NAME}")
   ament_package()

In the :code:`/assets/robots/ur5_moveit_config/package.xml`, replace the contents with the following: 

.. code-block:: bash

   <?xml version="1.0"?>
   <package format="3">
     <name>ur5_moveit_config</name>
     <version>1.2.7</version>
     <description>
        An automatically generated package with all the configuration and launch files for using the ur5 with the MoveIt Motion Planning Framework
     </description>
     <author>Felix Messmer</author>
     <maintainer email="g.a.vanderhoorn@tudelft.nl">G.A. vd. Hoorn</maintainer>
     <maintainer email="miguel.prada@tecnalia.com">Miguel Prada Sarasola</maintainer>
     <maintainer email="nhg@ipa.fhg.de">Nadia Hammoudeh Garcia</maintainer>
  
     <license>BSD</license>

     <url type="website">http://moveit.ros.org/</url>
     <url type="bugtracker">https://github.com/ros-planning/moveit_setup_assistant/issues</url>
     <url type="repository">https://github.com/ros-planning/moveit_setup_assistant</url>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <exec_depend>joint_state_publisher</exec_depend>
     <exec_depend>robot_state_publisher</exec_depend>
     <exec_depend>xacro</exec_depend>
     <depend>ur_description</depend>

     <export>
         <build_type>ament_cmake</build_type>
     </export>
   </package>

**Robotiq End Effector**

In the :code:`/assets/end_effectors/robotiq_85_gripper/robotiq_85_description/CMakeLists.txt`, replace the contents with the following: 

.. code-block:: bash

   cmake_minimum_required(VERSION 3.10.2)
   project(robotiq_85_description)
   find_package(ament_cmake REQUIRED)

   install(DIRECTORY meshes DESTINATION "share/${PROJECT_NAME}")
   install(DIRECTORY urdf DESTINATION "share/${PROJECT_NAME}")
   ament_package()

In the :code:`/assets/end_effectors/robotiq_85_gripper/robotiq_85_description/package.xml`, replace the contents with the following: 

.. code-block:: bash

   <?xml version="1.0"?>
   <package format="3">
    <name>robotiq_85_description</name>
     <version>0.6.4</version>
     <description>Stanley Innovation Robotiq 85 Visual Models</description>
     <maintainer email="dev@stanleyinnovation.com">Patrick Hussey</maintainer>
     <author>Patrick Hussey</author>

     <license>BSD</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <exec_depend>joint_state_publisher</exec_depend>
     <exec_depend>robot_state_publisher</exec_depend>

     <export>
         <build_type>ament_cmake</build_type>
     </export>
   </package>

In the :code:`/assets/end_effectors/robotiq_85_gripper/robotiq_85_moveit_config/CMakeLists.txt`, replace the contents with the following: 

.. code-block:: bash

   cmake_minimum_required(VERSION 3.10.2)
   project(robotiq_85_moveit_config)
   find_package(ament_cmake REQUIRED)


   install(DIRECTORY config DESTINATION "share/${PROJECT_NAME}")
   install(DIRECTORY launch DESTINATION "share/${PROJECT_NAME}")
   ament_package()


In the :code:`/assets/end_effectors/robotiq_85_moveit_config/package.xml`, replace the contents with the following: 

.. code-block:: bash

   <package>
     <name>robotiq_85_moveit_config</name>
     <version>0.2.0</version>
     <description>
        An automatically generated package with all the configuration and launch files for using the robotiq_85_gripper with the MoveIt Motion Planning Framework
     </description>
     <author email="assistant@moveit.ros.org">MoveIt Setup Assistant</author>
     <maintainer email="assistant@moveit.ros.org">MoveIt Setup Assistant</maintainer>

     <license>BSD</license>

     <url type="website">http://moveit.ros.org/</url>
     <url type="bugtracker">https://github.com/ros-planning/moveit_setup_assistant/issues</url>
     <url type="repository">https://github.com/ros-planning/moveit_setup_assistant</url>

   <buildtool_depend>ament_cmake</buildtool_depend>

     <exec_depend>joint_state_publisher</exec_depend>
     <exec_depend>robot_state_publisher</exec_depend>
     <exec_depend>xacro</exec_depend>
     <build_depend>robotiq_85_description</build_depend>
     <exec_depend>robotiq_85_description</exec_depend>

     <export>
         <build_type>ament_cmake</build_type>
     </export>
   </package>

Xacro-ize the SRDFs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As this workcell builder aims to create links between the manipulator and end effector, the semantic descriptions need to be accessible as macros. 

In the :code:`/assets/end_effectors/robotiq_85_gripper/robotiq_85_moveit_config/config` folder, make a copy of :code:`robotiq_85_gripper.srdf` and rename it :code:`robotiq_85_gripper.srdf.xacro` . in this file, add the xacro tags :code:`<xacro:macro name="robotiq_85">` and :code:`
</xacro:macro>` to the start and end of the file, as well as adding the XML NameSpace :code:`<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robotiq_85_gripper">` 

Your :code:`robotiq_85_gripper.srdf.xacro` file should be as shown

.. code-block:: bash

   <?xml version="1.0" ?>
   <!--This does not replace URDF, and is not an extension of URDF.
       This is a format for representing semantic information about the robot structure.
       A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined
   -->
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robotiq_85_gripper">
   <xacro:macro name="robotiq_85_gripper">
   ...
   ...
   ...

       <disable_collisions link1="gripper_root_link" link2="robotiq_coupler_link" reason="Adjacent" />
   </xacro:macro>
   </robot>
   
In the :code:`/assets/end_effectors/robotiq_85_gripper/ur5_moveit_config/config` folder, make a copy of :code:`ur5.srdf` and rename it :code:`ur5.srdf.xacro` . in this file, add the xacro tags :code:`<xacro:macro name="ur5">` and :code:`
</xacro:macro>` to the start and end of the file, as well as adding the XML NameSpace :code:`<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="ur5">` 

Your :code:`ur5.srdf.xacro` file should be as shown

.. code-block:: bash

   <?xml version="1.0" ?>
   <!--This does not replace URDF, and is not an extension of URDF.
       This is a format for representing semantic information about the robot structure.
       A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined
   -->
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="ur5">
   <xacro:macro name="ur5">
   ...
   ...
   ...

       <disable_collisions link1="wrist_2_link" link2="wrist_3_link" reason="Adjacent" />
   </xacro:macro>
   </robot>


Next step: :ref:`workcell_builder_example_gui`

