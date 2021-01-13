.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _Conventions:

Conventions
========================================================
## Naming Conventions

Following a standardized naming convention is highly recommended to avoid any issues with generating the workspace.


Description folders
^^^^^^^^^^^^^^^^^^^^^^^^^^

Any folder that provides a visual representation of each object in scene should be named :code:`<name>` _description

.. code-block:: bash

   The current exception to this rule is the description folder for universal robots, which is currently stored as a folder named  ur_description that encapsulates all the current robot models

URDF folders
^^^^^^^^^^^^^^^^^^^^^^^^^^

If the folder contains URDF files for description, it should be in a **xacro** format stored in the :code:`urdf` folder, and named: 

Robot :code:`<robot_model>`.urdf.xacro
  
End effector :code:`<end_effector_model>` _gripper.urdf.xacro

Environment objects :code:`<object_name>`.urdf.xacro

moveit_config folder
^^^^^^^^^^^^^^^^^^^^^^^^^^

All end effectors and robots should come with a moveit_config folder named :code:`<name>` _moveit_config and should be located in the same directory as your robot/end effector description folders.

This folder should be generated using the Moveit Setup Wizard. However, the package generated is currently in ROS1, hence you must make sure that the package is converted into a ROS2 package if your workcell is run in ROS2
