.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _workcell_builder_example:

Workcell Builder Example
========================================================


In this example we will be creating a simple robotic workcell using the UR5 and the Robotiq-2F gripper. The expected scene will be as shown. (Note that the robot is currently not in a home pose because the grasp execution node has not been initialized with this visualization) **In the package we include the UR and Robotiq description and moveit_config folders, but in this tutorial we will show you how to do it from scratch**


.. toctree::
   :maxdepth: 3
   
   workcell_builder_example_initial
   workcell_builder_example_gui
   workcell_builder_example_manipulator
   workcell_builder_example_ee
   workcell_builder_example_object
   workcell_builder_example_load_object
   workcell_builder_example_extj
   workcell_builder_example_files
   workcell_builder_example_check_scene
   workcell_builder_example_camera
   
