.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _Run workcell demo:

Run workcell demo
========================================================

To run the demo simulation for the scene you just generated, do the following:


ROS1
^^^^^^

.. code-block:: bash

   $ source /opt/ros/melodic/setup.bash
   
   $ catkin build
   
   $ source devel/setup.bash
   
   $ roslaunch <scene_name> demo.launch

ROS2
^^^^^^

In a new terminal, navigate to your workcell_ws

.. code-block:: bash

   $ source /opt/ros/foxy/setup.bash
   
   $ colcon build
   
   $ source install/setup.bash
   
   $ ros2 launch <scene_name> demo.launch.py


Rviz will then be launched and you should see your scene displayed. To integrate this scene with Moveit2, check out :ref:`grasp_execution_demo`
