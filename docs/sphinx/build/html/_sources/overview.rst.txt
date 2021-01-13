.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _Overview:

Overview
========================================================

Manipulation Pipeline
^^^^^^^^^^^^^^^^^^^^^^^^^^

To preserve the modularity of this package, the manipulation pipeline can be broken down into three main aspects, each of which can function separately, or together as an end to end pipeline. Each component has its own documentation and tutorials which are linked in the headers.

.. image:: ./images/emd_diagram.png
   :scale: 50%
   :align: center


Workcell Builder
---------------------------
The Workcell Builder provides an easy to use GUI that allows users to create a representation of their robot task space to provide robot simulation and to provide the initial state for trajectory planning using frameworks such as `Moveit2 <https://github.com/ros-planning/moveit2>`_ 

Grasp Planner
---------------------------

The Grasp Planner subscribes to a topic published by a perception source and outputs an End-effector specific grasp pose for the end effector using a novel, algorithmic depth-based method. This pose is then published to a ROS2 topic.

Grasp Execution
---------------------------
The Grasp Execution component subscribes to the output published by the Grasp Planner, and uses `Moveit2 <https://github.com/ros-planning/moveit2>`_  to develop a collision free trajectory for the robot to navigate to the required grasp object. 

Next step: :ref:`download_instructions`
