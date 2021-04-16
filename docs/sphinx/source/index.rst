.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Easy Manipulation Deployment
========================================================

A modular and easy to deploy ROS2 manipulation pipeline that integrates perception elements to establish an end-to-end pick and place task.

This package was tested with the `easy_perception_deployment <https://github.com/ros-industrial/easy_perception_deployment/>`_ ROS2 package, but any other perception system that provides the same ROS2 message in the right topic can work with this package as well. 

.. toctree::
   :maxdepth: 1
   :caption: About
   :name: sec-general

   about/index

.. toctree::
   :maxdepth: 1
   :caption: Getting Started
   :name: sec-start

   getting_started/common_concepts
   getting_started/download_instructions

.. toctree::
   :maxdepth: 1
   :caption: Tutorials
   :name: sec-tutorials

   tutorials/emd_example

.. toctree::
   :maxdepth: 1
   :caption: Individual Package Details
   
   emd_packages/workcell_builder/workcell_builder
   emd_packages/grasp_execution/grasp_execution_demo
   emd_packages/grasp_planner/grasp_planner
   
   


