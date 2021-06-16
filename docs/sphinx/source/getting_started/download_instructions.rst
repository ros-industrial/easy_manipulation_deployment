.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _download_instructions:

Download Instructions
========================================================

One of the key features of this package it is semi-modular.

ROS projects generally revolve around usage of URDFs to setup an environment of a robotic workcell.
Workcell Builder eases this process by implementing a GUI to generate your desired workcell environment and can be used for other projects as well!

Grasp Planner and Grasp Execution pipeline work hand in hand to provide a seamless pick and place solution with
the workcell environment created by Workcell Builder.

.. note:: Grasp Execution requires a robotic workcell to be set up, so do install the whole EMD package if a pick and place solution is what you're looking for!


Installing a perception package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


The Easy Manipulation Deployment(EMD) package, specifically Grasp Planner, subscribes to either a topic with PointCloud2 message type *OR*
topics from `easy_perception_deployment(EPD) <https://github.com/ros-industrial/easy_perception_deployment/>`_ package.

.. warning:: EMD is dependent on a perception source. If your camera driver does not provide any PointCloud2 message type topics, do check out the `EPD <https://github.com/ros-industrial/easy_perception_deployment/>`_ package!

Here :ref:`grasp_planner_input` is a link for more information to determine which perception input fits best for your requirements.

Installing complete Easy Manipulation Deployment suite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Installing Easy Manipulation Deployment dependencies
------------------------------------------------------------

.. rubric:: Moveit2

Follow this link `Moveit2 <https://moveit.ros.org/install-moveit2/source/>`_ to build Moveit2 from source.

.. note:: The following *Major* EMD dependencies do not require to be built from source and
          will be installed with rosdep install in the steps below.

          .. rubric:: Pointcloud Library (PCL) | version: 1.10

          .. rubric:: The Flexible Collision Library (FCL) | version: 0.5



Installing only the Workcell Builder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   mkdir -p ~/workcell_ws/src

   cd ~/workcell_ws/src

   git clone https://github.com/ros-industrial/easy_manipulation_deployment.git

   mv easy_manipulation_deployment/assets/ .

   mv easy_manipulation_deployment/scenes/ .

   mv easy_manipulation_deployment/easy_manipulation_deployment/workcell_builder ./easy_manipulation_deployment

   find ./easy_manipulation_deployment -mindepth 1 ! -regex '^./easy_manipulation_deployment/workcell_builder\(/.*\)?' -delete

   cd ~/workcell_ws

   source /opt/ros/foxy/setup.bash

   rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"

   colcon build

   source install/setup.bash

Installing entire Easy Manipulation Deployment package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   mkdir -p ~/workcell_ws/src

   cd ~/workcell_ws/src

   git clone https://github.com/ros-industrial/easy_manipulation_deployment.git

   mv easy_manipulation_deployment/assets/ .

   mv easy_manipulation_deployment/scenes/ .

   mv easy_manipulation_deployment/easy_manipulation_deployment/workcell_builder ./easy_manipulation_deployment

   cd ~/workcell_ws

   source /opt/ros/foxy/setup.bash

   rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"

   source ~/ws_moveit2/install/setup.bash

   colcon build

   source install/setup.bash


