.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _download_instructions:

Download Instructions
========================================================

One of the key features of this package is its modularity, meaning that each of the three components can be run together or separately (with some differences in initial installations)

Installing a perception package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**This package will not work without a perception source.** If you do not have your own perception package, check out the `easy_perception_deployment <https://github.com/ros-industrial/easy_perception_deployment/>`_ package.

Installing complete easy_manipulation_deployment suite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Build and install EMD dependencies 
---------------------------------------------

.. rubric:: PCL

- Build and install PCL from source in a seperate workspace

.. code-block:: bash

   cd 

   git clone https://github.com/PointCloudLibrary/pcl.git

   cd ~/pcl && mkdir build && cd build

   cmake -DCMAKE_BUILD_TYPE=Release ..

   sudo make -j4

   sudo make install

   cd

.. note:: Do take note that the current implementation uses PCL 1.11 which is built from source. If there are any conflicts with PCL in your other projects,
          it may be due to incompatible PCL release between debian and source. 

          Do the following steps to remove the source version of PCL..

          .. code-block:: shell

            cd ~/pcl && mkdir build && cd build

            sudo make uninstall


.. rubric:: Moveit2

- Build and install moveit2 from source in a separate workspace

.. code-block:: bash

   mkdir -p ~/moveit2_ws/src

   cd ~/moveit2_ws
   
   curl --output moveit2.repos \
     https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos
  
   vcs import src < moveit2.repos
   
   cd ~/moveit2_ws/src/moveit2/ && git checkout 2499a72f7388a371905eaef72685fcfaae04335a && cd ~/moveit2_ws

   source /opt/ros/foxy/setup.bash
   
   rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"
   
   colcon build


Build and Install easy_manipulation_deployment
----------------------------------------------

.. code-block:: bash

   mkdir -p ~/workcell_ws/src

   cd ~/workcell_ws/src

   git clone https://github.com/ros-industrial/easy_manipulation_deployment.git
   
   mv easy_manipulation_deployment/assets/ .

   mv easy_manipulation_deployment/scenes/ .
   
   cd ~/workcell_ws
   
   source /opt/ros/foxy/setup.bash
   
   rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"
   
   source ~/moveit2_ws/install/setup.bash

   colcon build

   source install/setup.bash
   


   
Installing only the Workcell Builder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   mkdir -p ~/workcell_ws/src

   cd ~/workcell_ws/src

   git clone https://github.com/ros-industrial/easy_manipulation_deployment.git
   
   mv easy_manipulation_deployment/assets/ .

   mv easy_manipulation_deployment/scenes/ .
   
   find ./easy_manipulation_deployment -mindepth 1 ! -regex '^./easy_manipulation_deployment/workcell_builder\(/.*\)?' -delete

   cd ~/workcell_ws
   
   source /opt/ros/foxy/setup.bash
   
   rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"

   colcon build

   source install/setup.bash


Installing only the Grasp Planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   mkdir -p ~/workcell_ws/src

   cd ~/workcell_ws/src

   git clone https://github.com/ros-industrial/easy_manipulation_deployment.git

   find ./easy_manipulation_deployment -mindepth 1 ! -regex '^./easy_manipulation_deployment/grasp_planner\(/.*\)?' ! -regex '^./easy_manipulation_deployment/custom_msgs\(/.*\)?' -delete

   cd ~/workcell_ws

   source /opt/ros/foxy/setup.bash
   
   rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"

   colcon build

   source install/setup.bash


Installing only Grasp Execution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Instructions coming soon**

