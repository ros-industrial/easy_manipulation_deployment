.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_example:

Grasp Planner Example
========================================================

In this part of the tutorial we will reference the scene that we have generated in the :ref:`workcell_builder_example`

If you currently do not have a working perception system, you can still test out the package using a ROSbag located in :code:`/workcell_ws/src/easy_manipulation_deployment/grasp_planner/rosbag/perception_example.zip`

The perception rosbag is an simple box as shown below.

.. image:: ./images/example/example_object.png


Set up end effector parameters
--------------------------------

The end effector we are currently using is the Robotiq 2F-85 gripper, thus we need to set the configuration files before running the grasp planner. In the configuration file :code:`/workcell_ws/src/easy_manipulation_deployment/grasp_planner/config/attributes.yaml` , replace the contents with the following: 

.. code-block:: bash

   end_effector:
     type: finger
     attributes:
       fingers: 2
       distance_between_fingers: 100
       longest_gripper_dim: 25
       table_height: 450
   parameters:
     min_zero_angle: 0.01
     min_height_diff_to_grip: 5
     min_gdi_diff_for_comparison: 5


note that the parameters like :code:`table_height` is specific to this current ROSbag, and should be changed accordingly for your own set up. more information about the other parameters, check out :ref:`Grasp Planner Configuration` 


Running the grasp planner
------------------------------

For this part of the example you need 2 terminals. 

In Terminal 1:

.. code-block:: bash

   cd ~/workcell_ws

   source /opt/ros/foxy/setup.bash

   colcon build

   source install/setup.bash

   ros2 run grasp_planning grasp_planning_node
   
You should then see the output in terminal 1: 

.. code-block:: bash

   [easy_manipulation_deployment][Grasp Planner] Waiting for topic....

In Terminal 2:

.. code-block:: bash

   cd ~/workcell_ws

   source /opt/ros/foxy/setup.bash

   colcon build

   source install/setup.bash

   ros2 bag play src/easy_manipulation_deployment/grasp_planner/rosbag/perception_simulator/perception_simulator/perception_simulator.db3
   
You should then see the output in terminal 2: 

.. code-block:: bash

   [INFO] [1610531624.795616932] [rosbag2_storage]: Opened database 'src/easy_manipulation_deployment/grasp_planner/rosbag/perception_simulator/perception_simulator/perception_simulator.db3' for READ_ONLY.
   
In Terminal 1, you should then see the following

.. code-block:: bash

   [easy_manipulation_deployment][Grasp Planner] Objects Detected!
   [easy_manipulation_deployment][Grasp Planner] Gripper Type Detected: finger

If a valid grasp is found, the grasp planner will also show the grasp quality of the grasp selected

.. code-block:: bash

   [easy_manipulation_deployment][Grasp Planner] Best Grasp found! GDI Score: 2941

The grasp planner will now be publishing the grasp poses for the object to be used in the :code:`grasp_execution` stage of the manipulation pipeline. 

Next we will look at how to visually check these grasp poses before running the grasp execution: :ref:`grasp_validator_example` 

