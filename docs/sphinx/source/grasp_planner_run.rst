.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_run:

Running the Grasp Planner
========================================================

To run the grasp planner:

.. code-block:: bash

   ros2 run grasp_planning grasp_planning_node

The package will then show the following when waiting for the perception topic

.. code-block:: bash

   [easy_manipulation_deployment][Grasp Planner] Waiting for topic....

**Example Grasp Planning Output:**

Finger gripper:

.. code-block:: bash

   [easy_manipulation_deployment][Grasp Planner] Waiting for topic....
   [easy_manipulation_deployment][Grasp Planner] Objects Detected!
   [easy_manipulation_deployment][Grasp Planner] Gripper Type Detected: finger
   [easy_manipulation_deployment][Grasp Planner] Generating Finger grasps
   [easy_manipulation_deployment][Grasp Planner] Number of Grasp Samples: 15
   [easy_manipulation_deployment][Grasp Planner] Best Grasp found! GDI Score: 2941
   [easy_manipulation_deployment][Grasp Planner] Generate Result Files... 
   [easy_manipulation_deployment][Grasp Planner] Time elapsed(ms): 445

Suction gripper:

.. code-block:: bash

   [easy_manipulation_deployment][Grasp Planner] Waiting for topic....
   [easy_manipulation_deployment][Grasp Planner] Objects Detected!
   [easy_manipulation_deployment][Grasp Planner] Gripper Type Detected: suction
   [easy_manipulation_deployment][Grasp Planner] Generating Suction Cup grasps
   [easy_manipulation_deployment][Grasp Planner] Best Grasp found! GDI Score: 226
   [easy_manipulation_deployment][Grasp Planner] Generate Result Files... 
   [easy_manipulation_deployment][Grasp Planner] Time elapsed(ms): 802
