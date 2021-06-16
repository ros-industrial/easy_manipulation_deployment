.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_parameters_suction:

Grasp Planner Suction Parameters
========================================================

.. code-block:: bash

   grasp_planning_node:
     ros__parameters:
       ...
       end_effectors:
         end_effector_names: [finger_gripper_1, suction_gripper_1]
         suction_gripper_1:
           type: suction
           num_cups_length: 1
           num_cups_breadth: 1
           dist_between_cups_length: 0.06
           dist_between_cups_breadth: 0.06
           cup_radius: 0.005
           cup_height: 0.01
           gripper_coordinate_system:
             length_direction: "x"
             breadth_direction: "y"
             grasp_approach_direction: "z"
           grasp_planning_params:
             num_sample_along_axis: 3
             search_resolution: 0.01
             search_angle_resolution: 4
             weights:
               curvature: 1.0
               grasp_distance_to_center: 1.0
               number_contact_points: 1.0


Physical Attributes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 2

   grasp_planner_parameters_suction_physical

Coordinate System Attributes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 2

   grasp_planner_parameters_suction_axis

Grasp Planning Attributes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 3

   grasp_planner_parameters_suction_planning



