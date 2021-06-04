.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_parameters_finger:

Grasp Planner Finger Parameters
========================================================

.. code-block:: bash

   grasp_planning_node:
     ros__parameters:
       ...
       end_effectors:
         end_effector_names: [finger_gripper_1, suction_gripper_1]
         finger_gripper_1:
           type: finger
           num_fingers_side_1: 1
           num_fingers_side_2: 1
           distance_between_fingers_1: 0.0
           distance_between_fingers_2: 0.0
           finger_thickness: 0.02
           gripper_stroke: 0.105
           gripper_coordinate_system:
             grasp_stroke_direction: "x"
             grasp_stroke_normal_direction: "y"
             grasp_approach_direction: "z"
           grasp_planning_params:
             grasp_plane_dist_limit: 0.007
             voxel_size: 0.01
             grasp_rank_weight_1: 1.5
             grasp_rank_weight_2: 1.0
             world_x_angle_threshold: 0.5
             world_y_angle_threshold: 0.5
             world_z_angle_threshold: 0.25

Physical Attributes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 2

   grasp_planner_parameters_finger_physical

Coordinate System Attributes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 2

   grasp_planner_parameters_finger_axis

Grasp Planning Attributes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 3

   grasp_planner_parameters_finger_planning



