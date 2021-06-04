.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_parameters_general_ee:

Grasp Planner General Parameters (End Effectors)
========================================================

Parameters that define the end effectors involved in grasp planning. Multiple end effectors can be declared in one config file

.. code-block:: bash

   grasp_planning_node:
     ros__parameters:
       ....
       end_effectors:
         end_effector_names: [finger_gripper_1, suction_gripper_1]
         finger_gripper_1:
           ....
         suction_gripper_1:
           .....



end_effectors.end_effector_names
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   end_effector_names: [finger_gripper_1, suction_gripper_1]

.. list-table::
   :widths: 5 20
   :header-rows: 0
   :stub-columns: 1

   * - Description
     - The array of names for end effectors used for grasp planning
   * - Type
     - String array

.. note:: Make sure the names here matches the name of the end effector in corresponding end effector parameters.
