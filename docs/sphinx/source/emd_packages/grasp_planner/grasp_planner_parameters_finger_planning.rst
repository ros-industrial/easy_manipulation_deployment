.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_parameters_finger_planning:

Grasp Planner Finger Parameters (Planning)
========================================================

These parameters directly affect the grasp planning aspects of the finger gripper.

To find out more about how the grasp is being ranked, go to :ref:`grasp_planner_theory_finger`


.. code-block:: bash

   grasp_planning_params:
     grasp_plane_dist_limit: 0.007
     voxel_size: 0.01
     grasp_rank_weight_1: 1.5
     grasp_rank_weight_2: 1.0
     world_x_angle_threshold: 0.5
     world_y_angle_threshold: 0.5
     world_z_angle_threshold: 0.25


<finger_gripper_name>.grasp_planning_params.grasp_plane_dist_limit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   num_sample_along_axis: 3

.. list-table::
   :widths: 5 20
   :header-rows: 0
   :stub-columns: 1

   * - Description
     - Determine the distance from the grasp plane which to determine the grasp area
   * - Type
     - Int

.. note:: The greater the number, the more points included in the grasp area, which increases accuracy,
          but also increases grasp planning times.

<finger_gripper_name>.grasp_planning_params.voxel_size
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   search_resolution: 0.01

.. list-table::
   :widths: 5 20
   :header-rows: 0
   :stub-columns: 1

   * - Description
     - Determines the voxel size for downsampling of grasp clusters.
   * - Type
     - Double

This parameter determines how much downsampling is done after grasp clusters are determined.

.. note:: The smaller the voxel size, the less downsampling is done, which means more grasp samples can be
          generated, but it means that grasp planning times will increase


<finger_gripper_name>.grasp_planning_params.grasp_rank_weight_1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: bash

   grasp_rank_weight_1: 1.5

.. list-table::
   :widths: 5 20
   :header-rows: 0
   :stub-columns: 1

   * - Description
     - Weight for first ranking portion of finger gripper
   * - Type
     - Double

<finger_gripper_name>.grasp_planning_params.grasp_rank_weight_2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   grasp_rank_weight_2: 1.0

.. list-table::
   :widths: 5 20
   :header-rows: 0
   :stub-columns: 1

   * - Description
     - Weight for second ranking portion of finger gripper
   * - Type
     - Double

<finger_gripper_name>.grasp_planning_params.world_x_angle_threshold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Currently not used

<finger_gripper_name>.grasp_planning_params.world_y_angle_threshold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Currently not used

<finger_gripper_name>.grasp_planning_params.world_z_angle_threshold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Currently not used
