.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_parameters_general_viz:

Grasp Planner General Parameters (Visualization)
========================================================

The parameters described here are the configuration components related to grasp visualization

.. code-block:: bash

   grasp_planning_node:
     ros__parameters:
       .....
       visualization_params:
         point_cloud_visualization: true

visualization_params.point_cloud_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   point_cloud_visualization: true

.. list-table::
   :widths: 5 20
   :header-rows: 0
   :stub-columns: 1

   * - Description
     - Provides 3D visualization of the grasp samples using PCL Visualizer
   * - Type
     - bool

.. warning:: If you set this parameter to :code:`true`,  the PCL Visualizer will be spun, and your grasp plans will be displayed.
             this is a blocking process, so your grasp plans will not be published until you exit the Visualizer. **Thus it is
             recommended to leave this as** :code:`false`

             To move to the next grasp sample, press :code:`q` on your keyboard with the Visualizer window selected to move to the
             next grasp sample.
