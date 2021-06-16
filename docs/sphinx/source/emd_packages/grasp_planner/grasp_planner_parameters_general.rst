.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_parameters_general:

Grasp Planner General Parameters
========================================================

The parameters described here are the general configuration components for both finger and suction end effectors. Most of the parameters here are used for either point cloud processing
or ROS2 component definitions.

.. code-block:: bash

   grasp_planning_node:
     ros__parameters:
       grasp_output_service: "grasp_requests"
       easy_perception_deployment:
         epd_enabled: false
         tracking_enabled: false
         epd_topic: "/processor/epd_localize_output"
       camera_parameters:
         point_cloud_topic: "/camera/pointcloud"
         camera_frame: "camera_color_optical_frame"
       point_cloud_params:
         passthrough_filter_limits_x: [-0.50, 0.50]
         passthrough_filter_limits_y: [-0.15, 0.40]
         passthrough_filter_limits_z: [0.01, 0.70]
         segmentation_max_iterations: 50
         segmentation_distance_threshold: 0.01
         cluster_tolerance: 0.01
         min_cluster_size: 750
         cloud_normal_radius: 0.03
         fcl_voxel_size: 0.02
       end_effectors:
         end_effector_names: [finger_gripper_1, suction_gripper_1]
         finger_gripper_1:
           ....
         suction_gripper_1:
           .....
       visualization_params:
         point_cloud_visualization: true

Parameter Descriptions
----------------------------------------

grasp_output_service
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 10 20
   :header-rows: 0
   :stub-columns: 1

   * - Description
     - ROS2 service name for the grasp execution component
   * - Type
     - string

Details of the GraspRequest Service can be found here: :ref:`grasp_planner_output`

It is recommended to use the EMD Grasp Execution component. If you do so, keep set :code:`grasp_output_service` as
:code:`"grasp_requests"`


easy_perception_deployment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 2

   grasp_planner_parameters_general_epd

camera_parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 2

   grasp_planner_parameters_general_camera

point_cloud_params
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 3

   grasp_planner_parameters_general_point_cloud

end_effectors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. toctree::
   :maxdepth: 2

   grasp_planner_parameters_general_ee

visualization_params
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. toctree::
   :maxdepth: 2

   grasp_planner_parameters_general_viz
