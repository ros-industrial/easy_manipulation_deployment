.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_parameters:

Grasp Planner Configuration File
========================================================

The grasp planner aims to be highly customizable, and this customization is done using the configuration file, that is typically stored in the config folder of your package.
The YAML format is used for this file for better understanding and readability.

Due to the huge number of parameters that can be tweaked, the parameters can be divided into the following subgroups, where explanation of each parameter will be provided

.. toctree::
   :maxdepth: 2

   grasp_planner_parameters_general
   grasp_planner_parameters_finger
   grasp_planner_parameters_suction


Sample configuration yaml file
-------------------------------
If you are unsure of how to begin writing a yaml file, the following is an example of the full configuration of the yaml file.


.. code-block:: bash

   grasp_planning_node:
     ros__parameters:
       grasp_output_topic: "/grasp_tasks"
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
       visualization_params:
         point_cloud_visualization: true