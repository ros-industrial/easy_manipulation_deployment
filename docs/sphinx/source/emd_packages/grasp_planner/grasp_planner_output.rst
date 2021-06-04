.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_output:

Grasp Planner Output Message Types
====================================

This section provides an understanding of the output from the EMD Grasp Planner. The output from the planner is typically
provided to the Grasp Execution Component of EMD, but you can also provide your own grasp execution solutions that takes in such
messages.


The EMD Grasp Planner consists of a ROS2 client that submits a request of the list of objects to be picked and how to pick them.

.. rubric:: Service Name :  `grasp_requests`

GraspRequest.srv
-------------------------------
- Service representing the entire pick and place operation. Contains a list of items (GraspTargets) to be grasped in the scene
 
+---------------+-----------------------------+----------------------------------------------------------------------------+
| Request name  | Field Type                  | Explanation                                                                |
+===============+=============================+============================================================================+
| grasp_targets | GraspTarget[]               | Array of Grasp Targets (Refer below to GraspTarget message type)           |
+---------------+-----------------------------+----------------------------------------------------------------------------+

+---------------+-----------------------------+----------------------------------------------------------------------------+
| Result name   | Field Type                  | Explanation                                                                |
+===============+=============================+============================================================================+
| success       | bool                        | Indicates successful run of triggered service                              |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| message       | string                      | Any other useful information from Grasp Execution                          |
+---------------+-----------------------------+----------------------------------------------------------------------------+

GraspTarget.msg
-------------------------------
- Represents a single object to be picked. Contains a list of end effector grasp plans (GraspMethods)

+---------------+-----------------------------+----------------------------------------------------------------------------+
| Message name  | Field Type                  | Explanation                                                                |
+===============+=============================+============================================================================+
| target_type   | string                      | Object ID. Object Names will be used if using EPD Workflow                 |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| target_pose   | geometry_msgs/PoseStamped   | Position and Orientation of target Object                                  |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| target_shape  | shape_msgs/SolidPrimitive   | Shape of target object (Used to create collision objects for path planning)|
+---------------+-----------------------------+----------------------------------------------------------------------------+
| grasp_methods | GraspMethod[]               | Array of Grasp Targets (Refer below to GraspMethod message type)           |
+---------------+-----------------------------+----------------------------------------------------------------------------+

GraspMethod.msg
-------------------------------
- Represents a Single end effector option. Contains a list grasp poses for that gripper, sorted by ranks
  
+---------------+-----------------------------+----------------------------------------------------------------------------+
| Message name  | Field Type                  | Explanation                                                                |
+===============+=============================+============================================================================+
| ee_id         | string                      | Name of End effector                                                       |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| grasp_poses   | geometry_msgs/PoseStamped[] | Array of grasp poses                                                       |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| grasp_ranks   | float32[]                   | Array of grasp ranks                                                       |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| grasp_markers | visualization_msgs/Marker[] | Array of markers representing grasp samples                                |
+---------------+-----------------------------+----------------------------------------------------------------------------+

