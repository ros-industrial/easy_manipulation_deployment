.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_msgs:

Grasp Planner Message Definitions
========================================================

Package Subcriber
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You are able to use any perception package to work with the grasp planner, **as long as it follows the message type definitions below**. 
The grasp planner ROS2 subscriber subscribes to either :code:`/processor/epd_localize_output` topic of message type :code:`EPDObjectLocalization.msg` 
**or** any ROS2 :code:`pointcloud` topic of message type :code:`PointCloud2.msg` `Pointcloud <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html>`_ , so make sure that your perception package publishes to a topic in that format. 

For convenience, you can use the `easy_perception_deployment <https://github.com/ros-industrial/easy_perception_deployment/>`_ package which follows this convention.

Subscribing to Easy Perception Deployment topic
------------------------------------------------

.. rubric:: Topic Name :  `/processor/epd_localize_output`


Message Name: EPDObjectLocalization.msg
-----------------------------------------
+---------------+--------------------------------+-------------------------------------------------------------------------------+
| Message name  | Field Type                     | Explanation                                                                   |
+===============+================================+===============================================================================+
| header        | std_msgs/Header                | General information from the camera                                           |
+---------------+--------------------------------+-------------------------------------------------------------------------------+
| objects       | LocalizedObject[]              | Information about the object (refer below to the LocalizedObject message type.|
+---------------+--------------------------------+-------------------------------------------------------------------------------+
| frame_width   | uint32                         | Width of the depth image                                                      |
+---------------+--------------------------------+-------------------------------------------------------------------------------+
| frame_height  | uint32                         | Height of the depth image                                                     |
+---------------+--------------------------------+-------------------------------------------------------------------------------+
| depth_image   | sensor_msgs/Image              | Depth image of the work area                                                  |
+---------------+--------------------------------+-------------------------------------------------------------------------------+
| process_time  | uint32                         |                                                                               |
+---------------+--------------------------------+-------------------------------------------------------------------------------+


Message Name: LocalizedObject.msg
-------------------------------------
+--------------------------------+------------------------------+------------------------------------+
| Message name                   | Field Type                   | Explanation                        |
+==============+==============================+======================================================+
| name                           | string                       | Name of object                     |
+--------------------------------+------------------------------+------------------------------------+
| roi                            | sensor_msgs/RegionOfInterest | Bounding Box for the object        |
+--------------------------------+------------------------------+------------------------------------+
| segmented_binary_mask          | sensor_msgs/Image            | Segmentation mask of the object    |
+--------------------------------+------------------------------+------------------------------------+
| centroid                       | geometry_msgs/PoseStamped    | Object Bounding Box centroid       |
+--------------------------------+------------------------------+------------------------------------+
| breadth                        | float64                      | Real object breadth                |  
+--------------------------------+------------------------------+------------------------------------+
| length                         | float64                      | Real object breadth                |
+--------------------------------+------------------------------+------------------------------------+
| height                         | float64                      | Real object height                 |
+--------------------------------+------------------------------+------------------------------------+
| segmented_pcl                  | sensor_msgs/PointCloud2      | Segmented object pointcloud        |
+--------------------------------+------------------------------+------------------------------------+
| axis                           | geometry_msgs/Vector3        | Axis of Bounding Box               |
+--------------------------------+------------------------------+------------------------------------+


Package Publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This package consists of a publisher that publishes to the following topic with the message structure shown

.. rubric:: Topic Name :  `/grasp_tasks`

Message Name: GraspTask.msg
-------------------------------
- Represents the entire pick and place operation. Contains a list of items (GraspTargets) to be grasped in the scene
  
+---------------+-----------------------------+----------------------------------------------------------------------------+
| Message name  | Field Type                  | Explanation                                                                |
+===============+=============================+============================================================================+
| task_id       | uint32                      |                                                                            |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| grasp_targets | GraspTarget[]               | Array of Grasp Targets (Refer below to GraspTarget message type)           |
+---------------+-----------------------------+----------------------------------------------------------------------------+

Message Name: GraspTarget.msg
-------------------------------
- Represents a single object to be picked. Contains a list of end effector grasp plans (GraspMethods)

+---------------+-----------------------------+----------------------------------------------------------------------------+
| Message name  | Field Type                  | Explanation                                                                |
+===============+=============================+============================================================================+
| target_type   | string                      |                                                                            |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| target_pose   | geometry_msgs/PoseStamped   | Position and Orientation of target Object                                  |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| target_shape  | shape_msgs/SolidPrimitive   | Shape of target object (Used to create collision objects for path planning)|
+---------------+-----------------------------+----------------------------------------------------------------------------+
| grasp_methods | GraspMethod[]               | Array of Grasp Targets (Refer below to GraspMethod message type)           |
+---------------+-----------------------------+----------------------------------------------------------------------------+

Message Name: GraspMethod.msg
-------------------------------
- Represents a Single end effector option. Contains a list grasp poses for that gripper, sorted by ranks
  
+---------------+-----------------------------+----------------------------------------------------------------------------+
| Message name  | Field Type                  | Explanation                                                                |
+===============+=============================+============================================================================+
| ee_id         | string                      |                                                                            |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| grasp_poses   | geometry_msgs/PoseStamped[] | Array of grasp poses                                                       |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| grasp_ranks   | float32[]                   | Array of grank ranks                                                       |
+---------------+-----------------------------+----------------------------------------------------------------------------+


