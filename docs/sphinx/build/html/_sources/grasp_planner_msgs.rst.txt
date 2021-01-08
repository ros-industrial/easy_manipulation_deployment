.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _Grasp Planner Message Definitions:

Grasp Planner Message Definitions
========================================================

Package Subcriber
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is the topic name and message type required for the grasp planner package. The `easy_perception_deployment <https://github.com/ros-industrial/easy_perception_deployment/>`_ package follows this convention, but you can use any other perception system as long as it adheres to the following:

Topic Name :  `/perception_output`
----------------------------------

Message Name: RectOutput.msg
-----------------------------
+---------------+--------------------------------+------------------------------------------------------------------------+
| Message name  | Field Type                     | Explanation                                                            |
+===============+================================+========================================================================+
| header        | std_msgs/Header                | General information from the camera                                    |
+---------------+--------------------------------+------------------------------------------------------------------------+
| objects       | DlObject[]                     | Information about the object (refer below to the DlOBject message type.|
+---------------+--------------------------------+------------------------------------------------------------------------+
| frame_width   | uint32                         | Width of the depth image                                               |
+---------------+--------------------------------+------------------------------------------------------------------------+
| frame_height  | uint32                         | Height of the depth image                                              |
+---------------+--------------------------------+------------------------------------------------------------------------+
| num_objects   | uint32                         | Number of objects in scene                                             |
+---------------+--------------------------------+------------------------------------------------------------------------+
| depth_image   | sensor_msgs/Image              | Depth image of the work area                                           |
+---------------+--------------------------------+------------------------------------------------------------------------+
| camera_info   | sensor_msgs/CameraInfo         | Camera-specific information                                            |
+---------------+--------------------------------+------------------------------------------------------------------------+
| roi_array     | sensor_msgs/RegionOfInterest[] | Array of bounding boxes containing the objects                         |
+---------------+--------------------------------+------------------------------------------------------------------------+

Message Name: DlObject.msg
---------------------------
+---------------+-------------------------------+--------------------------+
| Message name | Field Type                     | Explanation              |
+==============+==============================+============================+
| name         | string                       | Name of object             |
+--------------+------------------------------+----------------------------+
| pos          | geometry_msgs/PoseStamped    | Pose of object             | 
+--------------+------------------------------+----------------------------+
| roi          | sensor_msgs/RegionOfInterest | Bounding Box for the object|
+--------------+------------------------------+----------------------------+
| breadth      | float64                      | Real object breadth        |  
+--------------+------------------------------+----------------------------+
| length       | float64                      | Real object breadth        |
+--------------+------------------------------+----------------------------+
| height       | float64                      | Real object height         |
+--------------+------------------------------+----------------------------+


Package Publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This package consists of a publisher that publishes to the following topic with the message structure shown

Topic Name :  `/grasp_poses`
-------------------------------
Message Name: GraspPose.msg
-------------------------------
+---------------+-----------------------------+----------------------------------------------------------------------------+
| Message name  | Field Type                  | Explanation                                                                |
+===============+=============================+============================================================================+
| num_objects   | uint32                      | Number of grasp objects in the scene                                       |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| grasp_poses   | geometry_msgs/PoseStamped[] | Array of grasp object poses                                                |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| object_poses  | geometry_msgs/PoseStamped[] | Array of grasp poses                                                       |
+---------------+-----------------------------+----------------------------------------------------------------------------+
| object_shapes | shape_msgs/SolidPrimitive[] | Array of object shapes (Used to create collision objects for path planning)|
+---------------+-----------------------------+----------------------------------------------------------------------------+


