.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_parameters_general_camera:

Grasp Planner General Parameters (Camera)
========================================================

Parameters that define the camera parameters. May vary depending on the type of camera used.


.. code-block:: bash

   grasp_planning_node:
     ros__parameters:
       .....
       camera_parameters:
         point_cloud_topic: "/camera/pointcloud"
         camera_frame: "camera_color_optical_frame"

camera_parameters.point_cloud_topic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: bash

   point_cloud_topic: "/camera/pointcloud"

.. list-table::
   :widths: 5 20
   :header-rows: 0
   :stub-columns: 1

   * - Description
     - Topic published by the camera using the `PointCloud2 <http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html>`_
       message type
   * - Type
     - string

camera_parameters.camera_frame
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   camera_frame: "camera_color_optical_frame"

.. list-table::
   :widths: 5 20
   :header-rows: 0
   :stub-columns: 1

   * - Description
     - Tf reference frame which the point cloud is referenced from.
   * - Type
     - string

.. note:: The :code:`camera_frame` value may be different depending on the definition of the URDF. In order to determine what the frame is:

          1. Run the ROS2 package for your camera
          2. Use :code:`ros2 topic echo` command to look at the message published by the camera.

          Typically if the message type has a sub-message
          of `Header <http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html>`_ type, refer to the :code:`frame_id` portion.


