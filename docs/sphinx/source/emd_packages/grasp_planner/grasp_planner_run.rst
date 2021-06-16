.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_run:

Running the Grasp Planner
========================================================

To properly run the Grasp Planner, there are 3 main components needed for running the grasp planner

.. note:: Ensure that for these three terminals, the ROS2 distributions are sourced, and the workspace is built and sourced as well

1. Perception source

   Perception Source can be provided via Direct Camera Input, or via the Easy Perception Deployment ROS2 package.

2. Package Publishing the TF of the camera frame

   The Grasp Planner uses a `TF2 Message Filter <http://wiki.ros.org/tf2/Tutorials/Using%20stamped%20datatypes%20with%20tf2%3A%3AMessageFilter>`_
   that waits until the camera frame is published before triggering the planning itself, thus ensure that whatever package you are using is
   publishing the tf of the camera frame. You can configure the camera frame name directly in the grasp planner configuration file. More can be
   found here: :ref:`grasp_planner_parameters_general_camera`

   The EMD Grasp Execution Component provides such a tf publisher feature, thus you can use that as well, by running

.. code-block:: bash

   ros2 launch grasp_execution grasp_execution.launch.py

3. EMD Grasp Planner

- Copy any of the files found in ``grasp_planner/example/launch``, rename it to ``grasp_planner_(end_effector)_launch.py`` and replace
  the ``params_(end_effector).yaml`` within the launch file with the name of the .yaml file you have created. More can be
  found here: :ref:`grasp_planner_parameters`

To run the grasp planner, run the following command

.. code-block:: bash

   ros2 launch grasp_planner grasp_planner_(end_effector)_launch.py


The package will then show the following when waiting for the perception topic

.. code-block:: bash

   [pcl_test_node-1] waiting...

.. note:: A blank Cloud Viewer window will pop up, but will only be used if the :code:`point_cloud_visualization` parameter in the config file
          is true.

Example Grasp Planning Output:
------------------------------

.. rubric:: Finger gripper:

.. code-block:: bash

   [demo_node-1] [INFO] [1622278539.845614356] [GraspScene]: Using Direct Camera Input....
   [demo_node-1] [INFO] [1622278539.847663649] [GraspScene]: waiting....
   [demo_node-1] [INFO] [1622278568.222839760] [GraspScene]: Camera Point Cloud Received!
   [demo_node-1] [INFO] [1622278568.222866905] [GraspScene]: Processing Point Cloud...
   [demo_node-1] [INFO] [1622278568.314758446] [GraspScene]: Applying Passthrough filters
   [demo_node-1] [INFO] [1622278568.388353728] [GraspScene]: Removing Statistical Outlier
   [demo_node-1] [INFO] [1622278569.016152941] [GraspScene]: Downsampling Point Cloud
   [demo_node-1] [INFO] [1622278569.025191278] [GraspScene]: Segmenting plane
   [demo_node-1] [INFO] [1622278569.052835633] [GraspScene]: Point cloud successfully processed!
   [demo_node-1] [INFO] [1622278574.641794991] [GraspScene]: Extracting Objects from point cloud
   [demo_node-1] [INFO] [1622278575.542850053] [GraspScene]: Extracted 1 from point cloud
   [demo_node-1] [INFO] [1622278575.542983933] [GraspScene]: Loading finger gripper robotiq_2f
   [demo_node-1] [INFO] [1622278575.543278724] [GraspScene]: All End Effectors Loaded
   [demo_node-1] [INFO] [1622278575.723888039] [GraspScene]: Grasp planning time for robotiq_2f 10 [ms]
   [demo_node-1] [INFO] [1622278575.723914241] [GraspScene]: 19 Grasp Samples have been generated.


- 1. Proceed to click on the Cloud Viewer window and it will show the point cloud and bounding box of the object (Use the mouse scroll to view the point clouds better).
- 2. Press the ``Q`` key within the Cloud Viewer window to view the results of the grasp_samples
- 3. The terminal running grasp_planner_launch.py will show the ranks of all ranked grasps on the object and the total number of grasps that can be sampled for the object.
- 4. First grasp visualized on the viewer is the best grasp.
- 5. Pressing ``Q`` will show the rest of the consecutively ranked grasps.
- 6. Once all the grasps have been screened through, the grasp_planner will publish the /grasp_tasks topic.

.. warning:: If the point clouds shown on Cloud Viewer is not satisfactory, adjust the passthrough_filter_limits parameters defined in :ref:`grasp_planner_parameters` to
             suit to your desired environment.


.. rubric:: Suction gripper:

.. code-block:: bash

   [demo_node-1] [INFO] [1622278648.176338963] [GraspScene]: Using Direct Camera Input....
   [demo_node-1] [INFO] [1622278648.177783690] [GraspScene]: waiting....
   [demo_node-1] [INFO] [1622278652.348536010] [GraspScene]: Camera Point Cloud Received!
   [demo_node-1] [INFO] [1622278652.348569074] [GraspScene]: Processing Point Cloud...
   [demo_node-1] [INFO] [1622278652.453359019] [GraspScene]: Applying Passthrough filters
   [demo_node-1] [INFO] [1622278652.531642625] [GraspScene]: Removing Statistical Outlier
   [demo_node-1] [INFO] [1622278653.242787447] [GraspScene]: Downsampling Point Cloud
   [demo_node-1] [INFO] [1622278653.257366582] [GraspScene]: Segmenting plane
   [demo_node-1] [INFO] [1622278653.289915130] [GraspScene]: Point cloud successfully processed!
   [demo_node-1] [INFO] [1622278721.836112062] [GraspScene]: Extracting Objects from point cloud
   [demo_node-1] [INFO] [1622278722.761648258] [GraspScene]: Extracted 1 from point cloud
   [demo_node-1] [INFO] [1622278722.761778564] [GraspScene]: Loading suction gripper suction_cup
   [demo_node-1] [INFO] [1622278722.761936044] [GraspScene]: All End Effectors Loaded
   [demo_node-1] [INFO] [1622278723.371410203] [GraspScene]: Grasp planning time for suction_cup 490 [ms]
   [demo_node-1] [INFO] [1622278723.371440840] [GraspScene]: 64 Grasp Samples have been generated.


- 1. Proceed to click on the Cloud Viewer window and it will show the point cloud and bounding box of the object (Use the mouse scroll to view the point clouds better).
- 2. Press the ``Q`` key within the Cloud Viewer window to view the results of the grasp_samples
- 3. The terminal running grasp_planner_launch.py will show the ranks of all ranked grasps on the object and the total number of grasps that can be sampled for the object.
- 4. First grasp visualized on the viewer is the best grasp.
- 5. Pressing ``Q`` will show the rest of the ranked grasps consecutively.
- 6. Once all the grasps have been screened through, the grasp_planner will publish the /grasp_tasks topic.

.. warning:: If the point clouds shown on Cloud Viewer is not satisfactory, adjust the passthrough_filter_limits parameters defined in :ref:`grasp_planner_parameters`
             to suit to your desired environment.

The pose and orientation of the top ranked grasp will then be published for :ref:`grasp_execution_example`
