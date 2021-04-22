.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_run:

Running the Grasp Planner
========================================================

- Copy any of the files found in ``grasp_planner/example/launch``, rename it to ``grasp_planner_(end_effector)_launch.py`` and replace 
  the ``params_(end_effector).yaml`` within the launch file with the name of the .yaml file you have created.

.. rubric:: Build the workspace and source

.. code-block:: bash

   cd ~/workcell_ws
   colcon build
   source /install/setup.bash


.. rubric:: To run the grasp planner:

.. code-block:: bash

   ros2 launch grasp_planner grasp_planner_(end_effector)_launch.py

The package will then show the following when waiting for the perception topic

.. code-block:: bash

   [pcl_test_node-1] waiting...

- A blank Cloud Viewer window will pop up 
  
.. rubric:: Proceed to run the perception topic 

Example Grasp Planning Output:
------------------------------


.. rubric:: Finger gripper:

.. code-block:: bash

   [pcl_test_node-1]  Direct Point Cloud Grasp Planning:
   [pcl_test_node-1] indExt
   [pcl_test_node-1] indExt
   [pcl_test_node-1] Grasp planning time for computeCloudNormal 280[ms]
   [pcl_test_node-1] dim 00.00935828
   [pcl_test_node-1] dim 10.0627482
   [pcl_test_node-1] dim 20.14448
   [pcl_test_node-1] [Grasp Planning] Finger Gripper Available.
   [pcl_test_node-1] getCuttingPlanes
   [pcl_test_node-1] getGraspCloud
   [pcl_test_node-1] getInitialSamplePoints
   [pcl_test_node-1] It is oriented with the X axis
   [pcl_test_node-1] getInitialSampleCloud
   [pcl_test_node-1] voxelizeSampleCloud
   [pcl_test_node-1] getMaxMinValues
   [pcl_test_node-1] getFingerSamples
   [pcl_test_node-1] getGripperClusters
   [pcl_test_node-1] getAllGripperConfigs
   [pcl_test_node-1] end getAllGripperConfigs: 
   [pcl_test_node-1] grasp_samples size: 1

- 1. Proceed to click on the Cloud Viewer window and it will show the pointcloud and bounding box of the object (Use the mouse scroll to view the pointclouds better).
- 2. Press the ``Q`` key within the Cloud Viewer window to view the results of the grasp_samples
- 3. The terminal running grasp_planner_launch.py will show the ranks of all ranked grasps on the object and the total number of grasps that can be sampled for the object.
- 4. First grasp visualized on the viewer is the best grasp.
- 5. Pressing ``Q`` will show the rest of the consecutively ranked grasps.
- 6. Once all the grasps have been screened through, the grasp_planner will publish the /grasp_tasks topic.

.. warning:: If the pointclouds shown on Cloud Viewer is not satisfactory, adjust the passthrough_filter_limits parameters defined in :ref:`grasp_planner_configuration` to suit to your
             desired environment.


.. rubric:: Suction gripper:

.. code-block:: bash

   [pcl_test_node-1]  Direct Point Cloud Grasp Planning
   [pcl_test_node-1] indExt
   [pcl_test_node-1] indExt
   [pcl_test_node-1] Grasp planning time for computeCloudNormal 457[ms]
   [pcl_test_node-1] dim 00.00935828
   [pcl_test_node-1] dim 10.0627482
   [pcl_test_node-1] dim 20.14448
   [pcl_test_node-1] [Grasp Planning] Suction Gripper Available.
   [pcl_test_node-1] length dim: 0
   [pcl_test_node-1] breadth dim: 0
   [pcl_test_node-1] Object center: x -0.0138821
   [pcl_test_node-1] Object center: y 0.0386563
   [pcl_test_node-1] Object center: z 0.590018
   [pcl_test_node-1] object_top_point: x 0.0388006
   [pcl_test_node-1] object_top_point: y 0.0453913
   [pcl_test_node-1] object_top_point: z 0.582
   [pcl_test_node-1] Grasp planning time for suction_cup : 257[ms]

- 1. Proceed to click on the Cloud Viewer window and it will show the pointcloud and bounding box of the object (Use the mouse scroll to view the pointclouds better).
- 2. Press the ``Q`` key within the Cloud Viewer window to view the results of the grasp_samples
- 3. The terminal running grasp_planner_launch.py will show the ranks of all ranked grasps on the object and the total number of grasps that can be sampled for the object.
- 4. First grasp visualized on the viewer is the best grasp.
- 5. Pressing ``Q`` will show the rest of the ranked grasps consecutively.
- 6. Once all the grasps have been screened through, the grasp_planner will publish the /grasp_tasks topic.

.. warning:: If the pointclouds shown on Cloud Viewer is not satisfactory, adjust the passthrough_filter_limits parameters defined in :ref:`grasp_planner_configuration` to suit to your
             desired environment.

The pose and orientation of the top ranked grasp will then be published for :ref:`grasp_execution_example`
