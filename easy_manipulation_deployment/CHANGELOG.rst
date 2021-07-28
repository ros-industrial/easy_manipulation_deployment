0.1.1 (2021-07-28)
------------------
* Fixed bug that triggers Grasp Execution when no grasp plans found.
* Added a clearing and repopulating method in Grasp Planner for end effector to prevent retaining of past information
* Fixed inconsistencies in Grasp Planner Rotation by adding additional configuration parameters to file to specify end effector tf alignments
* Fixed initial point generation for finger grasp samples in Grasp Planner to account for object axis vector
* Added quick dimension and pose fix to ensure grasp object collision shape is properly formed for grasp execution.
* Added EPD Tracking output support to Grasp Planner
* Added Service based communication between EPD and EMD Grasp Planner
* Removed epd_msgs as a required dependency for EMD, and removed epd_msgs from main repository.
* Updated MoveItCppPtr namespace from ``moveit::planning_interface`` to ``moveit_cpp`` for Moveit 2.2.0 due to deprecation notice
* Fix for ignoring links
* Separate ``configure()`` from execution in ds_async_executor
* Overhaul several scheduler bugs
* Use chrono instead of rclcpp::time
* Contributors: Glenn Tan, Chen Bainian, Wong Wen Kang, Khairul Amin

0.1.0 (2021-06-04)
------------------
* Changed Grasp Planner methodology to implement a different mathematical approach.
* Restructured Grasp Planner libraries to Grasp Object, End Effector, and Grasp Scene for clear separation of different required functions
* Converted from using depth images to using Point Clouds to provide more information about the grasp area, using the Point Cloud Library
* Added support for collision detection using the Flexible Collision Library
* Added methods to generate multifinger gripper and suction cup array grasp samples.
* Added a more comprehensive configuration file for grasp planner to account for multifinger/suction array support, and point cloud support
* Added Ranking Algorithm that supports Multifinger gripper and suction cup array for Grasp Planner support
* Added unit tests for all added features for grasp planner
* Removed all documentation to a separate repository (Both doxygen and sphinx)
* Add in grasp_execution workcell context parameterization
* Temporary fix for MoveIt2 Execution timeout problem
* Switch to ROS2 Service Based Communication Between Grasp Execution and Grasp Planner to prevent execution interruption and grasping of repeated objects
* Fix speed changing problem
* Experimental Moveit2 octomap integration
* Added dynamic safety Executor plugin and update controller to latest
* Amended grasp_execution.rviz file to include grasp_markers and pointcloud topics
* Move grasp_execution inside easy_manipulation_deployment folder
* Add initial grasp_execution package
* Contributors: Glenn Tan, Chen Bainian, Khairul Amin, Chia Tse En
