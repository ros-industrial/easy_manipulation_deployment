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
* Contributors: Glenn Tan, Chen Bainian, Khairul Amin, Chia Tse En
