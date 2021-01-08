.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner:

Grasp Planner
========================================================

**Motivation**

Traditionally, grasp pose generation is done through using convolutional neural networks (CNNs) to achieve grasp plans. The issues with using machine learning and neural networks is several fold

1. Computation power required for fast grasp pose planning using CNNs.
2. Dataset for training neural netwoks are currently restricted to 2 finger grippers (notably, the `Cornell Grasping Dataset <http://pr.cs.cornell.edu/grasping/rect_data/data.php>`_ has been the most comprehensive and well labelled dataset for current grasp planning neural networks) 



.. image:: ./images/cornell_grasp_display.jpeg
   :align: center

3. Training of new types of grippers require **manual labelling** of new datasets (labour intensive).
4. Accurate and stable grasp poses may not be available for **irregular objects**, so specifically labelled dataset is needed in order to generate accurate grasps

This ROS2 package presents a solution that requires no training, no labelling and little computational power to generate a 3 + 1 DOF grasp poses. The modular design of this package also allows for expansion into other gripper types. Current support for this package includes **2 finger gripper and single suction cup gripper**.

.. |original1| image:: ./images/original_1.jpg   
   :scale: 10%
   :align: middle
.. |finger1| image:: ./images/finger_2_1_grasp.png
   :scale: 10%
   :align: middle
.. |suction1| image:: ./images/suction_1_1_1_grasp.png
   :scale: 10%
   :align: middle
   
.. |original2| image:: ./images/original_2.png   
   :scale: 20%
   :align: middle
.. |finger2| image:: ./images/finger_2_2_grasp.png
   :scale: 10%
   :align: middle
.. |suction2| image:: ./images/suction_1_1_2_grasp.png
   :scale: 10%
   :align: middle

+------------------+----------------------+-----------------------------+
| Input image      | 2 Finger Gripper     | Single Cup Suction Gripper  |
+==================+======================+=============================+
|   |original1|    |      |finger1|       |         |suction1|          |
+------------------+----------------------+-----------------------------+
|   |original2|    |      |finger2|       |         |suction2|          |
+------------------+----------------------+-----------------------------+


.. toctree::
   :maxdepth: 2
   
   grasp_planner_before
   grasp_planner_run
   grasp_validator

