.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_validator_example:

Grasp Validator Example
========================================================

This part of the example follows the previous :ref:`grasp_planner_example` . After running the grasp planner via 

.. code-block:: bash

   ros2 run grasp_planning grasp_planning_node
   
the :code:`grasps.txt` file will be generated in :code:`/workcell_ws/src/easy_manipulation_deployment/grasp_validator/results` . These are the 4 coordinate points of the grasp representation of the 2 fingered gripper. 

Make sure that you have the sample depth image before running the validator. For this example the depth image, :code:`depth_img.jpg` is provided in :code:`/workcell_ws/src/easy_manipulation_deployment/grasp_validator/images`. First we need to label the pixel points representing the object. 

.. image:: ./images/example/depth_img.jpg


To run the depth labeller:


.. code-block:: bash

   ros2 run grasp_validator depth_checker
   
You should now see a labelled version of the depth image. Press any key to exit. The labelled object, :code:`object.jpg` will be used for the grasp validator. 

.. image:: ./images/example/example_object.png
   

To visualize the previously generated grasps, run the grasp validator:

.. code-block:: bash

   ros2 run grasp_validator validator

You should then see a visualization of the grasp as seen below

.. image:: ./images/example/example_validator.png

Once the grasp plan is verified, we will execute the grasp plan in a simulation with the manipulator: :ref:`grasp_execution_example`
