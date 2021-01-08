.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _workcell_builder_example_manipulator:

Adding A Manipulator
=============================

Before you can add an end effector you need to first connect a robot to the world. Check the :code:`Include Robot` and click :code:`Add Robot`


In the :code:`Robot Brand` field , choose :code:`Universal Robot` 

In the :code:`Robot Model` field, choose :code:`ur5` 

In the :code:`Robot Base Link` field, choose :code:`base_link` 

In the :code:`Robot End Effector Link` field, choose :code:`ee_link` 

Since we want the robot to be at the origin point of the world, we can leave the origin field unchecked. Your final window should look like this:

.. image:: ./images/example/example_load_robot.png


After clicking :code:`Ok`, you returning to the scene creation window, there should be a confirmation that the robot is loaded and the option to include an end effector 

.. image:: ./images/example/example_robot_loaded.png

Next step: :ref:`workcell_builder_example_ee`
