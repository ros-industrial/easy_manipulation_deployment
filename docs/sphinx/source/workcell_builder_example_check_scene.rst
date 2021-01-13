.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _workcell_builder_example_check_scene:

Check if scene is properly generated
-------------------------------------

Next, to check if the scene is properly created, we will try running the package. In :code:`/workcell_ws/` ,

.. code-block:: bash

   source /opt/ros/foxy/setup.bash

   cd ~/workcell_ws/src
   
   colcon build
   
   source install/setup.bash
   
   ros2 launch new_scene demo.launch
   

RViz should be launched, and you should see your workspace in the simulation.

.. image:: ./images/example/example_generated_sim.png

Now we have a simulated set up of the scene. However, typically a manipulation system would need some form of perception system, which will be addresed next: Next step: :ref:`workcell_builder_example_camera` 
