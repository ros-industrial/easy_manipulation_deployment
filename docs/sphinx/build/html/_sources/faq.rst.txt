.. _faq:

Frequently Asked Questions
========================================================

Workcell Builder
^^^^^^^^^^^^^^^^^^^^^^^^^^

*How many Robots are supported in workcell generation?*
-----------------------------------------------------------

The current Workcell Builder only supports **one robot and one end effector**. Future plans may involve supporting multiple grippers

*I have my own object_description folders for existing objects that I want to to load into the scene. How do I load it into the Workcell Builder?*
---------------------------------------------------------------------------------------------------------------------------------------------------

It is currently not possible to load your own objects into the scene. You need to create it in the GUI. Object loading features will be included in the future versions of this package very soon.

*Can I create my own robot and end effector from the Workcell builder?*
--------------------------------------------------------------------------

The current version of the Workcell builder does not support robot and end effector creation. There are many existing repositories of robots from major robot vendors such as `Universal Robots <https://github.com/ros-industrial/universal_robot>`_ , `ABB <https://github.com/ros-industrial/abb>`_ `Fanuc <https://github.com/ros-industrial/fanuc_experimental>`_ and end effectors from vendors such as `Robotiq <https://github.com/ros-industrial/robotiq>`_

*How do I visualize the workspace during editing using the GUI*
------------------------------------------------------------------

Currently, you are not able to visualize the workcell during editing, but rather, using the demo.launch. Future improvements may include a real time visualization of the scene as you change the GUI parameters.


Grasp Planner
^^^^^^^^^^^^^^^^^^^^^^^^^^

*Can I use my own perception system with this package*
--------------------------------------------------------

Yes! While it is highly recommended to use the `easy_perception_deployment <https://github.com/ros-industrial/easy_perception_deployment/>`_ package for seamless integration, you can use your own perception system, but make sure to follow the :ref:`Grasp Planner Message Definitions`

*Can I use a 3D camera to plan grasps?*
--------------------------------------------

Yes you can, but make sure that the camera is able to generate depth images for grasp planning. **This package does not currently support point clouds**.

*Is it possible to do a side grasp rather than a top down grasp?*
-------------------------------------------------------------------

Yes, but your camera would then be required to then face the side which you want to grasp.




