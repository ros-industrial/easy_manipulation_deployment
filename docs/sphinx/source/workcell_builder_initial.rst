.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _workcell_initialization:

Workcell Initialization
========================================================

Launching the Workcell Builder GUI
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In your colcon workspace: 

.. code-block:: bash

   $ source /opt/ros/foxy/setup.bash
   
   $ colcon build
   
   $ source install/setup.bash
   
   $ ros2 run workcell_builder workcell_builder


Folder structure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is (Highly) recommended to select the same workspace location as the workspace that you lodaded this package in.

From the main window, choose your workspace location. Ensure that the folder selected is the **main workspace folder**, i.e catkin_ws or colcon_ws

.. image:: ./images/workcell_loaded.png
   :scale: 100%
   :align: center

Once you see the confirmation message that the workcell is loaded, you can then check that folder using your file explorer and you will see the various folders required to store your assets created. Choose the ROS version and distribution required and click next to be directed to the scene select window

Folder Structure for Assets
-----------------------------

This package requires a standardized folder structure in order for the workcell builder to function well. This serves as good practice as well for users to store their files in a logical and standardized format. The following is how your folder should be structured

workcell_ws
   src
      scenes
      assets
         robots
            robot_brand
               robot_model_description
               robot_model_moveit_config
         end_effectors
	    end_effector_brand
	       end_effector_model_description
	       end_effector_model_moveit_config
         environment_objects
            environment_objects_description

The rest of the documentation will highlight how it should be populated.

Uploading Relevant Assets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before generating a scene, you need to make sure you have the assets you need for the scene, especially for the robot and end effector. 

Robots
--------------------------

For increased reusability and ease of visualization, we will create separate folders for separate vendors of Robots. For example, we will create a folder to store UR robots

.. code-block:: bash

   $ cd workcell_ws/src/assets/robots

   $ mkdir universal_robot



Copy over the moveit_config folder and description folders of the relevant robot models you want to add, ensuring that the folder names and file names follow the  [naming conventions](#naming-conventions)


End Effector
--------------------------

Simlarly for End Effectors, we will create a separate folder for each End Effector Vendor. For example, we will create a folder to store Robotiq Grippers

.. code-block:: bash

   $ cd workcell_ws/src/assets/end_effectors

   $ mkdir robotiq

Copy over the moveit_config folder and description folders of the relevant end_effector models you want to add, ensuring that the folder names and file names follow the  [naming conventions](#naming-conventions)

Environment Objects
--------------------------

For objects that is part of the environment that will be used as static collision objects, it should be stored in the **workcell_ws/src/assets/environment folder**. 


***Current version of the GUI does not support loading of existing environment objects. For simple environment objects, consider creating a copy of the environment objects with the gui instead**

Next step: :ref:`Create A Scene`
