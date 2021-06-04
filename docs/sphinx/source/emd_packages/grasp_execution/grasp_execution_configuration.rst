.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_execution_configuration:

Grasp Execution Configuration Files
========================================================

Grasp Execution has many features and capabilities that can be turned on and off, and customized to the your liking. This is done in the configuration files, that are typically stored in the config folder of your package. The YAML format is used for this file for better understanding and readability.

Below lists the configuration files in the package that change different parameters in the grasp execution pipeline.

* Changing the start positions
* Changing the fake object published
* Changing the grasp execution parameters
* Changing the dynamic safety execution parameters
* Changing the workcell configurations


Changing the start positions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To change the home state, edit the values of each joint in the file ``grasp_execution/example/config/start_positions.yaml``:

.. literalinclude:: /../../../easy_manipulation_deployment/grasp_execution/example/config/start_positions.yaml
   :language: bash


Changing the fake object published
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To change the location and dimensions of the fake object published, edit following parameters in the file ``grasp_execution/example/config/fake_grasp_pose_publisher.yaml``:

.. literalinclude:: /../../../easy_manipulation_deployment/grasp_execution/example/config/fake_grasp_pose_publisher.yaml
   :language: bash

.. list-table::
   :widths: 7 5 20
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - frame_id
     - String
     - Base frame
   * - grasp_pose
     - double array
     - Location that the robot will plan to
   * - object_pose
     - double array
     - Location that the object will spawn at
   * - object_dimensions
     - double array
     - Dimensions of the object (x,y,z)
   * - delay
     - double
     - 


Changing the grasp execution parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To change the configuration of the default grasp execution, edit following parameters in the file ``grasp_execution/example/config/grasp_execution.yaml``:

.. literalinclude:: /../../../easy_manipulation_deployment/grasp_execution/example/config/grasp_execution.yaml
   :language: bash

.. list-table:: planning_scene_monitor_options
   :widths: 7 5 20
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - name
     - string
     - 
   * - robot_description
     - string
     - 
   * - joint_state_topic
     - string
     - 
   * - attached_collision_object_topic
     - string
     - 
   * - publish_planning_scene_topic
     - string
     - 
   * - monitored_planning_scene_topic
     - string
     - 
   * - wait_for_initial_state_timeout
     - double
     - 

.. list-table:: planning_pipelines
   :widths: 7 5 20
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - pipeline_names
     - string array
     - Planning pipelines to be used (as of now only ompl is supported)

.. list-table:: plan_request_params
   :widths: 7 5 20
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - planning_attempts
     - int
     - Number of planning attempts
   * - planning_pipeline
     - string
     - planning pipeline used
   * - max_velocity_scaling_factor
     - double
     - Maximum velocity scale
   * - max_acceleration_scaling_factor
     - double
     - Maximum acceleration scale


Changing the dynamic safety execution parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To change the configuration of the grasp execution with dynamic safety, edit following parameters in the file ``grasp_execution/example/config/dynamic_safety_demo.yaml``:

.. literalinclude:: /../../../easy_manipulation_deployment/grasp_execution/example/config/dynamic_safety_demo.yaml
   :language: bash

The first half of the parameters are the same as described in grasp_execution.yaml

.. list-table::
   :widths: 7 5 20
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - load_octomap
     - bool
     - Load the octomap

.. list-table::
   :widths: 7 5 20
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - rate
     - double
     - 
   * - allow_replan
     - bool
     - Replan if collision is detected. If set to false the robot will simply stop to avoid collision
   * - visualize
     - bool
     - 

.. list-table:: safety_zone
   :widths: 7 5 20
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - manual
     - bool
     - 
   * - unit_type
     - string
     - 
   * - collision_checking_deadline
     - double
     - 
   * - slow_down_time
     - double
     - 
   * - replan_deadline
     - double
     - 
   * - look_ahead_time
     - double
     - 

.. list-table:: collision_checker
   :widths: 7 5 20
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - distance
     - bool
     - 
   * - continuous
     - bool
     - 
   * - step
     - double
     - 
   * - thread_count
     - int
     - 
   * - realtime
     - bool
     - 

.. list-table:: next_point_publisher
   :widths: 7 5 20
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - command_out_type
     - string
     - 
   * - publish_joint_position
     - bool
     - 
   * - publish_joint_velocity
     - bool
     - 
   * - publish_joint_effort
     - bool
     - 

.. list-table:: replanner
   :widths: 7 5 20
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - planner_name
     - string
     - 

.. list-table:: visualizer
   :widths: 7 5 20
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - publish_frequency
     - double
     - 
   * - step
     - double
     - 
   * - topic
     - string
     - 


Changing the workcell configurations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To change the configuration of your workcell, edit following parameters in the file ``grasp_execution/example/config/dynamic_safety_demo.yaml``:

.. literalinclude:: /../../../easy_manipulation_deployment/grasp_execution/example/config/workcell_context.yaml
   :language: bash

.. list-table::
   :widths: 7 5 20
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - group_name
     - string
     - 
   * - end_effectors.robotiq_2f0.link
     - string
     - Tip link of end effector
   * - end_effectors.robotiq_2f0.clearance
     - double
     - Distance above the object that the end effector would plan to before moving down to pick the object up
