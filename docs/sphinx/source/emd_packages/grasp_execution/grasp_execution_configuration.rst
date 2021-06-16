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

.. code-block:: bash

   initial_positions:
     shoulder_pan_joint: 1.57
     shoulder_lift_joint: -2.35
     elbow_joint: 1.83
     wrist_1_joint: -1.03
     wrist_2_joint: -1.57
     wrist_3_joint: 0.0


Changing the fake object published
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To change the location and dimensions of the fake object published, edit following parameters in the file ``grasp_execution/example/config/fake_grasp_pose_publisher.yaml``:

.. code-block:: bash

   fake_grasp_pose_publisher:
     ros__parameters:
       interface: service
       frame_id: camera_frame
       ee_id: robotiq_2f
       grasp_pose: [0.0126, 0.0322, 0.442,
                    0.0, 0.0, 0.9997, 0.0250]

       object_pose: [0.0128, 0.0330, 0.443,
                     0.0, 0.0, 0.9997, 0.0250]

       object_dimensions: [0.087, 0.167, 0.023]

       delay: 1.5

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

.. code-block:: bash

   grasp_execution_node:
     ros__parameters:
       planning_scene_monitor_options:
         name: "planning_scene_monitor"
         robot_description: "robot_description"
         joint_state_topic: "/joint_states"
         attached_collision_object_topic: "/moveit_cpp/planning_scene_monitor"
         publish_planning_scene_topic: "/moveit_cpp/publish_planning_scene"
         monitored_planning_scene_topic: "/moveit_cpp/monitored_planning_scene"
         wait_for_initial_state_timeout: 10.0

       planning_pipelines:
         #namespace: "moveit_cpp"  # optional, default is ~
         pipeline_names: ["ompl"]

       plan_request_params:
         planning_attempts: 1
         planning_time: 0.5
         planning_pipeline: ompl
         max_velocity_scaling_factor: 1.0
         max_acceleration_scaling_factor: 1.0

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

.. code-block:: bash

   dynamic_safety_demo_node:
     ros__parameters:
       planning_scene_monitor_options:
         name: "planning_scene_monitor"
         robot_description: "robot_description"
         joint_state_topic: "/joint_states"
         attached_collision_object_topic: "/moveit_cpp/planning_scene_monitor"
         publish_planning_scene_topic: "/moveit_cpp/publish_planning_scene"
         monitored_planning_scene_topic: "/moveit_cpp/monitored_planning_scene"
         wait_for_initial_state_timeout: 10.0

       planning_pipelines:
         #namespace: "moveit_cpp"  # optional, default is ~
         pipeline_names: ["ompl"]

       plan_request_params:
         planning_attempts: 1
         planning_time: 0.5
         planning_pipeline: ompl
         max_velocity_scaling_factor: 1.0
         max_acceleration_scaling_factor: 1.0

       # Load octomap
       load_octomap: true

       # Dynamic safety parameters
       rate: 20
       allow_replan: true
       visualize: true

       safety_zone:
         manual: true
         unit_type: second
         collision_checking_deadline: 0.05
         slow_down_time: 0.2
         replan_deadline: 1.2
         look_ahead_time: 1.65

       collision_checker:
         distance: false
         continuous: false
         step: 0.1
         thread_count: 8
         realtime: false

       next_point_publisher:
         command_out_type: "trajectory_msgs/JointTrajectory"
         publish_joint_position: true
         publish_joint_velocity: false
         publish_joint_effort: false

       replanner:
         planner_name: ompl

       visualizer:
         publish_frequency: 10
         step: 0.1
         topic: "/dynamic_safety/displayed_state"

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

.. code-block:: bash

   workcell:
   - group_name: manipulator
     executors:
       default:
         plugin: grasp_execution/DefaultExecutor
       ds_async:
         plugin: grasp_execution/DynamicSafetyAsyncExecutor
         controller: ur5_arm_controller
     end_effectors:
       robotiq_2f0:
         brand: robotiq_2f
         link: ee_palm
         clearance: 0.1
         driver:
           plugin: grasp_execution/DummyGripperDriver
           controller: ""

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
