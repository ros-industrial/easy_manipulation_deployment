.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _grasp_planner_parameters_general_epd:

Grasp Planner General Parameters (EPD)
========================================================

The parameters described here are the configuration components related to the
`easy_perception_deployment <https://github.com/ros-industrial/easy_perception_deployment/>`_ ROS2 Package

.. code-block:: bash

   grasp_planning_node:
     ros__parameters:
       .....
       easy_perception_deployment:
         epd_enabled: false
         tracking_enabled: false
         epd_topic: "/processor/epd_localize_output"


easy_perception_deployment.epd_enabled
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. list-table::
   :widths: 5 20
   :header-rows: 0
   :stub-columns: 1

   * - Description
     - Enables the use of the EPD workflow
   * - Type
     - bool

if :code:`true`, EPD Workflow is triggered

if :code:`false`, Direct Camera Workflow is triggered

Details of the different workflows can be found here: :ref:`grasp_planner_input` 

easy_perception_deployment.tracking_enabled
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note:: This parameter will only be used if  :code:`epd_enabled` is set to :code:`true`

.. list-table::
   :widths: 5 20
   :header-rows: 0
   :stub-columns: 1

   * - Description
     - Enables the use of EPD Precision Level 3 Object Tracking
   * - Type
     - bool

if :code:`true`, EPD Precision Level 3, Object Tracking will be taken as input.

if :code:`false`, EPD Precision Level 2, Object Localization will be taken as input.

To understand more about the different precision levels, visit the
`easy_perception_deployment documentation <https://easy-perception-deployment.readthedocs.io/en/latest/>`_

To find out more about the Precision Level ROS2 message differences: :ref:`grasp_planner_input` 

easy_perception_deployment.epd_topic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. note:: This parameter will only be used if  :code:`epd_enabled` is set to :code:`true`


.. list-table::
   :widths: 5 20
   :header-rows: 0
   :stub-columns: 1

   * - Description
     - Topic name of output from the `easy_perception_deployment <https://github.com/ros-industrial/easy_perception_deployment/>`_ package
   * - Type
     - string

If your :code:`tracking_enabled` was set to :code:`true` , The default value of :code:`epd_topic` should be :code:`"/processor/epd_tracking_output"`

If your :code:`tracking_enabled` was set to :code:`false` , The default value of :code:`epd_topic` should be :code:`"/processor/epd_localize_output"`

