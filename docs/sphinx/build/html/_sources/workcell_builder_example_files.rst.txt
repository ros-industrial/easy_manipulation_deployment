.. easy_manipulation_deployment documentation master file, created by
   sphinx-quickstart on Thu Oct 22 11:03:35 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _workcell_builder_example_files:

Generating files and folders
-----------------------------------

After creating the scene, you should see the name of your scene in the dropdown menu.

.. image:: ./images/example/example_no_files.png

At this point, if you check your directory at :code:`/workcell_ws/src/scenes/` , you should see your :code:`new_scene` package being generated

.. image:: ./images/example/example_directory_scenes.png

However, while the :code:`CMakelists.txt` and :code:`package.xml` files are generated, the rest of the other files are not. We need to first click on the :code:`Generate Yaml file from scenes` . If successful, you should see the following message

.. image:: ./images/example/example_yaml_generated.png

To see what was the yaml that was generated, open the file at :code:`/workcell_ws/src/scenes/new_scene/environment.yaml`

.. image:: ./images/example/example_environment_yaml.png

Next, we will need to generate the rest of the relevant files and folders for the scenes. Click on :code:`Generate files from yaml` In the window. If successful, you will be prompted to exit the gui. 

.. image:: ./images/example/example_scenegen_success.png

Click :code:`exit`

Next step: :ref:`workcell_builder_example_check_scene`
