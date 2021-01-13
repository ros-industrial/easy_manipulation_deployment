<!-- # Unsupervised Learning, depth based approach to generating 3 + 1 DOF grasp poses for novel objects -->

### This ROS2 package provides an __algorithmic, depth based approach__ to  generate a __3+1 Degree of Freedom (DOF) Grasp pose__ from a depth image. 

[![Build Status](https://travis-ci.com/gtan039/algorithmic-depth-based-grasp-planner.svg?token=YGHiT6MgKhAnSEAAQbHk&branch=master)](https://travis-ci.com/gtan039/algorithmic-depth-based-grasp-planner)

# Introduction

Traditionally, grasp pose generation is done through using convolutional neural networks (CNNs) to achieve grasp plans. The issues with using machine learning and neural networks is several fold

1. Computation power required for fast grasp pose planning using CNNs.
2. Dataset for training neural netwoks are currently restricted to 2 finger grippers (notably, the [Cornell Grasping Dataset](http://pr.cs.cornell.edu/grasping/rect_data/data.php) has been the most comprehensive and well labelled dataset for current grasp planning neural networks) 

<!--     ![Cornell Labelled Example](./images/cornell_grasp_display.jpeg)-->

3. Training of new types of grippers require __manual labelling__ of new datasets (labour intensive).
4. Accurate and stable grasp poses may not be available for __irregular objects__, so specifically labelled dataset is needed in order to generate accurate grasps

## Package example
This ROS2 package presents a solution that requires no training, no labelling and little computational power to generate a 3 + 1 DOF grasp poses. The modular design of this package also allows for expansion into other gripper types. Current support for this package includes __2 finger gripper and single suction cup gripper__

<br />

# Package Details

## Assumptions
### Grasp object poses
This package assumes a __top down__ camera set up overlooking the grasping area. Assuming that the axis through the image plane is the z axis, grasp poses will consider a __one dimensional change in orientation__ in the z-direction (resulting in a 3+1 DOF grasp pose)

### Image Quality
This package was tested using an input depth image from the Intel Realsense D415 RGB-D camera. Certain variations may occur if a different camera is used

### Work Surface
This package assumes that the object is placed on a relatively flat surface. As this package requires the use of the depth image from the camera above the work surface, we assume that the distance from the work surface to the camera is constant. 

### End Effector Support
This package currently supports a 2 Finger gripper and a single suction cup gripper. Further development will include multiple finger gripper and suction cup array support.

## Package subscriber
This package consists of a subscriber that subscribes to the following topic with the message structure shown

Topic Name :  `/perception_output`

Message Name: RectOutput.msg

| Message name  | Field Type                     | Explanation |
| ------------- |--------------------------------| -----|
| header        | std_msgs/Header                | General information from the camera |
| objects       | DlObject[]                     | Information about the object (refer below to the DlOBject message type.|
| frame_width   | uint32                         | Width of the depth image |
| frame_height  | uint32                         | Height of the depth image |
| num_objects   | uint32                         | Number of objects in scene  |
| depth_image   | sensor_msgs/Image              | Depth image of the work area |
| camera_info   | sensor_msgs/CameraInfo         | Camera-specific information |
| roi_array     | sensor_msgs/RegionOfInterest[] | Array of bounding boxes containing the objects |

Message Name: DlObject.msg

| Message name | Field Type                     | Explanation |
| ------------ |--------------------------------| -----|
| name         | string                       | Name of object |
| pos          | geometry_msgs/PoseStamped    | Pose of object |
| roi          | sensor_msgs/RegionOfInterest | Bounding Box for the object|
| breadth      | float64                      | Real object breadth |
| length       | float64                      | Real object breadth  |
| height       | float64                      | Real object height |


## Package Publisher
This package consists of a publisher that publishes to the following topic with the message structure shown

Topic Name :  `/grasp_poses`

| Message name  | Field Type                  | Explanation  |
| ------------- |-----------------------------| -----|
| num_objects   | uint32                      | Number of grasp objects in the scene|
| grasp_poses   | geometry_msgs/PoseStamped[] | Array of grasp object poses|
| object_poses  | geometry_msgs/PoseStamped[] | Array of grasp poses|
| object_shapes | shape_msgs/SolidPrimitive[] | Array of object shapes (Used to create collision objects for path planning)|

<br />

---

## Configuring grasp attributes

In order for the grasp planner to plan the right type of grasp, we need to first create a configuration file in the config folder , ```attributes.yaml```. It is advised to write over the current ```attributes.yaml``` file to prevent any YAML parsing errors

### Finger Gripper

#### fingers
Number of fingers for the end effector. __currently only 2 fingered grippers are supported__
#### distance_between_fingers

The distance between the fingers of the end effector(in mm). Using the Robotiq 2F-85 gripper as an example: 

#### longest_gripper_dim

The longest dimensions of a finger (in mm). Using the robotiq 2F-85 gripper:

#### table_height

The distance between the camera used to capture the workspace and the surface on which the object is on.

### Suction Gripper

#### length_cups
The number of cups in the length dimension. __Currently only support value of 1__
#### breadth_cups
The number of cups in the breadth dimension. __Currently only support value of 1__
#### radius
The radius one of the suction cup.
#### table_height

The distance between the camera used to capture the workspace and the surface on which the object is on.

---

# How the package works

This package uses information from the depth image captured by the camera to generate grasp samples, and the quality of the grasps will be determined by a Grasp Decide Index (GDI).

This concept was demonstrated in [this paper](https://arxiv.org/pdf/2001.05856.pdf), with changes in the method of sampling potential points, due to the abstraction of the perception component of the system to a separate perception package. There is also an additonal support for single suction cup grippers. 

Given that a two finger gripper and a single suction cup gripper are considered to be the "base cases" for a finger gripper and a suction cup array, this provides opportunities for expansion to more complicated grippers beyond the scope of the paper.



