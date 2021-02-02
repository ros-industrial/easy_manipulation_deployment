# Define the target for grasping.
# This contains the target object information and the evaluated grasp method

# Target object shape, label and pose
shape_msgs/SolidPrimitive object
string label
geometry_msgs/Pose object_pose

# Evaluated grasp method(s)
emd_msgs/GraspMethod[] methods

# Reference frame ID
string frame_id
