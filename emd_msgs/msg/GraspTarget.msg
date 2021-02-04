# Define the target for grasping.
# This contains the target object information and the evaluated grasp method

# Target object type(detection label), shape and pose
string target_type
shape_msgs/SolidPrimitive target_shape
geometry_msgs/PoseStamped target_pose

# Evaluated grasp method(s)
emd_msgs/GraspMethod[] grasp_methods
