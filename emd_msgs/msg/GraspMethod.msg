# Define best / possible method of grasping

# The type of the gripper
string ee_id

# Target grasp pose(s) for the grasp method
geometry_msgs/PoseStamped[] grasp_poses

# Grasp quality for each grasp poses
float64[] grasp_ranks
