# If header.frame_id is given, it should be the same as the reference frame
Header header

# ref_frame [opt]: The reference frame of the pose. If not given, will be inferred with the context.
# ee_frame [opt]: The distal (end-effector) frame of the pose. If not given, will be inferred with the context.
# All poses in points represent the ee_frame's pose wrt the ref_frame
string ref_frame
string ee_frame

# goal_type [opt]: Type of the pose in the trajectory, could be:
# BASE_ABS (0): The trajectory pose is wrt the ref_frame. By default goal_type=0.
# BASE_REL (1): The pose is wrt the ref_frame transferred to current ee_frame's origin.
# EEF (2): The pose is wrt the ee_frame.
uint8 BASE_ABS=0
uint8 BASE_REL=1
uint8 EEF=2
uint8 goal_type

# Trajectory points containing poses, twists, accelerations, time_from_start
CartesianTrajectoryPoint[] points
