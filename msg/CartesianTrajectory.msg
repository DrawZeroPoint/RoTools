# If header.frame_id is given, it should be the same as the reference frame
Header header

# ref_frame [opt]: The reference frame of the pose. If not given, will be inferred with the context.
# ee_frame [opt]: The distal (end-effector) frame of the pose. If not given, will be inferred with the context.
# All poses in points represent the ee_frame's pose wrt the ref_frame
string ref_frame
string ee_frame

# goal_type [opt]: Type of the poses in the trajectory, which could be:
# BASE_ABS (0): The default. The trajectory poses are wrt the ref_frame.
# BASE_REL (1): The poses are wrt the ref_frame transferred to current ee_frame's origin.
# EEF (2): The poses are wrt the ee_frame.
uint8 BASE_ABS=0
uint8 BASE_REL=1
uint8 EEF=2
uint8 goal_type

# trajectory_type [optional]: Type of the trajectory, which could be:
# CUBIC (0): The default. The trajectory is generated with cubic hermite approach. The result is a cubic spline
#            not only linear in velocity at the start and end points, but also continuous in the curve and its
#            first and second derivatives. In other words, it is second-order continuous.
# FIRSTHOLD (1): The trajectory is first-order continuous. The result is a straight line between the specified
#                start and end points, and the speed is constant. Since the trajectory is linear,
#                the acceleration of the curve may be discontinuous at the start and end points.
uint8 CUBIC=0
uint8 FIRSTHOLD=1
uint8 trajectory_type

# Trajectory points containing poses, twists, accelerations, and point to point duration.
CartesianTrajectoryPoint[] points
