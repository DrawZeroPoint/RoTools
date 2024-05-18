# If header.frame_id is given, it should be the same as the reference frame
Header header

# All poses in points represent ee_frame's pose wrt the ref_frame
string ref_frame
string ee_frame

# Trajectory points containing poses, twists, accelerations, time_from_start
CartesianTrajectoryPoint[] points

# Defines the time the group should move to the first pose in the trajectory from its initial pose, in sec
# If it is not given, DEFAULT_INITIAL_DURATION will be used.
float64 DEFAULT_INITIAL_DURATION=4.0
float64 initial_duration
