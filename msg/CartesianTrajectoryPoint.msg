# pose: The goal pose of the controlled frame.
# twist: The goal twist of the controlled frame.
# acceleration: The goal acceleration of the controlled frame.
# duration: Time in seconds for reaching the goal pose from the previous goal pose.

geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Accel acceleration
float64 duration
