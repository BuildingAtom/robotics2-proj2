# A basic message for the pose (simplified for 2D)
# Estimates the pose of the robot's origin (the most forward point on the circle)

Header header       # Reference shenanigans (frame is always world)

float64 x           # x position in world frame in meters (inf if N/A)
float64 y           # y position in world frame in meters (inf if N/A)
float64 theta       # theta orientation in world frame in radians (inf if N/A)

