# One LiDAR obstacle cluster (sense_node -> plan_node inside Scene), map frame.
# Built from consecutive non-road returns of one scan (road-edge returns at
# |y| ~ 3.5 m are removed using the published road geometry). Units: m.
# The centroid of a partially seen object lies on its visible surface, so
# x/y/radius describe the visible extent, not the true object centre/size.

float64 x            # centroid of the member returns
float64 y
float64 radius       # max distance of a member return from the centroid
float64 range_min    # closest return to the sensor
uint32 n_points
