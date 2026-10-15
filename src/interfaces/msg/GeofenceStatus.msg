# Cylinder geofence check in the local ENU frame.
# distance_to_boundary_m is the tightest margin: positive inside, negative outside.
std_msgs/Header header
bool inside
float64 distance_to_boundary_m
string breach_type
