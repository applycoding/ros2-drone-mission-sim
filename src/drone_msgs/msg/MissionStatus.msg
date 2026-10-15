# Planner progress. state is IDLE, LOADED, RUNNING, COMPLETE, ABORTED, RTL, or FAILSAFE.
std_msgs/Header header
string state
int32 current_index
int32 waypoint_count
string current_name
float64 progress
string detail
