uint8 IDLE=0
uint8 ARMED=1
uint8 TAKEOFF=2
uint8 WAYPOINT=3
uint8 RTL=4
uint8 LAND=5
uint8 COMPLETE=6
uint8 FAILSAFE=7

std_msgs/Header header
uint8 state
string state_name
uint32 waypoint_index
uint32 waypoint_count
uint32 waypoints_reached
string failsafe_reason
float64 distance_to_target
bool geofence_inside
