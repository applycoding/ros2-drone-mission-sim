# Battery reading published by the kinematic demo.
# voltage is a linear 3S-pack display value, not a cell model.
std_msgs/Header header
float32 percent
float32 voltage
float32 drain_pct_per_s
bool low
bool critical
