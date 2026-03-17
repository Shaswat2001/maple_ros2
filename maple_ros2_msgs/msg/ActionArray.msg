# Action output from a MAPLE policy
std_msgs/Header header
float64[] action          # Raw action vector from policy
float64[] transformed_action  # Action after adapter transformation
string policy_id
string instruction
float64 inference_time_ms
uint32 step_index
