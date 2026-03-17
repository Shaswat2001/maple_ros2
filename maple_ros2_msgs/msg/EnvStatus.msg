# Status of a MAPLE environment container
string env_id
string backend_name
string device
string host
uint16 port
string status          # "serving", "setup", "running", "error", "stopped"
string current_task
string current_instruction
uint32 episode_step
float64 episode_reward
builtin_interfaces/Time stamp
