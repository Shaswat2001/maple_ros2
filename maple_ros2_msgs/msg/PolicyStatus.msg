# Status of a MAPLE policy container
string policy_id
string backend_name
string version
string device
string host
uint16 port
string status          # "serving", "loading", "error", "stopped"
float64 last_inference_time_ms
float64 avg_inference_time_ms
uint64 total_inferences
builtin_interfaces/Time stamp
