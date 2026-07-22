std_msgs/Header header

uint8 DISABLED=0
uint8 WAITING_INPUTS=1
uint8 READY_DRY_RUN=2
uint8 STREAMING=3
uint8 FAULT=4

uint8 state
string state_name
bool static_gate_allowed
bool runtime_gate_enabled
bool preview_received
bool preview_fresh
bool vehicle_state_fresh
bool mavros_connected
bool output_active
string reason
