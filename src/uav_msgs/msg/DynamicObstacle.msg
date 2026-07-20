std_msgs/Header header

uint8 TRACKED=0
uint8 OCCLUDED=1
uint8 LOST=2

uint32 id
uint8 tracking_state
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 size
float64[9] position_covariance
float32 confidence
PredictedState[] prediction
