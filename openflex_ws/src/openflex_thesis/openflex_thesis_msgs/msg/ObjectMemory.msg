# Event flags are a bit mask.
uint16 EVENT_NONE=0
uint16 EVENT_MOVED=1
uint16 EVENT_LOST=2
uint16 EVENT_REOBSERVED=4
uint16 EVENT_AFFORDANCE_INVALID=8

std_msgs/Header header
string object_id
geometry_msgs/PoseWithCovariance pose
float32 confidence
float32 freshness
bool visible
bool grasp_authorized
uint32 map_version
uint16 event_flags
uint32 observation_sequence
string source
