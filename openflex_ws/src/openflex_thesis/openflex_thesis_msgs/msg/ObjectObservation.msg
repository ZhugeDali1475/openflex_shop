# A single timestamped object observation. The pose is expressed in header.frame_id.
std_msgs/Header header
string object_id
geometry_msgs/PoseWithCovariance pose
float32 confidence
bool visible
uint32 sequence
string source
