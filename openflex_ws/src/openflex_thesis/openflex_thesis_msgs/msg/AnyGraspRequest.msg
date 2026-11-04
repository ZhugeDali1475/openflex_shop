# Topic-side request used by anygrasp_gateway to trigger an external inference process.
std_msgs/Header header
string request_id
ObjectMemory target
uint8 stage
uint8 active_arm
uint16 top_k
