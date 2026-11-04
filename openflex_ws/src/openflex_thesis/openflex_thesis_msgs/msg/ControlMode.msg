uint8 HOLD=0
uint8 NAV2=1
uint8 ACT=2
uint8 RECOVERY=3
uint8 PLACE=4
# MOVEIT is a mission-level ownership guard: the base output is locked to zero
# while MoveIt owns lift, arm and gripper execution.
uint8 MOVEIT=5

std_msgs/Header header
uint8 mode
uint32 epoch
string owner
string reason
bool inputs_healthy
