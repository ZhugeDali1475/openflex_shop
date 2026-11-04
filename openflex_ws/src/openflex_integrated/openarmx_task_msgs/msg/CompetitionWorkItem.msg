# A concrete, independently executable retail manipulation job.
uint8 PICK_ORDER=1
uint8 RESTOCK=2
uint8 RETURN_MISPLACED=3
uint8 kind

string work_id
string object_id
string sku
string specification
string source_station
string destination_station
string source_slot
string destination_slot
geometry_msgs/PoseStamped observed_pose
float32 confidence
string evidence_id
