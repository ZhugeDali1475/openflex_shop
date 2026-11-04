# One AnyGrasp TCP hypothesis. Poses are already transformed into the frame in
# each PoseStamped header and must use the calibrated MoveIt TCP convention.
uint8 STAGE_COARSE=1
uint8 STAGE_REFINED=2

uint32 grasp_id
uint8 stage
geometry_msgs/PoseStamped grasp_pose
geometry_msgs/PoseStamped pregrasp_pose
geometry_msgs/Vector3 approach
float32 width
float32 quality
float32 visibility_score
float32 collision_score
uint32 observation_sequence
bool physical_perception
string source
