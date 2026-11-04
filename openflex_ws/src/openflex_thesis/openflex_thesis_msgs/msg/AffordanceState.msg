# Compact TC-WAM condition consumed by the local policy and mission guard.
std_msgs/Header header
string object_id
uint32 map_version
uint8 active_arm
uint32 selected_grasp_id
float32 selected_lift_position
string planning_group
string ready_region_id
geometry_msgs/Pose target_in_base
geometry_msgs/Pose2D preferred_base_delta
geometry_msgs/PoseStamped best_base_pose
ReadyRegion ready_region
float32 current_score
float32 best_score
float32 reachability
float32 joint_margin
float32 robustness
bool ready
bool valid
bool physical_evaluation
bool object_visible
bool grasp_authorized
# Canonical order is documented in config/act_schema.yaml.
float32[] compact_state
