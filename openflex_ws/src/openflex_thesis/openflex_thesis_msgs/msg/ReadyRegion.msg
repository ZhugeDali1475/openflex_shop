# A ready set is conditioned on exactly one (active_arm, grasp_id, lift solution).
# member_base_poses must never be a union across different grasp or lift solutions.
std_msgs/Header header
string region_id
string object_id
uint32 map_version
uint8 active_arm
uint32 grasp_id
string planning_group
float32 lift_position
float32 score_threshold
geometry_msgs/Pose best_base_pose
uint32[] member_candidate_ids
geometry_msgs/Pose[] member_base_poses
float32 position_tolerance_m
float32 yaw_tolerance_rad
