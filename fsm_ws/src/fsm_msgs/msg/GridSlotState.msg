uint8 STATUS_UNKNOWN=0
uint8 STATUS_OCCUPIED=1
uint8 STATUS_REMOVED=2
uint8 STATUS_FAILED=3
uint8 STATUS_BLOCKED=4

string slot_id
uint8 wall_index
uint8 row_index
uint8 col_index
uint8 status
geometry_msgs/PoseStamped expected_pose_robot
geometry_msgs/PoseStamped latest_pose_robot
bool visible
float32 confidence
uint8 retry_count
builtin_interfaces/Time last_seen_time
uint16 last_error_code
