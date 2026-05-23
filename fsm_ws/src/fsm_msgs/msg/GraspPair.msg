uint8 MODE_DUAL=0
uint8 MODE_LEFT_ONLY=1
uint8 MODE_RIGHT_ONLY=2
uint8 PHASE_LEFT=0
uint8 PHASE_RIGHT=1

string pair_id
string task_id
uint8 wall_index
uint8 phase
string left_slot_id
string right_slot_id
geometry_msgs/PoseStamped left_box_pose_robot
geometry_msgs/PoseStamped right_box_pose_robot
geometry_msgs/Vector3 left_box_size
geometry_msgs/Vector3 right_box_size
geometry_msgs/PoseStamped fixed_place_pose_robot
uint8 grasp_mode
