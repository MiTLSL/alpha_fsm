uint8 SUCCESS_BOTH=0
uint8 SUCCESS_LEFT_ONLY=1
uint8 SUCCESS_RIGHT_ONLY=2
uint8 FAILED_BOTH=3
uint8 ESTOP=4
uint8 CANCELLED=5

string pair_id
uint8 result_code
uint8 left_result
uint8 right_result
string failed_stage
uint16 error_code
float32 vacuum_left_kpa
float32 vacuum_right_kpa
float32 execution_time_sec
geometry_msgs/PoseStamped final_robot_pose
