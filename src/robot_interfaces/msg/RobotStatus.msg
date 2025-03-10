# msg/RobotStatus.msg
int32 robot_id
geometry_msgs/Pose current_location
float32 battery_level
bool is_available
string status