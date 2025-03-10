# Task.msg
int32 task_id
int32 robot_id
int32 shelf_id
string item
int32 item_amount
geometry_msgs/Pose shelf_location
string task_type  # e.g., "pickup", "dropoff"