import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/behrouz/ws24-multi-robot-task-distribution/src/install/turtlebot3_teleop'
