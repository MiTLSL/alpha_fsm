import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sevnova/ros2_ws/install/isaac_visualization_bridge'
