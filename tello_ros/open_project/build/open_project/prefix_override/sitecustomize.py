import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/petri/common_project_drone_race/tello_ros/open_project/install/open_project'
