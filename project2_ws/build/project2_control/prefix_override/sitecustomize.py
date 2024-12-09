import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/leoo/Documents/GitHub/ENPM662_Project2/project2_ws/install/project2_control'
