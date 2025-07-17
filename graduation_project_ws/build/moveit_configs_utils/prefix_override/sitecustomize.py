import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/salem/graduation_project_ws/install/moveit_configs_utils'
