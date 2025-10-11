import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sh/projects/f1tenth_miru3/install/mouse_teleop'
