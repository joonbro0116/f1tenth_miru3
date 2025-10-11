import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/moon/sim_ws/src/bound_obstacle_detector/install/bound_obstacle_detector'
