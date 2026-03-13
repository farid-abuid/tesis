import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/farid/tesis/exo_right_arm_ws/install/exo_bringup'
