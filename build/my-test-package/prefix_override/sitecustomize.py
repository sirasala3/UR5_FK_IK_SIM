import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shiva/sixDOF_robot_ws_full_checks/install/my-test-package'
