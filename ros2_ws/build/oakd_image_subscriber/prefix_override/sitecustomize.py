import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rokey-kim/Documents/ros2_ws/install/oakd_image_subscriber'
