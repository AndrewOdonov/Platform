import time
from Robot_test_library_functions import RobotControl

rc = RobotControl()

rc.set_wheels_frequency('300', '300')
time.sleep(2)
rc.set_wheels_frequency('000', '000')
