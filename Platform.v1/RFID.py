from Robot_test_library_functions import RobotControl

RC = RobotControl()
RFID_last_value = '000'

while True:
    RFID_values = RC.RFID_read()
    if RFID_values != RFID_last_value:
        print(RFID_values)
