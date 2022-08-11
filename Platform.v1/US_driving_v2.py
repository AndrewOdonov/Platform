from Robot_test_library_functions import RobotControl

RC = RobotControl()
US_sensors_and_values = {}

while True:
    US_data = RC.US_read()
    print(US_data)
    for i in US_data:
        US_sensors_and_values.update({i[2]: int(i[3:6])})

    print('f', US_sensors_and_values.get('F'))
    print('cl', US_sensors_and_values.get('K'))
    print('cr', US_sensors_and_values.get('J'))

    f = US_sensors_and_values.get('F')
    cl = US_sensors_and_values.get('K')
    cr = US_sensors_and_values.get('J')

    if f > 20:
        RC.set_wheels_frequency('200', '200')
        print('FORWARD')

    if cr < 20:
        RC.set_wheels_frequency('-300', '300')
        print('LEFT')
    if cl < 20:
        RC.set_wheels_frequency('300', '-300')
        print('RIGHT')
