from Robot_library import RobotControl

rc = RobotControl()
us_sensors_and_values = {}

while True:
    us_data = rc.us_read()
    print(us_data)
    for i in us_data:
        us_sensors_and_values.update({i[2]: int(i[3:6])})

    print('f', us_sensors_and_values.get('F'))
    print('cl', us_sensors_and_values.get('K'))
    print('cr', us_sensors_and_values.get('J'))

    s3 = us_sensors_and_values.get('F')
    s2 = us_sensors_and_values.get('K')
    s1 = us_sensors_and_values.get('J')

    if s3 > 20:
        rc.set_wheels_frequency('200', '200')
        print('FORWARD')
    if s1 < 20:
        rc.set_wheels_frequency('-300', '300')
        print('LEFT')
    if s2 < 20:
        rc.set_wheels_frequency('300', '-300')
        print('RIGHT')
