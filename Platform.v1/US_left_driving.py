from Robot_test_library_functions import RobotControl

RC = RobotControl()
US_sensors_and_values = {}
while True:
    US_data = RC.US_read() # ученик  запрашивает функцию для чтения данных с УЗ
    threshold = 15 # пороговое начение, сигнализирующее о том, что препятствия нет
    print(US_data)
    for i in US_data:
        US_sensors_and_values.update({i[2]: int(i[3:6])})
    print('left_sensor', US_sensors_and_values.get('L'))
    print('corner_sensor_L', US_sensors_and_values.get('K'))
    print('forward_sensor', US_sensors_and_values.get('F'))
    left_wall = US_sensors_and_values.get('L') < threshold
    left_corner = US_sensors_and_values.get('K') < threshold
    front_wall = US_sensors_and_values.get('F') < threshold
    if front_wall:
        RC.US_wheel_movement('RIGHT')
        print('RIGHT')
    else:
        if left_corner:
            RC.US_wheel_movement('RIGHT')
            print('RIGHT')
        elif left_wall:
            RC.US_wheel_movement('FORWARD')
            print('FORWARD')
        else:
            RC.US_wheel_movement('LEFT')
            print("LEFT")