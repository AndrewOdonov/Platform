from Robot_library_functions import RobotControl
RC = RobotControl()

while True:
    IR_values = RC.IR_read() # ученик  запрашивает функцию для чтения данных с ИК
   # print(IR_values)
    threshold = 160 # пороговое начение, сигнализирующее о том, что линия под ИК датчиком
    try:
        s1, s2, s3, s4, s5 = int(IR_values[2:5]), int(IR_values[5:8]), int(IR_values[8:11]), int(IR_values[11:14]), int(
            IR_values[14:17])
        print(s1, s2, s3, s4, s5)
        if s2 > threshold and s3 > threshold and s4 > threshold:
            RC.IR_move_wheels('FORWARD')
            print('FORWARD')
        elif s2 > threshold:
            RC.IR_move_wheels('RIGHT')
            print('RIGHT')
        elif s4 > threshold:
            RC.IR_move_wheels('LEFT')
            print('LEFT')
        elif s3 > threshold:
            RC.IR_move_wheels('FORWARD')
            print('FORWARD')
        elif s1 > threshold:
            RC.IR_move_wheels('RIGHT')
            print('RIGHT')
        elif s5 > threshold:
            RC.IR_move_wheels('LEFT')
            print('LEFT')
        else:
            RC.IR_move_wheels_repeat()
    except Exception as e:
        pass
    finally:
        print(IR_values)
