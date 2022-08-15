from Robot_library import RobotControl

rc = RobotControl()

while True:
    IR_values = rc.ir_read()
    threshold = 160
    s1, s2, s3, s4, s5 = int(IR_values[2:5]), int(IR_values[5:8]), int(IR_values[8:11]), \
                         int(IR_values[11:14]), int(IR_values[14:17])
    print(s1, s2, s3, s4, s5)
    if s2 > threshold and s3 > threshold and s4 > threshold:
        rc.ir_move_wheels('FORWARD')
        print('FORWARD')
    elif s1 > threshold or s2 > threshold:
        rc.ir_move_wheels('RIGHT')
        print('RIGHT')
    elif s4 > threshold or s5 > threshold:
        rc.ir_move_wheels('LEFT')
        print('LEFT')
    elif s3 > threshold:
        rc.ir_move_wheels('FORWARD')
        print('FORWARD')
    else:
        rc.ir_move_wheels_repeat()
