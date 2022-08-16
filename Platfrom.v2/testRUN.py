""""settings"""
ser = serial.Serial('/dev/ttyUSB0')
ser.baudrate = 115200
# resize frame
percent = 23.4375
w = int(640 * percent / 100)
h = int(480 * percent / 100)
dim = (w, h)
# Parameters for delete distorsion
k = np.array([[669.53947377, 0., 316.57731188],
              [0., 650.21491053, 291.96812174],
              [0., 0., 1.]])
d = np.array([-0.4077693, 0.29706739, -0.00737684, -0.00562703, -0.29700514])
newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(k, d, (w, h), 0)
mapx, mapy = cv2.initUndistortRectifyMap(k, d, None, newcameramatrix, (w, h), 5)

"""messages"""
IR = 'SI010010010010010010010010E'
IR_last_msg = 'ST0+00000+00000E'
read_data_5_US = ['SUF030E\r\n', 'SUL030E\r\n', 'SUR030E\r\n', 'SUJ030E\r\n', 'SUK030E\r\n']
RFID = 'SF000000000000000E'
accum = 'please wait'
RFID_file = '000'
RFID_msgs_les1 = {
    '000000000000000': 'Start!', '087008128181106': 'RFID1.txt',
    '135249194180008': 'RFID2.txt', '151116120090193': 'RFID3.txt',
    '023189072181087': 'RFID4.txt', '199241182180052': 'RFID5.txt',
    '219251025049008': 'RFID6.txt', '011104030049076': 'RFID7.txt',
    '251012029049219': 'RFID8.txt', '235185160049195': 'RFID9.txt',
    '091134030049242': 'RFID10.txt'
}

RFID_msgs_les2 = {
    '000000000000000': 'Start!', '087008128181106': ['RFID1.1.txt', 'RFID1.2.txt', '0'],
    '135249194180008': ['RFID2.txt', '0', '0'],
    '151116120090193': ['RFID3.txt', '0', '0'],
    '023189072181087': ['RFID4.1.txt', 'RFID4.2.txt', '0'],
    '199241182180052': ['RFID5.1.txt', 'RFID5.2.txt', 'RFID5.txt'],
    '219251025049008': ['RFID6.txt', '0', '0'],
    '011104030049076': ['RFID7.txt', '0', '0'],
    '251012029049219': ['RFID8.txt', '0', '0'],
    '235185160049195': ['RFID9.txt', '0', '0']
}
msg_servo = 'SS0800000000000E'
msg_stop_driving = 'ST0+00000+00000E'
msg_turn_on_fl = 'SU++00100000000E'
msg_turn_off_flashlight_and_UV = 'SU--00000000000E'
IR_wheel_movements_dict = {'LEFT': 'ST0-00250+00250E', 'RIGHT': 'ST0+00250-00250E',
                                'FORWARD': 'ST0+00230+00230E'}
US_wheel_movements_dict = {'LEFT': 'ST0+00000+00300E', 'RIGHT': 'ST0+00300+00000E',
                                'FORWARD': 'ST0+00300+00300E'}
UV_wheel_movements_dict = {'SHARPLY_LEFT': 'ST0+00000+00250E', 'SHARPLY_RIGHT': 'ST0+00250+00000E',
                                'FORWARD': 'ST0+00300+00300E',
                                'SOFTLY_RIGHT': 'ST0+00300+00200E', 'RIGHT': 'ST0+00300+00050E',
                                'SOFTLY_LEFT': 'ST0+00200+00300E', 'LEFT': 'ST0+00050+00300E'}
simple_ride_movements_dict = {'LEFT': 'ST0-00250+00250E', 'RIGHT': 'ST0+00250-00250E',
                                   'FORWARD': 'ST0+00250+00250E', 'STOP': 'ST0+00000+00000E',
                                   'BACK': 'ST0-00300-00300E'}

"""checking"""
for i in range(0, 3):
    cap_lower = cv2.VideoCapture(i)  # замена на VideoStream нижняя камера
    i_for_lower = i
    time.sleep(2)
    if cap_lower is None or not cap_lower.isOpened():
        continue
    else:
        break
for i in range(0, 5):
    cap_upper = cv2.VideoCapture(i)  # замена на VideoStream нижняя камера
    i_for_upper = i
    time.sleep(2)
    if cap_upper is None or not cap_upper.isOpened():
        continue
    else:
        break