import serial
import cv2
import numpy as np
import sys
import time


class RobotControl:
    """"settings"""
    ser = serial.Serial('/dev/ttyUSB0')
    ser.baudrate = 115200
    # resize frame
    percent = 23.4375
    w = int(640 * percent / 100)
    h = int(480 * percent / 100)
    dim = (w, h)
    # Parameters for undistort frame from lower camera
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

    def __init__(self):
        self.US_must_be_different_sensors = []
        self.data_5_US = []
        self.frame = []
        self.UV_last_value = 0

    def encode_and_send_msg(self, msg):
        """
        encode and send message from computer to module of control
        :param msg: str
        :return:
        """
        msg = msg.encode('utf-8')
        self.ser.write(msg)

    def read_and_decode(self):
        """
        read and decode data from module of control
        :return:
        """
        try:
            data_from_serial = self.ser.readline()
            decoded = data_from_serial.decode()
            return decoded
        except Exception:
            return None

    def prepare_msg_of_one_wheel(self, wheel_speed):
        """
        prepare commands for set speed of wheels
        :param wheel_speed:
        :return:
        """
        if wheel_speed[0] != '-':
            if len(str(wheel_speed)) < 3:
                wheel_speed = (3 - len(str(wheel_speed))) * '0' + str(wheel_speed)
            msg_wheel = '+00' + wheel_speed
        else:
            if len(str(wheel_speed)[1:]) < 3:
                wheel_speed = (3 - len(str(wheel_speed))) * '0' + str(wheel_speed)
            msg_wheel = '-00' + wheel_speed[1:4]
        return msg_wheel

    def set_wheels_frequency(self, right_wheel_speed, left_wheel_speed):
        """
        set speed of wheels
        :param right_wheel_speed:
        :param left_wheel_speed:
        :return:
        """
        right_wheel_speed = str(right_wheel_speed)
        right_wheel_msg = self.prepare_msg_of_one_wheel(right_wheel_speed)
        left_wheel_speed = str(left_wheel_speed)
        left_wheel_msg = self.prepare_msg_of_one_wheel(left_wheel_speed)
        msg = 'ST0' + right_wheel_msg + left_wheel_msg + 'E'
        print(msg)
        print(msg.encode('utf-8'))
        self.encode_and_send_msg(msg)

    def set_wheels_movement(self, moving):
        """
        set speed of wheels with direction of movement
        :param moving:
        :return:
        """
        moving = moving.upper()
        msg = self.simple_ride_movements_dict.get(moving)
        self.encode_and_send_msg(msg)

    def stop_platform(self):
        self.encode_and_send_msg(self.msg_stop_driving)

    def turn_on_flashlight(self):
        """
        turn on flashlight
        :return:
        """
        self.encode_and_send_msg(self.msg_turn_on_fl)

    def turn_off_flashlight(self):
        """
        turn off flashlight
        :return:
        """
        msg = self.msg_turn_off_flashlight_and_UV
        self.encode_and_send_msg(msg)

    def analysis_illumination(self, img, thrshld):
        """
        define the level of lighting on frame
        :param thrshld:
        :return:
        """
        is_light = np.mean(img) > thrshld
        print(np.mean(img))  # вывести уровень света на кадре
        return 'light' if is_light else 'dark'

    def turn_on_uv(self, brightness):
        """
        turn on uv light
        :return:
        """
        brightness = str(brightness)
        if len(str(brightness)) < 3:
            brightness = (3 - len(str(brightness))) * '0' + str(brightness)
        msg = 'SU++000' + brightness + '00000E'
        self.encode_and_send_msg(msg)

    def turn_off_uv(self):
        """
        turn off uv light
        :return:
        """
        msg = self.msg_turn_off_flashlight_and_UV
        self.encode_and_send_msg(msg)

    def set_angle_servo(self):
        """
        set angle of servo
        :return:
        """
        # if len(str(grad)) < 3:
        #     grad = (3-len(str(grad))) * '0' + str(grad)
        # msg = 'SS'+str(grad)+'0000000000E'
        # self.encode_and_send_msg(msg)

        self.encode_and_send_msg(self.msg_servo)

    def camera_reading_lower(self):
        """
        read and undistort frame from the lower camera
        :return: frame
        """
        ret, frame = self.cap_lower.read()
        frame = cv2.resize(frame, self.dim, interpolation=cv2.INTER_AREA)  # уменьшаем разрешение фото
        frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)  # избавляемся от дисторсии
        return frame

    def camera_reading_upper(self):
        """
        read frame from upper camera
        :return: frame
        """
        ret, frame = self.cap_upper.read()
        frame = cv2.resize(frame, self.dim, interpolation=cv2.INTER_AREA)  # уменьшаем разрешение фото
        # frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)  # избавляемся от дисторсии
        return frame

    def ir_read(self):
        """
        get data from ir sensors
        :return: self.IR
        """
        try:
            us_ir = self.read_and_decode()
            if us_ir[:2] == 'SI':
                self.IR = us_ir
        except Exception:
            pass
        finally:
            return self.IR

    def ir_drive(self, moving: str):
        """
        set direction of movement for driving on ir sensors
        :param moving: str
        :return:
        """
        try:
            moving = moving.upper()
            msg = self.IR_wheel_movements_dict[moving]
            self.IR_last_msg = msg
            self.encode_and_send_msg(msg)
        except KeyError:
            print('Проверьте написание слова:', moving)
            self.encode_and_send_msg(self.msg_stop_driving)
            sys.exit()

    def ir_move_wheels_repeat(self):
        """
        search the line after her losing
        :return:
        """
        msg = self.IR_last_msg
        self.encode_and_send_msg(msg)

    def us_read(self):
        """
        get data from us sensors
        :return: self.read_data_4_US
        """
        try:
            us_ir = self.read_and_decode()
            if us_ir[:2] == 'SU':
                if us_ir[2] not in self.US_must_be_different_sensors:
                    self.US_must_be_different_sensors.append(us_ir[2])
                    self.data_5_US.append(us_ir)
                    if len(self.US_must_be_different_sensors) == 5:
                        self.US_must_be_different_sensors = []
                    if len(self.data_5_US) == 5:
                        self.read_data_5_US = self.data_5_US
                        self.data_5_US = []
        except Exception:
            pass
        finally:
            return self.read_data_5_US

    def us_drive(self, moving: str):
        """
        set direction of movement for driving on us sensors
        :param moving: str
        :return:
        """
        try:
            moving = moving.upper()
            msg = self.US_wheel_movements_dict[moving]
            print(msg)
            self.encode_and_send_msg(msg)
        except KeyError:
            print('Проверьте написание слова:', moving)
            self.encode_and_send_msg(self.msg_stop_driving)
            sys.exit()

    def line_area_select(self, frame, max_cntr, mask):
        """
        define the area line
        :param frame: np.ndarray
        :param max_cntr: np.ndarray
        :param mask: np.ndarray
        :return: aim_area
        """
        x, y, w, h = cv2.boundingRect(max_cntr)
        black = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)  # ---black in RGB
        max_cntr_area = cv2.rectangle(black, (x, y), (x + w + 10, y + h + 10), (0, 255, 0), -1)
        aim_area = cv2.bitwise_and(frame, max_cntr_area, mask=mask)
        return aim_area

    def splitting_frame_into_parts(self, frame, segment: int, img_height: int, width_of_segment: int, mask):
        """
        split frame on parts
        :param frame: np.ndarray
        :param segment: int
        :param img_height: int
        :param width_of_segment: int
        :param mask: np.ndarray
        :return: aim_area_part
        """
        black = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)
        grid = cv2.rectangle(black, (0, segment), (0 + img_height, segment + width_of_segment), (0, 255, 0), -1)
        aim_area_part = cv2.bitwise_and(frame, grid, mask=mask)
        return aim_area_part

    def uv_drive(self, moving: str, x_point: int):
        """
        set direction of movement for driving on uv line
        :param moving: str
        :param x_point: int
        :return:
        """
        try:
            moving = moving.upper()
            msg = self.UV_wheel_movements_dict[moving]
            self.UV_last_value = x_point
            self.encode_and_send_msg(msg)
        except KeyError:
            print('Проверьте написание слова:', moving)
            self.encode_and_send_msg(self.msg_stop_driving)
            sys.exit()

    def accum_read(self):
        try:
            pwr = self.read_and_decode()
            if pwr[:2] == 'SA':
                self.accum = pwr[2:5]
        except Exception:
            pass
        finally:
            return self.accum

    def rfid_read(self):
        """
        get data from rfid reader
        :return:
        """
        # print(n_of_les)
        try:
            data = self.read_and_decode()
            if data[:2] == 'SF':
                self.RFID = data[2:17]
                print(self.RFID)
                # if n_of_les == 'les1':
                #     self.RFID_file = self.RFID_msgs_les1[self.RFID]
                # if n_of_les == 'les2':
                #     self.RFID_file = self.RFID_msgs_les2[self.RFID]
                #     # print(self.RFID_file)

        except Exception:
            pass
        finally:
            return self.RFID_file
