import serial
import cv2
import numpy as np
import sys
import time


class RobotControl:
    """variables"""
    settings = None    # configuration serial port
    mapx, mapy = None, None    # parameters of frame after deleting distorsion
    resolution = []    # resolution of frame
    data_from_ir = []  # data from ir sensors
    data_from_us = []    # data from us sensors
    data_from_rfid = None    # data from rfid reader
    data_from_battery = None    # power of battery
    msg_servo = 'SS0800000000000E'

    """messages"""
    MSG_TURN_ON_LIGHT = 'SU++10000000000E'    # message to turn on flashlight
    MSG_STOP_DRIVING = 'ST0+00000+00000E'    # message to stop robot
    MSG_TURN_OFF_LIGHT = 'SU--00000000000E'    # message to turn off flashlight
    direction_of_movements_robot = {
                                    'LEFT': 'ST0-00300+00300E', 'RIGHT': 'ST0+00300-00300E',
                                    'FORWARD': 'ST0+00200+00200E', 'STOP': 'ST0+00000+00000E',
                                    'BACK': 'ST0-00200-00200E'
                                   }
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
        self.UV_last_value = None
        self.data_from_ir = []

    def configure_serial_port(self):
        """
        set port and rate of transmitting data
        :return: settings
        """
        settings = serial.Serial('/dev/ttyUSB0')
        settings.baudrate = 115200
        return settings

    def resize_frame_and_delete_distortion(self, percent):
        """
        resize frame and delete distorsion
        :return: mapx, mapy, resolution
        """
        percent = int(percent)
        w = int(640 * percent / 100)
        h = int(480 * percent / 100)
        resolution = [w, h]
        # Parameters for undistort frame from lower camera
        undistorsion_parameters = [
                                    np.array([[669.53947377, 0., 316.57731188],
                                             [0., 650.21491053, 291.96812174], [0., 0., 1.]]),
                                    np.array([-0.4077693, 0.29706739, -0.00737684, -0.00562703, -0.29700514])
                                  ]
        cameramatrix, roi = cv2.getOptimalNewCameraMatrix(undistorsion_parameters[0], undistorsion_parameters[1],
                                                          (resolution[0], resolution[1]), 0)
        mapx, mapy = cv2.initUndistortRectifyMap(undistorsion_parameters[0], undistorsion_parameters[1], None,
                                                 cameramatrix, (resolution[0], resolution[1]), 5)
        return mapx, mapy, resolution

    def encode_and_send_msg(self, msg):
        """
        encode and send message from computer to module of control
        :param msg: str
        :return:
        """
        msg = msg.encode('utf-8')
        self.settings.write(msg)

    def read_and_decode(self):
        """
        read and decode data from module of control
        :return:
        """
        try:
            data_from_serial = self.settings.readline()
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
        try:
            moving = moving.upper()
            msg = self.direction_of_movements_robot.get(moving)
            self.encode_and_send_msg(msg)
        except KeyError:
            print('Check the spelling of the word:', moving)
            self.encode_and_send_msg(self.MSG_STOP_DRIVING)
            sys.exit()

    def stop_platform(self):
        """
        stop movement of robot
        :return:
        """
        self.encode_and_send_msg(self.MSG_STOP_DRIVING)

    def turn_on_flashlight(self):
        """
        turn on flashlight
        :return:
        """
        self.encode_and_send_msg(self.msg_turn_on_fl)

    def turn_off_light(self):
        """
        turn off flashlight
        :return:
        """
        msg = self.MSG_TURN_OFF_LIGHT
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

    # def set_angle_servo(self):
    #     """
    #     set angle of servo
    #     :return:
    #     """
    #     # if len(str(grad)) < 3:
    #     #     grad = (3-len(str(grad))) * '0' + str(grad)
    #     # msg = 'SS'+str(grad)+'0000000000E'
    #     # self.encode_and_send_msg(msg)
    #
    #     self.encode_and_send_msg(self.msg_servo)

    set_angle_servo = lambda angle_servo: ['ss' + str(angle_servo) + '0000000000E']

    def camera_reading_lower(self):
        """
        read and undistort frame from the lower camera
        :return: frame
        """
        ret, frame = self.cap_lower.read()
        frame = cv2.resize(frame, self.resolution, interpolation=cv2.INTER_AREA)  # уменьшаем разрешение фото
        frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)  # избавляемся от дисторсии
        return frame

    def camera_reading_upper(self):
        """
        read frame from upper camera
        :return: frame
        """
        ret, frame = self.cap_upper.read()
        frame = cv2.resize(frame, self.resolution, interpolation=cv2.INTER_AREA)  # уменьшаем разрешение фото
        return frame

    def ir_read(self):
        """
        get data from ir sensors
        :return: self.IR
        """
        try:
            us_ir = self.read_and_decode()
            if us_ir[:2] == 'SI':
                self.data_from_ir = us_ir    # CHECK! + print()
        except Exception:
            pass
        finally:
            return self.data_from_ir

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
                        self.data_from_us = self.data_5_US
                        self.data_5_US = []
        except Exception:
            pass
        finally:
            return self.data_from_us

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

    def accum_read(self):
        """
        get power of battery
        :return:
        """
        try:
            pwr = self.read_and_decode()
            if pwr[:2] == 'SA':
                self.data_from_battery = pwr[2:5]
        except Exception:
            pass
        finally:
            return self.data_from_battery

    def rfid_read(self):
        """
        get data from rfid reader
        :return:
        """
        try:
            data = self.read_and_decode()
            if data[:2] == 'SF':
                self.data_from_rfid = data[2:17]
                print(self.data_from_rfid)
        except Exception:
            pass

