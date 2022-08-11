import cv2
from Robot_test_library_functions import RobotControl
RC = RobotControl()

while True:
    frame_lower = RC.camera_reading_lower()
    frame_upper = RC.camera_reading_upper()
    cv2.imshow('lower_camera', frame_lower)
    cv2.imshow('upper_camera', frame_upper)
    if cv2.waitKey(1) == ord('q'):
        break