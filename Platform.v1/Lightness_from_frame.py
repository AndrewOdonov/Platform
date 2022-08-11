from Robot_test_library_functions import RobotControl
import cv2
RC = RobotControl()

while True:
    frame = RC.camera_reading_lower()
    lighting = RC.analysis_lighting(frame, 100)
    print(lighting)
    cv2.imshow('frame_Lower', frame)

    frame2 = RC.camera_reading_upper()
    cv2.imshow('frame_Upper', frame2)

    if cv2.waitKey(1) == ord('q'):
        break