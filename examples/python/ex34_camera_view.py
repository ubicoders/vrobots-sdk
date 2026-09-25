"""ex34 - show the robot's front_left camera in an OpenCV window."""

import cv2

from vrsdk import RobotType, VirtualRobot

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()
cam = mr.open_camera("front_left", "720p", "rgba8")  # rgba8: 4 channels, not rgb8

while True:
    frame = cam.read()
    if frame is not None:
        cv2.imshow("vrobots camera", cv2.cvtColor(frame.image, cv2.COLOR_RGBA2BGR))
    if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
        break

cv2.destroyAllWindows()
