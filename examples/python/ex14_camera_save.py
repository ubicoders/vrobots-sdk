"""ex14 - grab one frame, write it to disk, exit."""

import cv2
import numpy as np

from vrsdk import RobotType, VirtualRobot

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()
cam = mr.open_camera("front_left", "720p", "rgba8")  # rgba8: 4 channels, not rgb8
print(f"attached to {cam.service_name}")

cam.wait_new_frame(2.0)
frame = cam.read()
assert frame is not None, "wait_new_frame returned, so one is waiting"

print(
    f"frame seq={frame.seq} t={frame.elapsed:.3f}s {frame.width}x{frame.height} "
    f"{frame.format} ({frame.channels} B/px, step={frame.step}, "
    f"{len(frame.data)} bytes)"
)
i = frame.intrinsics
print(
    f"intrinsics fx={i.fx:.1f} fy={i.fy:.1f} cx={i.cx:.1f} cy={i.cy:.1f} "
    f"fov_y={np.degrees(i.fov_y):.1f} deg  clip {i.near_clip:.2f}..{i.far_clip:.0f} m"
)

img = frame.image  # numpy (h, w, 4) uint8, top-down, RGBA
print(f"numpy {img.shape} {img.dtype}")
cv2.imwrite("frame.png", cv2.cvtColor(img, cv2.COLOR_RGBA2BGR))  # OpenCV wants BGR
print("wrote frame.png")
