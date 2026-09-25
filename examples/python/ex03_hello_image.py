"""ex03 - read camera frames alongside states."""

import cv2

from vrsdk import RobotType, VirtualRobot

FRAMES = 300

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor
mr.connect()
cam = mr.open_camera("front_left", "720p", "rgba8")  # rgba8: 4 channels, not rgb8
print(f"camera stream: {cam.service_name}")

seen = 0
while seen < FRAMES:
    s = mr.states
    if cam.fresh:  # True only when a frame arrived since the last read
        frame = cam.frame  # metadata for the image we are about to read
        img = cam.image  # numpy (h, w, 4) uint8, top-down, RGBA
        seen += 1
        print(
            f"Image {frame.camera_name} t={frame.elapsed:.3f} "
            f"size=({frame.width}x{frame.height}) seq={frame.seq} "
            f"lag_vs_state={(s.t_ns - frame.t_ns) / 1e6:.1f} ms"
        )
        cv2.imshow("vrsdk front_left", cv2.cvtColor(img, cv2.COLOR_RGBA2BGR))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    mr.rate(100)

cv2.destroyAllWindows()
st = cam.stats
print(
    f"{seen} frame(s), received={st.received} "
    f"decode_errors={st.decode_errors} seq_gaps={st.seq_gaps}"
)
