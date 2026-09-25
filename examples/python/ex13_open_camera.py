"""ex13 - attach to an existing camera stream, changing nothing in the sim."""

from vrsdk import RobotType, VirtualRobot

FRAMES = 60
TIMEOUT = 0.5  # seconds to wait for each frame

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()

# rgba8: 4 channels, not rgb8; name/resolution/format are the stream identity
cam = mr.open_camera("front_left", "720p", "rgba8")
print(f"attached to {cam.service_name} (nothing in the sim changed)")
spec = cam.spec
print(f"spec: name={spec.name} resolution={spec.resolution} format={spec.format}")

# frame-paced: wait_new_frame blocks until the next rendered frame
seen = 0
while seen < FRAMES:
    cam.wait_new_frame(TIMEOUT)
    frame = cam.read()  # consumes freshness; None if someone else got it
    if frame is None:
        continue
    seen += 1
    if seen % 10 == 1:
        m = frame.mount
        print(
            f"frame {seen}: seq={frame.seq} {frame.width}x{frame.height} "
            f"{len(frame.data)} bytes, "
            f"mount=({m.position[0]:+.2f},{m.position[1]:+.2f},{m.position[2]:+.2f}) m"
        )

st = cam.stats
print(
    f"read {seen} frame(s): received={st.received} decode_errors={st.decode_errors} "
    f"seq_gaps={st.seq_gaps} missed_frames={st.missed_frames}"
)
