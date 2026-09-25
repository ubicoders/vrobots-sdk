"""ex16 - two camera streams in one loop, each with its own freshness."""

from vrsdk import RobotType, VirtualRobot

FRAMES = 60  # per camera
HZ = 100

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()

# two subscriptions, no mutation: both cameras are already on the robot
left = mr.open_camera("front_left", "720p", "rgba8")
right = mr.open_camera("front_right", "720p", "rgba8")
print(f"left : {left.service_name}")
print(f"right: {right.service_name}")
print(f"mounted by this handle: {mr.mounted_cameras()}  <- neither is ours")

n_left = n_right = 0
last_left_ns = 0

while n_left < FRAMES or n_right < FRAMES:
    f = left.read()
    if f is not None:
        n_left += 1
        last_left_ns = f.t_ns
        if n_left % 20 == 1:
            print(f"L frame {n_left}: seq={f.seq} t={f.elapsed:.3f}")

    f = right.read()
    if f is not None:
        n_right += 1
        if n_right % 20 == 1:
            # the SDK never pairs the streams; relate frames by t_ns yourself
            skew_ms = (
                float("nan") if last_left_ns == 0 else (f.t_ns - last_left_ns) / 1e6
            )
            print(
                f"R frame {n_right}: seq={f.seq} t={f.elapsed:.3f}  "
                f"skew_vs_last_left={skew_ms:+.1f} ms"
            )

    mr.rate(HZ)

ls, rs = left.stats, right.stats
print(
    f"\nleft : {n_left} read, received={ls.received} seq_gaps={ls.seq_gaps} "
    f"missed={ls.missed_frames}"
)
print(
    f"right: {n_right} read, received={rs.received} seq_gaps={rs.seq_gaps} "
    f"missed={rs.missed_frames}"
)
