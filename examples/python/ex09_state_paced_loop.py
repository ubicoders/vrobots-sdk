"""ex09 - wait_new_state: run the loop once per published sample."""

from vrsdk import RobotType, VirtualRobot

TIMEOUT = 0.2  # seconds; 5x the 25 Hz state period

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor
mr.connect()

last_seq = 0
last_t_ns = 0
while True:
    mr.wait_new_state(TIMEOUT)  # blocks until a snapshot newer than the current one

    s = mr.states
    dt_ms = float("nan") if last_t_ns == 0 else (s.t_ns - last_t_ns) / 1e6
    skipped = max(0, s.seq - (last_seq + 1))  # seq jumps reveal dropped samples
    last_seq, last_t_ns = s.seq, s.t_ns

    x, y, z = s.kin.lin_pos
    note = f"  <- {skipped} sample(s) skipped" if skipped else ""
    print(f"seq={s.seq} dt={dt_ms:6.1f} ms pos=({x:.3f},{y:.2f},{z:.2f}){note}")
