"""ex19 - a simple state/command loop."""

from vrsdk import RobotType, VirtualRobot

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()

samples = 0
while True:
    mr.wait_new_state(0.5)
    samples += 1
    if samples % 25 == 0:  # one status line a second at 25 Hz
        s = mr.states
        st = mr.stats
        x, y, z = s.kin.lin_pos
        print(
            f"seq={s.seq} t={s.elapsed:.2f}s pos=({x:.2f},{y:.2f},{z:.2f}) "
            f"echo={s.actuator.pwm} received={st.received} gaps={st.seq_gaps}"
        )
    mr.set_mr_pwm([1501.0] * 4)
