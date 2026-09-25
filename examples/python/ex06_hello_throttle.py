"""ex06 - set_mr_throttle: normalised per-rotor throttle, watched via the echo."""

from vrsdk import RobotType, VirtualRobot

THROTTLE = [0.6, 0.6, 0.6, 0.6]  # normalised 0..1, one per rotor

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor
mr.connect()

while True:
    # On the wire but no robot type acts on it yet; the state echo is the only evidence.
    mr.set_mr_throttle(THROTTLE)

    s = mr.states
    down = s.kin.lin_pos[2]  # "frd" frame: lin_pos[2] is down, altitude is its negation
    norm = [round(v, 3) for v in s.actuator.normalized]
    meas = [round(v, 3) for v in s.actuator.measured]
    print(
        f"sent {THROTTLE} -> alt={-down:.2f} m  pwm={s.actuator.pwm} "
        f"normalized={norm} measured={meas}"
    )

    mr.rate(25)
