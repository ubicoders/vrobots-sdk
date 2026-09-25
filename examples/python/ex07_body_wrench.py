"""ex07 - push the robot around with body force and torque commands."""

from vrsdk import RobotType, VirtualRobot

GUST_N = (5.0, 0.0, 0.0)  # newtons, in the frame from the connect options
TWIST_NM = (0.0, 0.0, 0.2)  # newton-metres, same frame

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor
mr.connect()
opts = mr.options
print(f"sending vectors tagged {opts['coord_frame_id']!r} (axes {opts['axis_convention']})")

step = 0
while True:
    # One verb per iteration: force, torque, then both (set_body_ft).
    if step % 3 == 0:
        mr.set_body_force(GUST_N)
        sent = f"set_body_force({GUST_N})"
    elif step % 3 == 1:
        mr.set_body_torque(TWIST_NM)
        sent = f"set_body_torque({TWIST_NM})"
    else:
        mr.set_body_ft(GUST_N, TWIST_NM)
        sent = f"set_body_ft({GUST_N}, {TWIST_NM})"
    step += 1

    s = mr.states
    fx, fy, fz = s.wrench.force
    tx, ty, tz = s.wrench.torque
    print(sent)
    print(
        f"    state.wrench force=({fx:+.2f},{fy:+.2f},{fz:+.2f}) N  "
        f"torque=({tx:+.2f},{ty:+.2f},{tz:+.2f}) N.m  in {s.coord_frame_id!r}"
    )

    mr.rate(2)
