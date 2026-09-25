"""ex08 - send_cmd, the generic escape hatch for the whole command space."""

from vrsdk import RobotType, VirtualRobot, cmd

STEER_US = 1500  # centre
THROTTLE_US = 1600  # light forward
BRAKE_US = 1100  # released

car = VirtualRobot(RobotType.TRUCK, sys_id=0)  # sys_id 0 = truck
car.connect()

while True:
    # cmd_id decides which payload fields mean anything; unset fields stay off the wire.
    car.send_cmd(cmd.SET_CAR, int_arr=[STEER_US, THROTTLE_US, BRAKE_US])

    # An id nothing acts on yet, with no typed wrapper: published, silently ignored.
    car.send_cmd(cmd.ADD_BODY_FORCE, vec3=(0.0, 0.0, 25.0))

    s = car.states
    fx, fy, fz = s.wrench.force
    print(
        f"sent {cmd.name(cmd.SET_CAR)}({cmd.SET_CAR}) + "
        f"{cmd.name(cmd.ADD_BODY_FORCE)}({cmd.ADD_BODY_FORCE}) -> echo={s.actuator.pwm} "
        f"wrench=({fx:+.2f},{fy:+.2f},{fz:+.2f}) N"
    )

    car.rate(5)
