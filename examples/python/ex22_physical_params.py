"""ex22 - change the mass mid-flight and compare climb rates."""

from vrsdk import RobotType, VirtualRobot

COLLECTIVE_US = 1800.0  # high enough that both masses still climb
HZ = 25

robot = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = the scene's multirotor
robot.connect()


def climb_run():
    robot.reset()  # same start for both runs; configuration survives a state reset
    for _ in range(25):
        robot.set_mr_pwm(COLLECTIVE_US, COLLECTIVE_US, COLLECTIVE_US, COLLECTIVE_US)
        robot.rate(HZ)
    start = robot.states
    for _ in range(50):
        robot.set_mr_pwm(COLLECTIVE_US, COLLECTIVE_US, COLLECTIVE_US, COLLECTIVE_US)
        robot.rate(HZ)
    end = robot.states
    # "frd": z counts DOWN, so a climb is a decrease
    return (start.kin.lin_pos[2] - end.kin.lin_pos[2]) / (end.elapsed - start.elapsed)


# Mass and inertia are not in the state message: the changed climb rate IS the receipt.
robot.set_physical_params(mass=1.0, moi=(0.02, 0.02, 0.04))
light = climb_run()

robot.set_physical_params(mass=2.0)
heavy = climb_run()

print(f"{COLLECTIVE_US} us on every rotor, twice:")
print(f"  1.0 kg -> {light:+6.2f} m/s")
print(f"  2.0 kg -> {heavy:+6.2f} m/s")
