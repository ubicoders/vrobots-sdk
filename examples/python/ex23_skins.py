"""ex23 - cycle the truck through its skin catalog while driving."""

import math

from vrsdk import RobotType, VirtualRobot

TRUCK_SKINS = ("black", "blue", "camouflage", "gray", "red")
STEER_US = 1500.0  # straight ahead
THROTTLE_US = 1600.0  # slow forward, so the wheels are always turning
BRAKE_US = 1100.0  # released
HZ = 25

robot = VirtualRobot(RobotType.TRUCK)
robot.connect()
print(f"created sys_id={robot.sys_id}")

for skin in TRUCK_SKINS:
    robot.set_skin(skin)
    print(f"set_skin({skin!r})")
    for i in range(30):
        robot.set_car(STEER_US, THROTTLE_US, BRAKE_US)
        if i % 15 == 0:
            s = robot.states
            speed = math.dist(s.kin.lin_vel, (0.0, 0.0, 0.0))
            # measured[0:4] = wheel speeds FL, FR, RL, RR (rad/s); a skin swap
            # rebinds the wheel colliders, so they must keep turning
            wheels = [round(v, 3) for v in s.actuator.measured[:4]]
            print(f"  t={s.elapsed:6.2f}s speed={speed:5.2f} m/s wheels={wheels}")
        robot.rate(HZ)

# A key outside this type's catalog is acked ok and silently dropped by the sim.
robot.set_skin("gold")

robot.delete()
print(f"deleted sys_id={robot.sys_id}")
