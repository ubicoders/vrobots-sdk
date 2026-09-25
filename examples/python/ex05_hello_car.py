"""ex05 - drive the truck with set_car."""

import math

from vrsdk import RobotType, VirtualRobot

STEER_US = 1400.0  # 1500 = centre
THROTTLE_US = 1650.0  # 1500 = stop, 1900 = full forward
BRAKE_US = 1100.0  # brake is bottom-anchored: 1100 = released, 1900 = full

car = VirtualRobot(RobotType.TRUCK, sys_id=0)  # sys_id 0 = truck, 1 = multirotor
car.connect()

while True:
    s = car.states
    x, y, z = s.kin.lin_pos
    # lin_vel is a body-frame vector; its magnitude is the speed.
    speed = math.dist(s.kin.lin_vel, (0.0, 0.0, 0.0))
    print(
        f"State t={s.elapsed:.3f} pos=({x:.3f},{y:.2f},{z:.2f}) "
        f"speed={speed:.2f} m/s echo={s.actuator.pwm}"
    )

    car.set_car(STEER_US, THROTTLE_US, BRAKE_US)

    car.rate(50)
