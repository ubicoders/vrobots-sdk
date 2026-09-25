"""ex01 - read the robot's states in a loop."""

from vrsdk import RobotType, VirtualRobot

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()

while True:
    s = mr.states  # immutable latest snapshot, never torn
    x, y, z = s.kin.lin_pos
    print(f"State t={s.elapsed:.3f} pos=({x:.3f},{y:.2f},{z:.2f})")
    mr.rate(50)  # drift-compensated pacing, Hz
