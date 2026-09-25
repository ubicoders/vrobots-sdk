"""ex21 - reset the robot to its home pose."""

import math

from vrsdk import RobotType, VirtualRobot

CLIMB_US = 1700.0
HZ = 25

robot = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = the scene's multirotor
robot.connect()

# Home is the pose captured at the robot's first physics step, not where you found
# it: reset once, settle, and read it off the state stream.
robot.reset()
for _ in range(50):
    robot.rate(HZ)
home = robot.states
print(f"home pos={home.kin.lin_pos}")

# Leave home under power.
for _ in range(75):
    robot.set_mr_pwm(CLIMB_US, CLIMB_US, CLIMB_US, CLIMB_US)
    robot.rate(HZ)

# Nothing sent: the last command latches and keeps flying it.
for _ in range(25):
    robot.rate(HZ)

before = robot.states
robot.reset()  # a bare GET; the teleport lands in phase 0 of the next physics step
for _ in range(50):
    robot.rate(HZ)
after = robot.states

d_before = math.dist(before.kin.lin_pos, home.kin.lin_pos)
d_after = math.dist(after.kin.lin_pos, home.kin.lin_pos)
print(f"position:  {d_before:.2f} m from home before, {d_after:.2f} m after")
# reset re-latches the robot's INITIAL command (1100 us idle), nothing was sent
print(f"actuators: echo {before.actuator.pwm} -> {after.actuator.pwm}")
# seq and elapsed never reset; only the robot moves
print(f"time:      seq {before.seq} -> {after.seq}, elapsed {before.elapsed:.2f}s -> {after.elapsed:.2f}s")
