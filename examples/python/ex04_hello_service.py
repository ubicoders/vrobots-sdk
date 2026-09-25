"""ex04 - create a new robot in the sim, then delete it."""

import vrsdk
from vrsdk import RobotType, VirtualRobot

# No sys_id: the manager creates a robot and assigns one.
robot = VirtualRobot(RobotType.MULTIROTOR)
robot.connect()
sys_id = robot.sys_id
print(f"created sys_id = {sys_id}")

s = robot.states
print(f"first state: t={s.elapsed:.3f} seq={s.seq} name={s.name!r}")
print(f"its state topic: {vrsdk.topics(sys_id)['state']}")

# Robots outlive the process; only delete() removes one from the scene.
robot.delete()
print(f"deleted sys_id = {sys_id} (is_deleted={robot.is_deleted})")
