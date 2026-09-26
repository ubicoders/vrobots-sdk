"""ex27 - rebuild the multirotor's rotor list with configure_rotors."""

import math
import sys

from vrsdk import RobotType, RotorSpec, VirtualRobot

# optional: attach to the scene multirotor by id (`vrobots topic list`); it KEEPS the mass and
# rotors set below until the scene reloads. With none, one is created and deleted, but a created
# multirotor does not fly in sim v3.0.0 (every climb reads 0.00 m/s), so prefer the id.
sys_id = int(sys.argv[1]) if len(sys.argv) > 1 else None

ARM_M = 0.25  # hub distance from the robot origin
COLLECTIVE_US = 1800.0
THRUST_SCALE = 0.70
HZ = 25

robot = VirtualRobot(RobotType.MULTIROTOR, sys_id=sys_id)  # None: the manager creates one
robot.connect()
print(f"{'created' if sys_id is None else 'attached to'} sys_id={robot.sys_id}")
robot.set_physical_params(mass=1.0)  # pinned, so the runs are comparable

# The rotor count is fixed at spawn; read it, do not assume four.
rotors = len(robot.states.actuator.pwm)
collective = [COLLECTIVE_US] * rotors
print(f"this airframe has {rotors} rotor(s)")


def ring(n):
    # Flat ring in the default "unity" header frame: x-z plane, +y up. Positions
    # are measured from the robot origin, not the centre of mass. spin_dir 0
    # lets the simulator alternate the yaw torque sign by index.
    out = []
    for i in range(n):
        angle = math.pi / 4.0 + i * math.tau / n
        out.append(RotorSpec(position=(ARM_M * math.sin(angle), 0.0, ARM_M * math.cos(angle))))
    return out


def climb(label):
    robot.reset()
    for _ in range(25):
        robot.set_mr_pwm(collective)
        robot.rate(HZ)
    start = robot.states
    for _ in range(75):
        robot.set_mr_pwm(collective)
        robot.rate(HZ)
    end = robot.states
    # "frd": z counts DOWN, so a climb is a decrease
    rate = (start.kin.lin_pos[2] - end.kin.lin_pos[2]) / (end.elapsed - start.elapsed)
    measured = [round(v, 1) for v in end.actuator.measured]
    print(f"  {label:<20} climb={rate:+6.2f} m/s  rotor speed echo={measured}")


climb("as spawned")

# The list is REPLACED, not merged, and must describe every rotor in index order.
# A wrong-length list is dropped whole inside the sim and still acked ok.
reference = RotorSpec()  # the simulator's own reference rotor is the base to build on
weak = [
    RotorSpec(
        position=r.position,
        thrust_a=reference.thrust_a * THRUST_SCALE,
        thrust_b=reference.thrust_b * THRUST_SCALE,
        thrust_c=reference.thrust_c * THRUST_SCALE,
    )
    for r in ring(rotors)
]
robot.configure_rotors(weak)
# The rotor speed echo comes from the ang_vel line, not thrust: it will not change.
climb("70% thrust curve")

if sys_id is None:  # created above, so deleted here; a scene robot keeps the weak rotors
    robot.delete()
    print(f"deleted sys_id={robot.sys_id}")
