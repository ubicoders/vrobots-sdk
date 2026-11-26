"""ex39 - rail_arm: seven joints to a pose, the vacuum on and off, and back.

    python examples/python/ex39_rail_arm.py        # the warehouse rail arm, sys_id 2
    python examples/python/ex39_rail_arm.py <id>   # one at another id

A KUKA KR210 riding an 8 m linear rail, with a 400 x 300 mm vacuum pad on its
flange. The warehouse scene pins it at `sys_id` 2. Its base never moves --
the state's kinematics are the base's -- so everything this arm does is in
its actuator report.

One setpoint per joint, SI
--------------------------

`SET_ARM_JOINTS` carries seven numbers in actuator order: the rail in
**metres**, then A1 to A6 in **radians** (`rail_arm.*` names the indices).
The numbers are the KUKA controller's axis values, and at home they are:

| joint | range | top speed | home |
|---|---|---|---|
| rail | 0 to 8 m | 1.2 m/s | 4 m |
| A1, swivel | -185 to 185 deg | 105 deg/s | 0 |
| A2, shoulder | -140 to -5 deg | 94 deg/s | -60 |
| A3, elbow | -120 to 168 deg | 100 deg/s | 90 |
| A4, forearm roll | -350 to 350 deg | 136 deg/s | 0 |
| A5, wrist bend | -122.5 to 122.5 deg | 129 deg/s | 60 |
| A6, flange turn | -350 to 350 deg | 206 deg/s | 0 |

With A4 at 0 the pad faces straight down whenever A2 + A3 + A5 = 90 deg, as at
home; the pose below keeps that sum. The command is a **latch**: every joint
is clamped to its range and driven toward its setpoint at its own top speed,
independently, so they arrive at different times. Nothing replies. The
report is the only proof, and it can end a move three ways:

- **arrived**: every joint within a tolerance of its setpoint. The values
  cross the wire as 32-bit floats both ways, so never compare with `==`.
- **held short**: CONTACT is not `no_contact`. Every step is tested against
  the scene before it is taken, and a step into anything is refused -- the
  protective stop -- with the setpoint still latched.
- **at a limit**: a setpoint past a joint's range is clamped, so that joint
  stops short with `joints_normalized` at -1 or 1.

The two entries after the joints
--------------------------------

| index | entry | `normalized` | `measured` |
|---|---|---|---|
| 7 | VACUUM | the latch, 0 off and 1 on | the held mass, kg; 0 = nothing |
| 8 | CONTACT | 0 | 0 none, 1 tool contact, 2 collision stop |

`SET_ARM_VACUUM` is a latch too. Switched on with nothing under the pad it
grips nothing -- it seals only on a loose object within 1 cm of the pad,
square to it -- so up in the air the proof is the latch echoing back with a
held mass of 0. The table arm reports nine entries too, laid out differently
after the joints: `rail_arm_actuators()` decodes by robot type, never by
count (ex40).

Leaving it as found
-------------------

The joints are read at the start and driven back there at the end, and the
vacuum latch is put back as it was. An arm found holding a load is left
alone: switching the vacuum off would drop it. `reset()` would also put the
arm home -- instantly, with its setpoints -- but home is not necessarily where
it was found. Scene-authored: attached by `sys_id`, never deleted.
"""

import math
import sys

from vrsdk import ArmContact, RobotType, VirtualRobot, VrError, rail_arm

USAGE = """\
The warehouse scene pins its rail arm at sys_id 2, which is the default. For one
at another id, pass it:

    python examples/python/ex39_rail_arm.py 2

The scene has to be loaded and in Play mode, and on the Community tier only the
focused robot is on the wire at all. List what is publishing with:

    vrobots topic list"""

SYS_ID = 2  # the warehouse scene's rail arm
HZ = 25.0
MOVE_SAMPLES = 125  # 5 s; the pose below takes about 0.8
VACUUM_SAMPLES = 10  # 0.4 s for the latch to come back
TOLERANCE = 1e-3  # m on the rail, rad on the others


def attach(sys_id):
    """Attach to `sys_id`, and make sure it really is the rail arm.

    `connect()` takes the robot type on trust, and the table arm reports nine
    actuator entries too. `z/properties` names each one; once this handle has
    read it (ex44), `rail_arm_actuators()` refuses a robot whose layout is not
    the rail's linear joint, six servos, VACUUM and CONTACT.
    """
    robot = VirtualRobot(RobotType.RAILARM, sys_id=sys_id)
    robot.connect()
    robot.properties()
    robot.rail_arm_actuators()
    return robot


def move_to(robot, goal):
    """Latch one setpoint per joint, and watch the measured joints converge.

    The setpoint latches, so once would do; sending it every iteration costs
    nothing and heals a command lost on the way.
    """
    t0 = robot.states.elapsed
    for i in range(MOVE_SAMPLES):
        robot.set_arm_joints(goal)
        robot.rate(HZ)
        arm = robot.rail_arm_actuators()
        t = robot.states.elapsed - t0
        worst = largest_error(arm.joints, goal)
        if worst < TOLERANCE:
            print(f"   t={t:5.2f}s  {joints(arm.joints)}  -- arrived")
            return
        if arm.contact != ArmContact.NO_CONTACT:
            print(
                f"   t={t:5.2f}s  {joints(arm.joints)}  -- held short: CONTACT "
                f"{arm.contact_name} (code {arm.contact})"
            )
            return
        if i % 5 == 0:
            print(f"   t={t:5.2f}s  {joints(arm.joints)}  worst error {worst:.4f}")
    arm = robot.rail_arm_actuators()
    print(
        f"   not there after {MOVE_SAMPLES / HZ:.0f} s. joints_normalized "
        f"{list(arm.joints_normalized)}: a joint at -1 or 1 is clamped at its limit"
    )


def vacuum(robot, on):
    """Switch the vacuum, then read the two entries after the joints."""
    print(f"\n-- vacuum {'on' if on else 'off'} --")
    for _ in range(VACUUM_SAMPLES):
        robot.set_arm_vacuum(on)
        robot.rate(HZ)
    arm = robot.rail_arm_actuators()
    print(
        f"   VACUUM  [{rail_arm.VACUUM}]: on={arm.vacuum_on}, holding={arm.holding}, "
        f"held {arm.held_mass_kg:.2f} kg"
    )
    print(f"   CONTACT [{rail_arm.CONTACT}]: {arm.contact_name} (code {arm.contact})")


def largest_error(now, goal):
    """The largest gap between where the joints are and where they were sent."""
    worst = 0.0
    for a, b in zip(now, goal):
        worst = max(worst, abs(a - b))
    return worst


def joints(j):
    """The rail in metres, A1 to A6 in degrees."""
    return (
        f"rail {j[rail_arm.RAIL]:.3f} m  A1 {math.degrees(j[rail_arm.A1]):+6.1f}  "
        f"A2 {math.degrees(j[rail_arm.A2]):+6.1f}  A3 {math.degrees(j[rail_arm.A3]):+6.1f}  "
        f"A4 {math.degrees(j[rail_arm.A4]):+6.1f}  A5 {math.degrees(j[rail_arm.A5]):+6.1f}  "
        f"A6 {math.degrees(j[rail_arm.A6]):+6.1f} deg"
    )


def deg(degrees):
    """Degrees to the radians the command takes."""
    return math.radians(degrees)


def sys_id_from_args():
    """The rail arm's sys_id: the one command-line argument, or `SYS_ID`."""
    if len(sys.argv) < 2:
        return SYS_ID
    arg = sys.argv[1]
    if arg.isdecimal() and int(arg) < 2**32:  # a u32, as the wire carries it
        return int(arg)
    print(f"usage: {sys.argv[0]} [sys_id]\n\n{USAGE}", file=sys.stderr)
    sys.exit(2)


# ===== setup =====
sys_id = sys_id_from_args()
try:
    robot = attach(sys_id)
except VrError as e:
    print(f"no rail arm at sys_id {sys_id}: [{e.code}] {e}\n\n{USAGE}", file=sys.stderr)
    sys.exit(1)
s = robot.states
print(f"attached to sys_id={robot.sys_id} '{s.name}'")

# ===== as found =====
found = robot.rail_arm_actuators()
print(f"found: {joints(found.joints)}")
print(
    f"       vacuum on={found.vacuum_on}, holding={found.holding} "
    f"({found.held_mass_kg:.2f} kg), CONTACT {found.contact_name}"
)
if found.holding:
    print(
        f"the pad is holding {found.held_mass_kg:.2f} kg, and this example switches the "
        "vacuum off: nothing was sent. Release it first, or reset() the arm."
    )
    sys.exit(0)

# ===== to a pose =====
# Home is (4 m, 0, -60, 90, 0, 60, 0). This pose slides the carriage a metre,
# swings A1 30 deg and takes the shoulder 10 deg back, and A5 follows so that
# A2 + A3 + A5 stays 90 deg: the pad keeps facing straight down.
pose = [5.0, deg(30.0), deg(-70.0), deg(90.0), 0.0, deg(70.0), 0.0]
print(f"\n-- to {joints(pose)} --")
move_to(robot, pose)

# ===== the vacuum, on and off =====
vacuum(robot, True)
vacuum(robot, False)

# ===== hand it back =====
print("\n-- back to where it was found --")
move_to(robot, found.joints)
robot.set_arm_vacuum(found.vacuum_on)
print(
    f"joints back, vacuum on={found.vacuum_on}. Scene-authored: left as it was found, "
    "never deleted."
)
