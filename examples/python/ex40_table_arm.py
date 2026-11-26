"""ex40 - table_arm: six joints, a two-finger gripper in metres, and what the fingers report.

    python examples/python/ex40_table_arm.py        # the table-arm scene's UR10, sys_id 0
    python examples/python/ex40_table_arm.py <id>   # one at another id

A UR10 with a Robotiq 2F-85 gripper, at the back of the table-arm scene's
sorting cell, pinned at `sys_id` 0. The base never moves.

Joints
------

`SET_ARM_JOINTS` takes six setpoints, A1 to A6 -- base, shoulder, elbow and
three wrists -- in **radians**, each within -360 to 360 deg, reached at
120 deg/s (A1-A2) or 180 deg/s (A3-A6). It is the rail arm's command (ex39)
with one joint fewer, and the same rules: a latch, every joint clamped and
driven independently, every step tested against the scene first. This arm
also refuses to drive into **itself**. Home is whatever pose the arm loaded
in, and nothing on the wire publishes it -- so read the joints once after
connecting, which is also how this example knows where to come back to.

The gripper is in metres
------------------------

`SET_ARM_GRIPPER` takes the gap between the pads in **metres**, clamped to
[0, 0.085] (`table_arm.GRIPPER_STROKE_M`, the 2F-85's stroke). The clamp
is silent, so a millimetre figure such as `40.0` opens the gripper fully.
The fingers move at 0.15 m/s of opening, the whole stroke in under 0.6 s.

Closing on an object stops the pads at its width, and the object is **held**
when both pads press it from inside and it is a loose object within the
5 kg payload. Closed on thin air, as here, the pads simply arrive and nothing
is held.

The three entries after the joints
----------------------------------

| index | entry | `normalized` | `measured` |
|---|---|---|---|
| 6 | GRIPPER | the latch, as a fraction of the stroke | the gap between the pads now, m |
| 7 | CONTACT | 0 | 0 none, 1 tool contact, 2 collision stop |
| 8 | HOLD | 1 holding, 0 not | the held mass, kg |

Nine entries, like the rail arm's, and laid out differently after the
joints: `table_arm_actuators()` decodes them by robot type, never by count.
A finger stopping on an object is **not** a contact -- that is how the
gripper feels it -- so CONTACT speaks only for the arm's joint steps.

Leaving it as found
-------------------

The joints and the gripper latch are read at the start and put back at the
end. An arm found holding something is left alone: opening would drop it.
Scene-authored: attached by `sys_id`, never deleted.
"""

import math
import sys

from vrsdk import ArmContact, RobotType, VirtualRobot, VrError, table_arm

USAGE = """\
The table-arm scene pins its UR10 at sys_id 0, which is the default. For one at
another id, pass it:

    python examples/python/ex40_table_arm.py 0

The scene has to be loaded and in Play mode; its scene card needs the Pro tier.
List what is publishing with:

    vrobots topic list"""

SYS_ID = 0  # the table-arm scene's UR10
A1_TURN_DEG = 15.0  # swing the base
A6_TURN_DEG = 45.0  # and turn the gripper about its own axis
GRIP_M = 0.040  # METRES, so 40 mm. A bare 40.0 would open fully
HZ = 25.0
MOVE_SAMPLES = 125  # 5 s; this move takes well under 1
GRIP_SAMPLES = 125  # 5 s: the whole stroke takes 0.6, or 4.3 at srv/arm's slowest
TOLERANCE = 1e-3  # rad
GRIP_TOLERANCE_M = 0.0005  # half a millimetre


def attach(sys_id):
    """Attach to `sys_id`, and make sure it really is the table arm.

    `connect()` takes the robot type on trust, and the rail arm reports nine
    actuator entries too. `z/properties` names each one; once this handle has
    read it (ex44), `table_arm_actuators()` refuses a robot whose layout is not
    six servos, GRIPPER, CONTACT and HOLD.
    """
    robot = VirtualRobot(RobotType.TABLEARM, sys_id=sys_id)
    robot.connect()
    robot.properties()
    robot.table_arm_actuators()
    return robot


def move_to(robot, goal):
    """Latch six setpoints and watch the measured joints converge -- or stop short,
    with CONTACT saying why.
    """
    t0 = robot.states.elapsed
    for i in range(MOVE_SAMPLES):
        # A latch: once would do, and every iteration heals a lost command.
        robot.set_arm_joints(goal)
        robot.rate(HZ)
        arm = robot.table_arm_actuators()
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
    arm = robot.table_arm_actuators()
    print(
        f"   not there after {MOVE_SAMPLES / HZ:.0f} s. joints_normalized "
        f"{list(arm.joints_normalized)}: a joint at -1 or 1 is clamped at its limit"
    )


def gripper(robot, opening_m):
    """Command an opening, metres, and read GRIPPER, HOLD and CONTACT once the pads
    have arrived -- or once they have closed on something.
    """
    print(f"\n-- gripper to {opening_m * 1e3:.1f} mm --")
    t0 = robot.states.elapsed
    for _ in range(GRIP_SAMPLES):
        robot.set_arm_gripper(opening_m)
        robot.rate(HZ)
        arm = robot.table_arm_actuators()
        if arm.holding or abs(arm.gripper_opening_m - opening_m) < GRIP_TOLERANCE_M:
            print(f"   t={robot.states.elapsed - t0:5.2f}s  {tool(arm)}")
            return
    # Stopped short with nothing held: something is between the pads that the
    # gripper will not hold -- fixed scenery, or over the payload.
    print(f"   stopped short, holding nothing: {tool(robot.table_arm_actuators())}")


def tool(arm):
    """The three entries after the joints, decoded."""
    return (
        f"GRIPPER [{table_arm.GRIPPER}]: latch {arm.gripper_command_m * 1e3:.1f} mm, "
        f"pads {arm.gripper_opening_m * 1e3:.1f} mm apart   "
        f"HOLD [{table_arm.HOLD}]: holding={arm.holding}, {arm.held_mass_kg:.2f} kg   "
        f"CONTACT [{table_arm.CONTACT}]: {arm.contact_name} (code {arm.contact})"
    )


def largest_error(now, goal):
    """The largest gap between where the joints are and where they were sent."""
    worst = 0.0
    for a, b in zip(now, goal):
        worst = max(worst, abs(a - b))
    return worst


def joints(j):
    """A1 to A6, in degrees."""
    return (
        f"A1 {math.degrees(j[table_arm.A1]):+7.1f}  A2 {math.degrees(j[table_arm.A2]):+7.1f}  "
        f"A3 {math.degrees(j[table_arm.A3]):+7.1f}  A4 {math.degrees(j[table_arm.A4]):+7.1f}  "
        f"A5 {math.degrees(j[table_arm.A5]):+7.1f}  A6 {math.degrees(j[table_arm.A6]):+7.1f} deg"
    )


def deg(degrees):
    """Degrees to the radians the command takes."""
    return math.radians(degrees)


def sys_id_from_args():
    """The table arm's sys_id: the one command-line argument, or `SYS_ID`."""
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
    print(f"no table arm at sys_id {sys_id}: [{e.code}] {e}\n\n{USAGE}", file=sys.stderr)
    sys.exit(1)
s = robot.states
print(f"attached to sys_id={robot.sys_id} '{s.name}'")

# ===== as found =====
# There is no home pose on the wire: these joints are the only record of it.
found = robot.table_arm_actuators()
print(f"found: {joints(found.joints)}")
print(f"       {tool(found)}")
if found.holding:
    print(
        f"the gripper is holding {found.held_mass_kg:.2f} kg, and this example opens it: "
        "nothing was sent. Let go of it first, or reset() the arm."
    )
    sys.exit(0)

# ===== joints =====
goal = list(found.joints)
goal[table_arm.A1] += deg(A1_TURN_DEG)
goal[table_arm.A6] += deg(A6_TURN_DEG)
print(f"\n-- A1 {A1_TURN_DEG:+g} deg, A6 {A6_TURN_DEG:+g} deg: to {joints(goal)} --")
move_to(robot, goal)

# ===== close the gripper to a width, then open it =====
gripper(robot, GRIP_M)
gripper(robot, found.gripper_command_m)

# ===== hand it back =====
print("\n-- back to where it was found --")
move_to(robot, found.joints)
print("joints and gripper back. Scene-authored: left as it was found, never deleted.")
