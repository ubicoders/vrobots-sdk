"""ex41 - hello_turret: find it, turn it by rate and by target, fire the laser, narrow its limits.

    python examples/python/ex41_hello_turret.py        # find the turret on the wire
    python examples/python/ex41_hello_turret.py <id>   # or name it

A fixed pan-tilt turret: a pylon turns (**heading**, positive to the right
seen from above) and a head tilts (**pitch**, positive raises the barrel), with
a laser along the barrel and a stereo camera pair on the head. It reports in
CV, and its attitude **is** its joints -- but joint angles carry no axes, so
every number here means the same whatever the frame.

Finding it
----------

The sandbox scene authors a turret without pinning its id, so with no
argument this example looks for it, in two steps:

1. **Who is on the wire.** ex11's `list_topics`: every robot publishes a
   state topic, so the `vrobots/{id}/z/state` keys are the robots.
2. **Which one is a turret.** Nothing in a state message names the type, and
   the number of actuator entries does not either -- a cart-pole reports
   three as well. `z/properties` does: the turret is the robot whose layout
   is `[servo, servo, laser]`. `read_properties` reads a candidate's with no
   handle to it, so nothing is attached until the turret is found, and
   reading changes nothing.

Two motion commands, one latch
------------------------------

| command | carries | the turret |
|---|---|---|
| `set_turret_rates` | rad/s | turns at that rate, clamped to the max, until a limit |
| `set_turret_joints` | rad | turns to that target at the max rate, then holds |

Both latch, and they share one latch: whichever arrived last is in charge. A
rate is the command for closing an aiming loop yourself; a target is the one
for "go there". Stop sending rates and the pylon keeps turning, all the way
to its limit -- which is why the rate phase below ends on a rate of zero.

The laser
---------

A latch of its own. While on, the simulator measures the distance from the
muzzle to the first thing the beam hits, up to 200 m, every physics step, and
that one number is what reaches the wire: `laser_range`, 0 when the laser is
off **or** nothing is within range. The barrel is aimed 5 deg down below so
that on open, flat ground the beam meets the floor about 24 m out.

`srv/turret`, and why the target survives it
--------------------------------------------

`configure_turret` sets the joint limits and rates (also the laser's range
and the stereo pair). Limits that exclude the current pose do not snap it:
the joint walks back to the nearest limit at its max rate. The **target stays
latched** -- the limit only clamps it -- so when the shipped limits come back
the turret walks out to its old target again. Nothing reads the limits back,
so "restore" means sending the shipped values; a scene that had tuned them
differently gets the shipped ones.

Leaving it as found
-------------------

The joints and the laser latch are read at the start and put back at the end,
the limits are restored, and the last motion command is a target -- the
found pose -- which the turret holds. Scene-authored: never deleted.
"""

import math
import sys

import vrsdk
from vrsdk import RobotType, VirtualRobot, VrError, list_topics, read_properties, turret_actuator

USAGE = """\
With no argument this example finds the turret: the sandbox scene authors one
but does not pin its id. To skip the search, pass the id:

    python examples/python/ex41_hello_turret.py 3

The scene has to be loaded and in Play mode, and on the Community tier only the
focused robot is on the wire at all. List what is publishing with:

    vrobots topic list"""

WINDOW = 1.5  # ex11's listening window, s
HZ = 25.0
RATE = 0.3  # rad/s of heading, to the right: ~17 deg/s
RATE_SAMPLES = 38  # ~1.5 s of it
AIM_HEADING_DEG = -30.0  # left of rest
AIM_PITCH_DEG = -5.0  # barrel a little down, so the laser meets the floor
MOVE_SAMPLES = 100  # 4 s; any move within the limits takes under 3
LASER_SAMPLES = 10  # 0.4 s: measured every physics step, on from the next
WATCH_SAMPLES = 25  # 1 s to watch a limit change land
TOLERANCE = 1e-3  # rad
NARROW_DEG = 20.0  # heading limits narrowed to +/- this
SHIPPED_HEADING_DEG = 90.0  # the shipped heading limits, +/- this


def find_turret():
    """Find the turret: list the robots on the wire, then ask each what it is.

    Returns the attached handle, which has already read the layout -- so its
    `turret_actuators()` is checked against it from here on -- or `None` when no
    robot on the wire is a turret.
    """
    # Step 1, ex11's: a zenoh topic is listed only if it published during the
    # window, and a robot's state publishes at 25 Hz.
    print(f"listening for {WINDOW:g}s for the robots on the wire ...")
    topics = list_topics(WINDOW)
    candidates = []
    for t in topics:
        if t.sys_id is None:
            continue
        if t.key == vrsdk.topics(t.sys_id)["state"] and t.sys_id not in candidates:
            candidates.append(t.sys_id)
    print(f"robots on the wire: {candidates}")

    # Step 2: the layout on z/properties is what a robot says it is, and
    # reading it takes no handle.
    for sys_id in candidates:
        try:
            p = read_properties(sys_id)
        except VrError as e:
            print(f"   sys_id {sys_id}: no layout: [{e.code}] {e}")
            continue
        kinds = p.actuator_type_names
        if p.actuator_types == turret_actuator.KINDS:
            print(f"   sys_id {sys_id}: {kinds} -- the turret")
            return attach(sys_id)
        print(f"   sys_id {sys_id}: {kinds} -- not a turret")
    return None


def attach(sys_id):
    """Attach to `sys_id`, and make sure it really is a turret.

    `connect()` takes the robot type on trust; once this handle has read
    `z/properties` (ex44), `turret_actuators()` refuses a robot whose layout is
    not `[servo, servo, laser]`.
    """
    robot = VirtualRobot(RobotType.TURRET, sys_id=sys_id)
    robot.connect()
    robot.properties()
    robot.turret_actuators()
    return robot


def aim(robot, heading, pitch):
    """Latch a joint target and watch both joints arrive.

    The target latches, so once would do; sending it every iteration costs
    nothing and heals a command lost on the way.
    """
    print(
        f"\n-- aim: heading {math.degrees(heading):+.1f} deg, "
        f"pitch {math.degrees(pitch):+.1f} deg --"
    )
    t0 = robot.states.elapsed
    for i in range(MOVE_SAMPLES):
        robot.set_turret_joints(heading, pitch)
        robot.rate(HZ)
        now = robot.turret_actuators()
        t = robot.states.elapsed - t0
        if abs(now.heading - heading) < TOLERANCE and abs(now.pitch - pitch) < TOLERANCE:
            print(f"   t={t:5.2f}s  {report(now)}  -- there")
            return
        if i % 5 == 0:
            print(f"   t={t:5.2f}s  {report(now)}")
    print(
        "   not there: a target past a limit is clamped to it (srv/turret, or another client's "
        "limits)"
    )


def laser(robot, on):
    """Switch the laser and read the range it measures."""
    print(f"\n-- laser {'on' if on else 'off'} --")
    for _ in range(LASER_SAMPLES):
        robot.set_turret_laser(on)
        robot.rate(HZ)
    now = robot.turret_actuators()
    print(
        f"   LASER [{turret_actuator.LASER}]: on={now.laser_on}, range {now.laser_range:.2f} m  "
        "(0 = off, or nothing within range)"
    )


def watch(robot):
    """Send nothing for a second and watch the head: whatever moves it now is the
    target already latched, clamped by whatever limits the service just set.
    """
    for i in range(WATCH_SAMPLES):
        robot.rate(HZ)
        if i % 5 == 0:
            print(f"   {report(robot.turret_actuators())}")


def report(t):
    """The decoded report, in degrees."""
    return (
        f"heading {math.degrees(t.heading):+6.1f} deg "
        f"({t.heading_rate_fraction * 100.0:+4.0f}% of max rate)  "
        f"pitch {math.degrees(t.pitch):+5.1f} deg ({t.pitch_rate_fraction * 100.0:+4.0f}%)  "
        f"laser {'on ' if t.laser_on else 'off'} {t.laser_range:.2f} m"
    )


def deg(degrees):
    """Degrees to the radians the commands take."""
    return math.radians(degrees)


def sys_id_from_args():
    """The turret's sys_id if one was given; `None` means find it."""
    if len(sys.argv) < 2:
        return None
    arg = sys.argv[1]
    if arg.isdecimal() and int(arg) < 2**32:  # a u32, as the wire carries it
        return int(arg)
    print(f"usage: {sys.argv[0]} [sys_id]\n\n{USAGE}", file=sys.stderr)
    sys.exit(2)


# ===== setup: find the turret, or attach to the one named =====
sys_id = sys_id_from_args()
if sys_id is not None:
    try:
        robot = attach(sys_id)
    except VrError as e:
        print(f"no turret at sys_id {sys_id}: [{e.code}] {e}\n\n{USAGE}", file=sys.stderr)
        sys.exit(1)
else:
    try:
        robot = find_turret()
    except VrError as e:
        print(f"could not look: [{e.code}] {e}\n\n{USAGE}", file=sys.stderr)
        sys.exit(1)
    if robot is None:
        print(f"no robot on the wire has a turret's layout.\n\n{USAGE}", file=sys.stderr)
        sys.exit(1)
s = robot.states
print(f"attached to sys_id={robot.sys_id} '{s.name}', reporting in {s.coord_frame_id!r}")

# ===== as found =====
found = robot.turret_actuators()
print(f"found: {report(found)}")

# ===== a rate: SET_TURRET_RATES =====
print(f"\n-- heading rate {RATE:g} rad/s for {RATE_SAMPLES / HZ:.1f} s --")
for i in range(RATE_SAMPLES):
    robot.set_turret_rates(RATE, 0.0)
    robot.rate(HZ)
    if i % 10 == 9:
        print(f"   {report(robot.turret_actuators())}")
# A rate latches: without this the pylon would keep turning to its limit.
robot.set_turret_rates(0.0, 0.0)

# ===== a target: SET_TURRET_JOINTS =====
heading = deg(AIM_HEADING_DEG)
pitch = deg(AIM_PITCH_DEG)
aim(robot, heading, pitch)

# ===== the laser =====
laser(robot, True)
laser(robot, False)

# ===== srv/turret: narrow the heading limits, then restore them =====
# Nothing is sent between the two calls but the calls themselves: the head
# moves on the target latched above and whatever the limits now allow.
robot.configure_turret(heading_min=deg(-NARROW_DEG), heading_max=deg(NARROW_DEG))
print(
    f"\n-- srv/turret: heading limits +/-{NARROW_DEG:g} deg; the {AIM_HEADING_DEG:g} deg "
    "target is clamped to them --"
)
watch(robot)

robot.configure_turret(
    heading_min=deg(-SHIPPED_HEADING_DEG), heading_max=deg(SHIPPED_HEADING_DEG)
)
print(
    f"\n-- srv/turret: the shipped +/-{SHIPPED_HEADING_DEG:g} deg back. The target was "
    "latched all along, so the head walks back out to it --"
)
watch(robot)

# ===== hand it back =====
aim(robot, found.heading, found.pitch)
robot.set_turret_laser(found.laser_on)
print("pose and laser back as found. Scene-authored: left holding that pose, never deleted.")
