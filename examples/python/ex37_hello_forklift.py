"""ex37 - hello_forklift: forks up and out, a short drive, a brake, and everything put back.

    python examples/python/ex37_hello_forklift.py        # the warehouse forklift, sys_id 0
    python examples/python/ex37_hello_forklift.py <id>   # a forklift at another id

The warehouse scene pins its counterbalance forklift at `sys_id` 0, so that
is the default. Its front pair drives, its **rear pair steers** -- the tail
swings out in a turn -- and its mast carries two **position servos**. One
command, `SET_FORKLIFT`, drives all four, RC style:

| channel | 1100 us | 1500 us | 1900 us |
|---|---|---|---|
| steer | full left | straight | full right (45 deg) |
| throttle | full reverse | stop, on the idle brake | full forward (~5 m/s) |
| lift | -0.215 m | 0.448 m | 1.11 m |
| fork width | 0.158 m | 0.746 m | 1.334 m |
| brake, an optional fifth | released | half | full |

Two kinds of channel
--------------------

Steer and throttle centre on 1500 with a +/-20 us deadband, like the truck's
(ex05). Lift and fork width have no centre at all: every microsecond is a
place on the mast, linear across the band -- the arithmetic in `lift_m()` and
`width_m()` below. And the mast **slews** there, the lift at 0.5 m/s and the
width at only 0.1 m/s, so `actuator.pwm` echoes a new pulse on the very next
sample while the forks are still on their way. The move is done when
`actuator.measured` says so, and `forklift_actuators()` decodes that into
metres: the typed actuator report is the confirmation, not the echo.

The report is not in command order
----------------------------------

The command is `[steer, throttle, lift, width]`. The report leads with the
drive -- `[drive, steering, lift, width]` -- so `actuator.pwm[0]` echoes the
**throttle**. `forklift_channel.*` indexes the command, `forklift_actuator.*`
the report.

The brake
---------

A fifth channel, `set_forklift`'s optional `brake_us`, and **bottom-anchored**:
1100 is released and 1900 is full, not 1500. It latches with the rest of the
frame, and the next four-channel `set_forklift`, one without `brake_us`,
releases it. The report has no brake entry; the proof is the drive speed
falling to zero.

Leaving it as found
-------------------

Every frame latches, so whatever this program sends last is what the
forklift keeps doing after it exits. The echo *is* the latch, so the forks'
pulses are read off it at the start and sent again at the end, with the drive
centred and the brake released. The forklift drives back as far as it went --
open loop, so to within a few centimetres. `reset()` would be quicker -- it
snaps the mast to the reset pulses instead of slewing it -- but it also
teleports the forklift to where the scene started it, which is "home", not
"where it was found" (ex21).

Let it run to the end: a Ctrl-C mid-drive leaves the throttle latched, and
the forklift keeps driving. Scene-authored: attached by `sys_id`, never
deleted.
"""

import math
import sys

from vrsdk import RobotType, VirtualRobot, VrError, forklift_actuator

USAGE = """\
The warehouse scene pins its forklift at sys_id 0, which is the default. For a
forklift at another id, pass it:

    python examples/python/ex37_hello_forklift.py 0

The scene has to be loaded and in Play mode, and on the Community tier only the
focused robot is on the wire at all. List what is publishing with:

    vrobots topic list"""

SYS_ID = 0  # the warehouse scene's forklift

# The shipped mast ranges, for lift_m() and width_m(). srv/forklift can move
# them (ex44), and then the expectations printed below are wrong.
LIFT_MIN_M = -0.215
LIFT_MAX_M = 1.11
WIDTH_MIN_M = 0.158
WIDTH_MAX_M = 1.334
TYRE_RADIUS_M = 0.36  # front tyres: drive rad/s times this is m/s

NEUTRAL_US = 1500.0  # straight ahead, and stopped on the idle brake
LIFT_UP_US = 1500.0  # the carriage to 0.448 m
WIDTH_OUT_US = 1500.0  # the forks to 0.746 m apart
FORWARD_US = 1600.0  # a quarter of the forward half: gentle
BACK_US = 1400.0  # the same quarter in reverse
BRAKE_FULL_US = 1900.0  # bottom-anchored: 1100 is released
HZ = 20.0
DRIVE_SAMPLES = 50  # 2.5 s each way, about a metre
BRAKE_SAMPLES = 40  # 2 s, far more than a stop needs
MAST_SAMPLES = 200  # 10 s; the slow width needs ~3 s for this move
TOLERANCE_M = 0.005
STOPPED_RAD_S = 0.05


def attach(sys_id):
    """Attach to `sys_id`, and make sure it really is a forklift.

    `connect()` takes the robot type on trust: nothing in a state message names
    it, and a truck at this id would attach just as happily. `z/properties` does
    name it -- the kind of every actuator entry -- and once this handle has read
    it (ex44 prints the rest), `forklift_actuators()` refuses a robot whose
    layout is not `[wheel, servo, linear, linear]` instead of decoding its
    report into wrong numbers.
    """
    robot = VirtualRobot(RobotType.FORKLIFT, sys_id=sys_id)
    robot.connect()
    robot.properties()
    robot.forklift_actuators()
    return robot


def move_mast(robot, lift_us, width_us):
    """Stream one mast frame -- straight, stopped, brake released -- until the
    forks are where the band says those pulses put them.

    The frame latches, so sending it once would do; sending it every iteration
    costs nothing and heals a frame lost on the way.
    """
    want_lift = lift_m(lift_us)
    want_width = width_m(width_us)
    t0 = robot.states.elapsed
    for i in range(MAST_SAMPLES):
        robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us)
        robot.rate(HZ)
        a = robot.forklift_actuators()
        t = robot.states.elapsed - t0
        there = (
            abs(a.lift_height - want_lift) < TOLERANCE_M
            and abs(a.fork_width - want_width) < TOLERANCE_M
        )
        if there:
            print(f"   t={t:5.2f}s  {report(a)}  -- there")
            return
        if i % 10 == 0:
            print(f"   t={t:5.2f}s  {report(a)}")
    print(
        f"   not there after {MAST_SAMPLES / HZ:.0f} s: is the band or a range not the "
        "shipped one (ex44)?"
    )


def drive(robot, label, throttle_us, lift_us, width_us):
    """Hold one throttle, straight ahead, with the forks where they are.

    A frame carries all four channels, so the mast pulses ride along in every
    drive frame: one that left them out would move the forks as well.
    """
    print(f"\n-- {label}: throttle {throttle_us:g} us for {DRIVE_SAMPLES / HZ:.1f} s --")
    for i in range(DRIVE_SAMPLES):
        robot.set_forklift(NEUTRAL_US, throttle_us, lift_us, width_us)
        robot.rate(HZ)
        if i % 10 == 9:
            print(f"   {report(robot.forklift_actuators())}")


def brake(robot, lift_us, width_us):
    """Centre the throttle and add the fifth channel at full until the front axle
    stops. The brake stays latched until the next four-channel frame.
    """
    before = robot.forklift_actuators().drive_speed
    t0 = robot.states.elapsed
    for _ in range(BRAKE_SAMPLES):
        robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us, brake_us=BRAKE_FULL_US)
        robot.rate(HZ)
        a = robot.forklift_actuators()
        if abs(a.drive_speed) < STOPPED_RAD_S:
            print(
                f"   brake {BRAKE_FULL_US:g} us: drive {before:+.2f} -> {a.drive_speed:+.2f} "
                f"rad/s in {robot.states.elapsed - t0:.2f} s"
            )
            return
    print(f"   still rolling after {BRAKE_SAMPLES / HZ:.0f} s of full brake")


def lift_m(us):
    """Where a lift pulse puts the carriage: linear across the shipped band, no
    deadband.
    """
    return LIFT_MIN_M + (us - 1100.0) / 800.0 * (LIFT_MAX_M - LIFT_MIN_M)


def width_m(us):
    """Where a fork-width pulse puts the forks, centre to centre: the same rule."""
    return WIDTH_MIN_M + (us - 1100.0) / 800.0 * (WIDTH_MAX_M - WIDTH_MIN_M)


def report(a):
    """The decoded report, in the units a person reads."""
    return (
        f"lift {a.lift_height:+.3f} m  width {a.fork_width:.3f} m  "
        f"drive {a.drive_speed:+5.2f} rad/s ({a.drive_speed * TYRE_RADIUS_M:+.2f} m/s)  "
        f"steering {math.degrees(a.steering_angle):+5.1f} deg"
    )


def distance(a, b):
    """Straight-line distance between two positions, in whatever frame they share."""
    return math.dist(a, b)


def sys_id_from_args():
    """The forklift's sys_id: the one command-line argument, or `SYS_ID`."""
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
    print(f"no forklift at sys_id {sys_id}: [{e.code}] {e}\n\n{USAGE}", file=sys.stderr)
    sys.exit(1)
s = robot.states
print(f"attached to sys_id={robot.sys_id} '{s.name}', reporting in {s.coord_frame_id!r}")

# ===== as found =====
# The echo is the latch -- the last frame anybody sent -- so it is also what
# to send back at the end. Report order: pwm[DRIVE] is the throttle.
pwm = s.actuator.pwm
found_lift_us = (
    float(pwm[forklift_actuator.LIFT]) if len(pwm) > forklift_actuator.LIFT else 1100.0
)
found_width_us = (
    float(pwm[forklift_actuator.FORK_WIDTH]) if len(pwm) > forklift_actuator.FORK_WIDTH else 1301.0
)
print(f"found: echo {pwm} = [throttle, steer, lift, width] us")
print(f"       {report(robot.forklift_actuators())}")

# ===== forks up, forks out =====
print(
    f"\n-- lift {LIFT_UP_US:g} us, width {WIDTH_OUT_US:g} us: by the band "
    f"{lift_m(LIFT_UP_US):.3f} m and {width_m(WIDTH_OUT_US):.3f} m --"
)
move_mast(robot, LIFT_UP_US, WIDTH_OUT_US)

# ===== a gentle drive forward, and the brake =====
start = robot.states.kin.lin_pos
drive(robot, "forward", FORWARD_US, LIFT_UP_US, WIDTH_OUT_US)
brake(robot, LIFT_UP_US, WIDTH_OUT_US)

# ===== and back the same way =====
drive(robot, "back", BACK_US, LIFT_UP_US, WIDTH_OUT_US)
brake(robot, LIFT_UP_US, WIDTH_OUT_US)

# ===== hand it back =====
# A four-channel frame, so the brake is released too.
print(
    f"\n-- hand it back: straight, stopped, brake released, forks to {found_lift_us:g} / "
    f"{found_width_us:g} us --"
)
move_mast(robot, found_lift_us, found_width_us)
end = robot.states.kin.lin_pos
print(
    f"ended {distance(start, end):.2f} m from where it started (open loop, so not quite 0). "
    "Scene-authored: left as it was found, never deleted."
)
