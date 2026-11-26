"""ex38 - hello_palletrobot: four wheel channels, no twist command, and the mix is yours.

    python examples/python/ex38_hello_palletrobot.py        # the warehouse pallet robot, sys_id 1
    python examples/python/ex38_hello_palletrobot.py <id>   # one at another id

The pallet robot is the low mecanum robot that drives under a pedestal and
lifts it on a turning pad. The warehouse scene pins it at `sys_id` 1.
`SET_PALLETROBOT` is six pulse widths, in one order for the command and the
actuator report alike (`palletrobot_channel.*`):

| index | channel | 1100 us | 1500 us | 1900 us |
|---|---|---|---|---|
| 0-3 | wheel ESC FL, FR, RL, RR | -28 rad/s | stop | +28 rad/s, rolling forward |
| 4 | pad lift, a position servo | 0 m | 0.03 m | 0.06 m |
| 5 | pad turn, a rate | -90 deg/s | hold | +90 deg/s, clockwise from above |

The robot takes no twist
------------------------

There is no "go right at 0.3 m/s" command. Each wheel has its own ESC, and
turning a body twist into four wheel speeds -- the mecanum mix -- is the
client's job. It is a few lines of arithmetic, in `mix()` below, for rollers
in the standard X layout, on the robot's virtual geometry R = 0.054 m,
lx = 0.35 m and ly = 0.30 m:

    FL = (vx + vy + (lx+ly) wz) / R        FR = (vx - vy - (lx+ly) wz) / R
    RL = (vx - vy + (lx+ly) wz) / R        RR = (vx + vy - (lx+ly) wz) / R
    us = round(1500 + 400 * w / 28)        for each wheel speed w in rad/s

`vx` is forward and `vy` right in m/s, `wz` clockwise seen from above in
rad/s: FRU, the frame this robot reports in unless `srv/frames` changed it,
so the state's `kin.lin_vel[0]`, `kin.lin_vel[1]` and `kin.ang_vel[2]` are the
same three numbers coming back. The wheel channels themselves are physical
(+ rolls forward) and are never frame-converted. If any wheel would pass
28 rad/s, all four scale down by the same factor: clipping one alone changes
the direction of travel.

The geometry is virtual -- the model has no rollers -- and nothing on the
wire reads it back, so a client keeps its own copy. If `srv/palletrobot` has
changed it, the constants below must follow.

Deadband, ramps, and what confirms a move
-----------------------------------------

The wheels and the pad turn have a +/-20 us deadband, which makes about
0.08 m/s the slowest controllable speed; the pad lift has none. The wheels
ramp at 60 rad/s^2 and the pad climbs at 0.03 m/s, so the actuator report,
`palletrobot_actuators()`, is where a move is confirmed. Its wheel speeds are
the motors' own: a blocked robot still reports them, and `kin` is what the
body actually did.

Leaving it as found
-------------------

Every move is undone by its mirror image -- strafe right then left, turn one
way then back -- so the robot ends where it started, to within the open-loop
error. The pad returns to the pulse it was found at, and the last frame
stops the wheels; with all four stopped the robot holds where it parked. Let
it run to the end: a Ctrl-C leaves the last frame latched. Scene-authored:
attached by `sys_id`, never deleted.
"""

import math
import sys

from vrsdk import RobotType, VirtualRobot, VrError, palletrobot_channel

USAGE = """\
The warehouse scene pins its pallet robot at sys_id 1, which is the default. For
one at another id, pass it:

    python examples/python/ex38_hello_palletrobot.py 1

The scene has to be loaded and in Play mode, and on the Community tier only the
focused robot is on the wire at all. List what is publishing with:

    vrobots topic list"""

SYS_ID = 1  # the warehouse scene's pallet robot

# The virtual mecanum geometry: the shipped srv/palletrobot values.
WHEEL_RADIUS_M = 0.054  # R
HALF_WHEELBASE_M = 0.35  # lx, centre to the front and rear axles
HALF_TRACK_M = 0.30  # ly, centre to the left and right wheels
MAX_WHEEL_RAD_S = 28.0  # a wheel at 400 us off neutral
PAD_MAX_M = 0.06  # the pad at the top of the band

NEUTRAL_US = 1500.0  # a wheel stopped, the pad turn holding
PAD_TOP_US = 1900.0
STRAFE_M_S = 0.3
TURN_RAD_S = 0.5  # ~29 deg/s
HZ = 25.0
MOVE_SAMPLES = 50  # 2 s per move
PAD_SAMPLES = 100  # 4 s; the whole 0.06 m takes 2
PAD_TOLERANCE_M = 0.001


def attach(sys_id):
    """Attach to `sys_id`, and make sure it really is a pallet robot.

    `connect()` takes the robot type on trust; `z/properties` names it. Once
    this handle has read it (ex44), `palletrobot_actuators()` refuses a robot
    whose layout is not four wheels, a linear pad lift and a servo pad turn.
    """
    robot = VirtualRobot(RobotType.PALLETROBOT, sys_id=sys_id)
    robot.connect()
    robot.properties()
    robot.palletrobot_actuators()
    return robot


def drive(robot, label, vx, vy, wz, pad_us):
    """Mix one body twist into wheel pulses and hold it for `MOVE_SAMPLES`.

    The pad pulse rides along in every frame: a frame is all six channels, and
    one that left the pad out would move it.
    """
    w = mix(vx, vy, wz)
    us = [to_us(w[0]), to_us(w[1]), to_us(w[2]), to_us(w[3])]
    print(f"\n-- {label}: vx {vx:+.2f} m/s, vy {vy:+.2f} m/s, wz {wz:+.2f} rad/s --")
    print(
        f"   mix: [FL, FR, RL, RR] = [{w[0]:+.2f}, {w[1]:+.2f}, {w[2]:+.2f}, {w[3]:+.2f}] rad/s "
        f"-> [{us[0]:.0f}, {us[1]:.0f}, {us[2]:.0f}, {us[3]:.0f}] us"
    )
    for i in range(MOVE_SAMPLES):
        robot.set_palletrobot(us[0], us[1], us[2], us[3], pad_us, NEUTRAL_US)
        robot.rate(HZ)
        if i % 25 == 24:
            # FRU: lin_vel is [forward, right, up], ang_vel[2] clockwise from above.
            s = robot.states
            print(
                f"   {report(robot.palletrobot_actuators())}  body vx {s.kin.lin_vel[0]:+.2f} "
                f"vy {s.kin.lin_vel[1]:+.2f} m/s, wz {s.kin.ang_vel[2]:+.2f} rad/s"
            )


def pad(robot, pad_us):
    """Move the pad to `pad_us`, wheels stopped, until the report says it is there."""
    # A position servo with no deadband: linear from 0 m at 1100 to the top at 1900.
    want = PAD_MAX_M * (pad_us - 1100.0) / 800.0
    print(f"\n-- pad lift {pad_us:g} us: {want:.3f} m by the band --")
    t0 = robot.states.elapsed
    for i in range(PAD_SAMPLES):
        robot.set_palletrobot(NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, pad_us, NEUTRAL_US)
        robot.rate(HZ)
        a = robot.palletrobot_actuators()
        t = robot.states.elapsed - t0
        if abs(a.pad_height - want) < PAD_TOLERANCE_M:
            print(f"   t={t:5.2f}s  {report(a)}  -- there")
            return
        if i % 10 == 0:
            print(f"   t={t:5.2f}s  {report(a)}")
    print(f"   not there after {PAD_SAMPLES / HZ:.0f} s: has srv/palletrobot changed the pad?")


def mix(vx, vy, wz):
    """The mecanum mix: a body twist into four wheel speeds, rad/s, `[FL, FR, RL, RR]`.

    `vx` forward and `vy` right in m/s, `wz` clockwise from above in rad/s.
    """
    k = HALF_WHEELBASE_M + HALF_TRACK_M  # lx + ly
    wheels = [
        (vx + vy + k * wz) / WHEEL_RADIUS_M,
        (vx - vy - k * wz) / WHEEL_RADIUS_M,
        (vx - vy + k * wz) / WHEEL_RADIUS_M,
        (vx + vy - k * wz) / WHEEL_RADIUS_M,
    ]

    # Too fast for one wheel: slow all four by the same factor, which keeps the
    # direction of travel. Clipping only the fast one would bend it.
    fastest = 0.0
    for w in wheels:
        fastest = max(fastest, abs(w))
    if fastest > MAX_WHEEL_RAD_S:
        wheels = [w * (MAX_WHEEL_RAD_S / fastest) for w in wheels]
    return wheels


def to_us(rad_per_s):
    """One wheel speed as the pulse that asks for it: 400 us either side of neutral
    is `MAX_WHEEL_RAD_S`.

    Rounded to a whole microsecond, which is all the wire carries. The SDK rounds
    to the nearest one as well, so this only makes the printed pulse the one that
    goes out.
    """
    return round(NEUTRAL_US + 400.0 * rad_per_s / MAX_WHEEL_RAD_S)


def report(a):
    """The decoded report, in the units a person reads."""
    fl, fr, rl, rr = a.wheel_speeds
    return (
        f"wheels [{fl:+5.1f}, {fr:+5.1f}, {rl:+5.1f}, {rr:+5.1f}] rad/s  "
        f"pad {a.pad_height:.3f} m  pad angle {math.degrees(a.pad_angle):+.1f} deg"
    )


def sys_id_from_args():
    """The pallet robot's sys_id: the one command-line argument, or `SYS_ID`."""
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
    print(f"no pallet robot at sys_id {sys_id}: [{e.code}] {e}\n\n{USAGE}", file=sys.stderr)
    sys.exit(1)
s = robot.states
print(f"attached to sys_id={robot.sys_id} '{s.name}', reporting in {s.coord_frame_id!r}")

# ===== as found =====
# The echo is the latch, in command order, so the pad's pulse is read off
# it here and sent back at the end.
pwm = s.actuator.pwm
found_pad_us = (
    float(pwm[palletrobot_channel.PAD_LIFT]) if len(pwm) > palletrobot_channel.PAD_LIFT else 1100.0
)
print(f"found: echo {pwm} = [FL, FR, RL, RR, pad lift, pad turn] us")
print(f"       {report(robot.palletrobot_actuators())}")

# ===== strafe right, then back =====
drive(robot, "strafe right", 0.0, STRAFE_M_S, 0.0, found_pad_us)
drive(robot, "strafe back", 0.0, -STRAFE_M_S, 0.0, found_pad_us)

# ===== turn on the spot, clockwise, then back =====
drive(robot, "turn clockwise", 0.0, 0.0, TURN_RAD_S, found_pad_us)
drive(robot, "turn back", 0.0, 0.0, -TURN_RAD_S, found_pad_us)

# ===== stop =====
# A zero twist mixes to 1500 on every wheel: a stop is just another frame.
drive(robot, "stop", 0.0, 0.0, 0.0, found_pad_us)

# ===== the pad: up to the top, then back =====
pad(robot, PAD_TOP_US)
pad(robot, found_pad_us)

# ===== hand it back =====
# pad() ends on a frame with the wheels stopped and the pad turn holding,
# and that frame stays latched.
print(
    f"\nwheels stopped, pad back at {found_pad_us:g} us. Scene-authored: left as it was "
    "found, never deleted."
)
