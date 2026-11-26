"""ex44 - robot_properties: what a robot is, and what each actuator index means.

    python examples/python/ex44_robot_properties.py        # the warehouse forklift, sys_id 0
    python examples/python/ex44_robot_properties.py <id>   # any robot

Every robot publishes `VRobotProperties` on `vrobots/{id}/z/properties`: its
mass, centre of gravity, inertia, and the **kind of every actuator entry**.
These are the values the simulation runs *with*, not per-step output, so they
ride their own topic -- published when the robot's wire opens, again on any
parameter change, and as a 1 Hz keepalive -- and `properties()` is a one-shot
read: subscribe, take the first sample, undeclare.

The layout is the type check
----------------------------

`actuator.pwm`, `normalized` and `measured` in the state are index-parallel
arrays, and nothing in a state message says what index `i` *is* -- a wheel
speed in rad/s, a lift height in m, a held mass in kg. `actuator_types` does,
index for index, and that makes it the cheap answer to "what am I attached
to?": `connect()` takes the robot type on trust, while the layout is the
robot's own word. (ex30's capability probe, a service query timed until it
fails, was the only answer before this topic existed.) Once a handle has read
it, the typed getters -- `forklift_actuators()` and its siblings -- refuse a
robot whose layout contradicts theirs, which is how ex37-ex41 make sure.

As observed on the simulator:

| robot | `actuator_types` |
|---|---|
| forklift | wheel, servo, linear, linear |
| pallet robot | wheel x4, linear, servo |
| rail arm | linear, servo x6, vacuum, contact |
| table arm | servo x6, gripper, contact, hold |
| turret | servo, servo, laser |
| truck | wheel x4, servo |
| multirotor, half-drone | rotor per rotor |
| msd, cart-pole | unspecified: no kind exists for a force |

`cg` and `inertia` are on body axes in the frame the message names, the
robot's own. What the properties do **not** carry is how a robot is tuned: no
lift speed, no wheel geometry, no joint range. No config service reads
anything back.

The new config services
-----------------------

Each new robot type has one, and every one takes only the fields passed to
it, as keyword arguments, and acks a **receipt**, not a result (ex22): the
state stream is the confirmation. This example uses the forklift's, on the
lift speed, and measures the carriage to prove it landed. The others are the
same shape:

- `configure_forklift(...)` -- `srv/forklift`: drive, steering, the mast's
  ranges and speeds, the pulse band, the reset pulses.
- `configure_palletrobot(...)` -- `srv/palletrobot`: the pulse band, the
  virtual mecanum geometry, the drive, the pad.
- `configure_arm(...)` -- `srv/arm`, one service for both arms: the
  protective stop, and the tool -- `suction_*` on the rail arm, `gripper_*`
  on the table arm, each arm ignoring the other's.
- `configure_turret(...)` -- `srv/turret`: joint limits and rates, the
  laser's range, the stereo pair (ex41).
- `configure_lidars([LidarConfigEntry(name, ...)])` -- `srv/lidars`, on every
  robot that carries one: mount, segments and layer mask, by lidar name. Pro
  only: on Community it refuses in its reply, one of the two services that
  ever say no (`srv/skin` is the other, ex23).

None has a getter, so "restore" means sending the shipped value back, which
is what this example does. A setting another client had changed comes back
as the shipped one. Everything set survives `reset()`; nothing survives a
simulator restart.
"""

import math
import sys

from vrsdk import (
    ActuatorKind,
    ForkliftActuators,
    RobotType,
    VirtualRobot,
    VrError,
    forklift_actuator,
)

USAGE = """\
Reads any robot; the default is the warehouse scene's forklift, sys_id 0, the
only robot the srv/forklift half below runs on. For another robot, pass its id:

    python examples/python/ex44_robot_properties.py 1

The scene has to be loaded and in Play mode, and on the Community tier only the
focused robot is on the wire at all. List what is publishing with:

    vrobots topic list"""

SYS_ID = 0  # the warehouse scene's forklift

# The shipped lift band, for lift_m(). This example changes only the speed.
LIFT_MIN_M = -0.215
LIFT_MAX_M = 1.11

SHIPPED_LIFT_SPEED = 0.5  # m/s, what restore sends back
SLOW_LIFT_SPEED = 0.25  # m/s, half of it
NEUTRAL_US = 1500.0
HZ = 25.0
MOVE_SAMPLES = 150  # 6 s; the slow move takes under 3
MOVING_M = 0.02  # a carriage this far from its start has started


def lift_speed(robot, lift_us, width_us):
    """Drive the lift to `lift_us` -- drive centred, forks at `width_us` -- and
    measure how fast the carriage travelled, m/s.

    Timed over the middle of the move, from the first sample that has left the
    start to the first that has arrived, so the command's own latency does not
    count against the speed. Height and time come from one snapshot, decoded in
    hand, so they belong to the same sample.
    """
    want = lift_m(lift_us)
    start = robot.forklift_actuators().lift_height
    left_at = None  # (elapsed, height) once it has started
    for _ in range(MOVE_SAMPLES):
        robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us)
        robot.rate(HZ)
        s = robot.states
        a = ForkliftActuators.from_actuator(s.actuator)
        if a is None:
            continue
        if left_at is None:
            if abs(a.lift_height - start) > MOVING_M:
                left_at = (s.elapsed, a.lift_height)
        else:
            t_from, h_from = left_at
            if abs(a.lift_height - want) < MOVING_M:
                metres = abs(a.lift_height - h_from)
                seconds = s.elapsed - t_from
                print(
                    f"   {start:+.3f} m -> {a.lift_height:+.3f} m: {metres:.3f} m in "
                    f"{seconds:.2f} s mid-move = {metres / seconds:.2f} m/s"
                )
                # Let it cover the last couple of centimetres before the next move.
                for _ in range(5):
                    robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us)
                    robot.rate(HZ)
                return metres / seconds
    print(f"   the carriage did not arrive in {MOVE_SAMPLES / HZ:.0f} s")
    return 0.0


def lift_m(us):
    """Where a lift pulse puts the carriage: linear across the shipped band (ex37)."""
    return LIFT_MIN_M + (us - 1100.0) / 800.0 * (LIFT_MAX_M - LIFT_MIN_M)


def unit(kind):
    """What a `measured` entry of this kind is in -- the reason the layout exists."""
    return {
        ActuatorKind.ROTOR: "rad/s, rotor speed",
        ActuatorKind.SERVO: "rad, a joint or steering angle",
        ActuatorKind.WHEEL: "rad/s, wheel speed",
        ActuatorKind.SURFACE: "rad, surface deflection",
        ActuatorKind.LINEAR: "m, along a prismatic axis",
        ActuatorKind.VACUUM: "kg held by the suction pad, 0 = nothing",
        ActuatorKind.LASER: "m to the hit, 0 = off or none",
        ActuatorKind.GRIPPER: "m between the pads",
        ActuatorKind.CONTACT: "an ArmContact code: 0 none, 1 tool, 2 stop",
        ActuatorKind.HOLD: "kg held by the tool, 0 = nothing",
        ActuatorKind.UNSPECIFIED: "no kind fits, e.g. a force in N",
    }.get(kind, "a kind newer than this SDK")


def sys_id_from_args():
    """The robot's sys_id: the one command-line argument, or `SYS_ID`."""
    if len(sys.argv) < 2:
        return SYS_ID
    arg = sys.argv[1]
    if arg.isdecimal() and int(arg) < 2**32:  # a u32, as the wire carries it
        return int(arg)
    print(f"usage: {sys.argv[0]} [sys_id]\n\n{USAGE}", file=sys.stderr)
    sys.exit(2)


# ===== setup =====
sys_id = sys_id_from_args()
# The type is a claim: properties() below reads what the robot really is.
robot = VirtualRobot(RobotType.FORKLIFT, sys_id=sys_id)
try:
    robot.connect()
except VrError as e:
    print(f"no robot at sys_id {sys_id}: [{e.code}] {e}\n\n{USAGE}", file=sys.stderr)
    sys.exit(1)

# ===== z/properties =====
try:
    p = robot.properties()
except VrError as e:
    print(f"no properties from sys_id {sys_id}: [{e.code}] {e}", file=sys.stderr)
    sys.exit(1)
s = robot.states
print(
    f"sys_id {robot.sys_id} '{s.name}': {p.mass:.1f} kg, centre of gravity ({p.cg[0]:+.3f}, "
    f"{p.cg[1]:+.3f}, {p.cg[2]:+.3f}) m, in {p.coord_frame_id!r}"
)
print("inertia about the centre of gravity, kg*m^2:")
for row in p.inertia:
    print(f"   [{row[0]:12.1f} {row[1]:12.1f} {row[2]:12.1f}]")
if not p.moment_arms:
    print("moment arms: none published for this type")
else:
    print(f"moment arms: {p.moment_arms} m")

# ===== the actuator layout, index by index =====
# Index-parallel to the state's actuator arrays: kind i is what measured[i] is.
print(f"\n{len(p.actuator_types)} actuator entries; the state's measured value beside each:")
values = s.actuator.measured
for i, kind in enumerate(p.actuator_types):
    measured = values[i] if i < len(values) else math.nan
    print(f"   [{i}] {ActuatorKind.name(kind):<12} {measured:+10.3f}  {unit(kind)}")

# ===== srv/forklift: only on a forklift =====
if p.actuator_types != forklift_actuator.KINDS:
    print(
        "\nnot a forklift's layout, so there is no srv/forklift to call here; the header "
        "lists the other robots' configure_* calls. Nothing was sent."
    )
    sys.exit(0)

# The forks as found: the echo is the latch (report order, pwm[DRIVE] is the
# throttle). The lift moves half the band away, whichever way there is room.
pwm = s.actuator.pwm
found_lift_us = (
    float(pwm[forklift_actuator.LIFT]) if len(pwm) > forklift_actuator.LIFT else 1100.0
)
found_width_us = (
    float(pwm[forklift_actuator.FORK_WIDTH]) if len(pwm) > forklift_actuator.FORK_WIDTH else 1301.0
)
lift_to_us = found_lift_us + 400.0 if found_lift_us < 1500.0 else found_lift_us - 400.0

print(
    f"\n-- srv/forklift: lift_speed {SLOW_LIFT_SPEED:g} m/s "
    f"(shipped: {SHIPPED_LIFT_SPEED:g}) --"
)
robot.configure_forklift(lift_speed=SLOW_LIFT_SPEED)
print("acked -- a receipt. The carriage is the proof:")
slow = lift_speed(robot, lift_to_us, found_width_us)

# ===== restore it =====
print(f"\n-- srv/forklift: lift_speed back to the shipped {SHIPPED_LIFT_SPEED:g} m/s --")
robot.configure_forklift(lift_speed=SHIPPED_LIFT_SPEED)
shipped = lift_speed(robot, found_lift_us, found_width_us)

print(
    f"\nmeasured {slow:.2f} m/s after asking for {SLOW_LIFT_SPEED:g}, {shipped:.2f} m/s after "
    f"asking for {SHIPPED_LIFT_SPEED:g}. The forks are back at {found_lift_us:g} / "
    f"{found_width_us:g} us, the drive centred. Scene-authored: never deleted."
)
