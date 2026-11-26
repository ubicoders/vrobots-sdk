# Rail arm

A six-axis industrial arm riding an 8 m linear rail, posed by joint setpoints, with a vacuum pad on its flange.

## Identity

| Property | Value |
|---|---|
| `RobotType` | `RailArm` |
| Catalog key | `kr210_rail` |
| Synonyms | `railarm`, `rail_arm`, `kuka_kr210_rail` |
| Creatable | yes, on the Pro plan |
| Scene-authored | yes, the Warehouse scene ships one, pinned at `sys_id` 2 |
| Wire name | `kuka_kr210_rail` |
| Default frame | `fru` |
| Type-specific service | `srv/arm`, shared with the [table arm](11-table-arm.md) |

The Warehouse scene pins its robots, so the rail arm is `sys_id` 2 on every load, beside the
[forklift](08-forklift.md) at 0 and the [pallet robot](09-palletrobot.md) at 1. Attach with
`connect(RobotType::RailArm, Some(2))`.

## Physical model

A KUKA KR210 on a linear track: seven joints, the rail in metres and A1 to A6 in radians.
The arm is **kinematic**. Each joint is a position servo that moves toward its setpoint at
its own top speed, with no acceleration ramp and no coordination between joints, so one far
setpoint starts every joint at full speed and they arrive at different times.

The range, speed limit and home of every joint are in the actuator table under
[Hardware](#hardware).

The angles are the controller's axis values, not Denavit-Hartenberg joint angles. With A4 at
0 the pad faces straight down whenever A2 + A3 + A5 = 90 degrees, as at home. Nothing on the
wire publishes the ranges, the speeds or a tool pose, so forward kinematics is yours.

The base never moves, and the state's kinematics are the base's. Everything the arm does is
in its actuator report.

**Protective stop.** Every joint step is tested against the scene before it is taken. A step
that would drive the arm into something is refused: the joints stay, the setpoint stays
latched, and the arm halts within a sixteenth of a step of the contact. Backing away is
never refused.

**Vacuum.** A 400 by 300 mm pad. While on and empty, it grips a loose object once the pad is
within 1 cm of it and within 15 degrees of square to its surface, provided the object weighs
no more than the pad's capacity, 400 N by default. Fixed scenery, robots and people are
never gripped. The load is dropped again if `m·|a - g|` exceeds the capacity, so a fast
start can tear a heavy load off.

## Commands accepted

| Command | Method | Units and range | Status |
|---|---|---|---|
| `SET_ARM_JOINTS` (315) | `set_arm_joints(setpoints)` | 7 values: rail m, A1 to A6 rad | live |
| `SET_ARM_VACUUM` (316) | `set_arm_vacuum(on)` | on or off | live |

Both latch. Each joint setpoint is clamped to its range. The SDK refuses a wrong count or a
non-finite entry before publishing, because the simulator drops such a command whole and
says nothing. `SET_ARM_GRIPPER` belongs to the table arm and is ignored here.

Nothing replies, and a move ends one of three ways: the joints arrive, CONTACT turns
non-zero, or a clamped joint stops at its limit with `joints_normalized` at -1 or 1. The
`move_to` helper tells them apart:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex39_rail_arm.rs`:

```rust
fn move_to(robot: &VirtualRobot, goal: &[f64; rail_arm::JOINT_COUNT]) -> Result<(), VrError> {
    let t0 = robot.states().elapsed;
    for i in 0..MOVE_SAMPLES {
        robot.set_arm_joints(goal)?;
        robot.rate(HZ);
        let arm = robot.rail_arm_actuators()?;
        let t = robot.states().elapsed - t0;
        let worst = largest_error(&arm.joints, goal);
        if worst < TOLERANCE {
            println!("   t={t:5.2}s  {}  -- arrived", joints(&arm.joints));
            return Ok(());
        }
        if arm.contact != ArmContact::NoContact {
            println!(
                "   t={t:5.2}s  {}  -- held short: CONTACT {} (code {})",
                joints(&arm.joints),
                arm.contact.name(),
                arm.contact.code()
            );
            return Ok(());
        }
        if i % 5 == 0 {
            println!(
                "   t={t:5.2}s  {}  worst error {worst:.4}",
                joints(&arm.joints)
            );
        }
    }
    let arm = robot.rail_arm_actuators()?;
    println!(
        "   not there after {:.0} s. joints_normalized {:?}: a joint at -1 or 1 is clamped at \
         its limit",
        f64::from(MOVE_SAMPLES) / HZ,
        arm.joints_normalized
    );
    Ok(())
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex39_rail_arm.cpp`:

```cpp
void move_to(vrsdk::VirtualRobot& robot, const Joints& goal) {
    const double t0 = robot.states().elapsed;
    for (int i = 0; i < MOVE_SAMPLES; ++i) {
        robot.set_arm_joints(goal);
        robot.rate(HZ);
        const vrsdk::RailArmActuators arm = robot.rail_arm_actuators();
        const double t = robot.states().elapsed - t0;
        const double worst = largest_error(arm.joints, goal);
        if (worst < TOLERANCE) {
            std::printf("   t=%5.2fs  %s  -- arrived\n", t, joints(arm.joints).c_str());
            return;
        }
        if (arm.contact != vrsdk::ArmContact::NoContact) {
            std::printf("   t=%5.2fs  %s  -- held short: CONTACT %s (code %d)\n", t,
                        joints(arm.joints).c_str(), vrsdk::to_string(arm.contact),
                        static_cast<int>(arm.contact));
            return;
        }
        if (i % 5 == 0) {
            std::printf("   t=%5.2fs  %s  worst error %.4f\n", t, joints(arm.joints).c_str(),
                        worst);
        }
    }
    const vrsdk::RailArmActuators arm = robot.rail_arm_actuators();
    std::printf(
        "   not there after %.0f s. joints_normalized %s: a joint at -1 or 1 is clamped at its "
        "limit\n",
        MOVE_SAMPLES / HZ, list_of(arm.joints_normalized).c_str());
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex39_rail_arm.py`:

```python
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
```

{{#endtab }}
{{#endtabs }}

A clear move prints a progress line every fifth sample and ends on the `arrived` line, with
the rail in metres and the axes in degrees. The 1 m of rail is the slowest part of this
move, and its last line reads:

```text
   t= 0.81s  rail 5.000 m  A1  +30.0  A2  -70.0  A3  +90.0  A4   +0.0  A5  +70.0  A6   +0.0 deg  -- arrived
```

## Services

The common eight plus `srv/arm`, called through `configure_arm`. Both arms serve it, and
each ignores the other arm's tool fields. Every field is optional and is read live from the
next step. The simulator heals an out-of-range value and still acknowledges `ok`, and no
field can be read back.

| Field | Units | Default | Notes |
|---|---|---|---|
| `fallback_speed_limits` | 7 values: m/s, then rad/s | 1.2 m/s, then 105 / 94 / 100 / 136 / 129 / 206 deg/s | all or nothing; **changes no speed on this arm** |
| `protective_stop` | bool | on | |
| `self_collision` | bool | off | acts only with the protective stop on; untested on this arm |
| `contact_tolerance` | m | 0.005 | how deep a link may press before the stop; below 0 becomes 0 |
| `strike_speed` | m/s | 1 | above it a collision is reported as a strike; 0 or less reverts |
| `collision_rearm` | s | 1 | a party met again within it is the same episode; below 0 becomes 0 |
| `suction_capacity` | N | 400 | a held load needing more drops on the next step; below 0 becomes 0 |
| `suction_reach` | m | 0.01 | below 0 becomes 0 |
| `suction_max_tilt` | rad | 0.262 | clamped to 0 to π/2 |
| `suction_accel_window` | s | 0.5 | averaging window for the load's acceleration; 0 or less reverts, capped at 5 |
| `gripper_speed`, `gripper_max_payload` | | | ignored: this arm has no gripper |

> **Gotcha.** `fallback_speed_limits` is a fallback, not the joint speed. It backs only a
> joint that reports no speed limit of its own, and every joint of this arm reports one, so
> the request is acknowledged and no joint changes speed.

`reset()` lets go of any load where it is, at rest, switches the vacuum off, clears CONTACT
and teleports every joint home **with its setpoint**. A client still publishing setpoints
drives the arm away again on the next step. The full field reference is on
[The arms](../ch06-services/10-arm-config.md).

## Hardware

The simulator's sources hold no scale drawing of this arm, so the figure below is a block
schematic drawn for this page. It shows the order of the joints, their limits, the tool and
the two cameras, and it pretends to no scale or geometry.

[![Rail arm block schematic: an 8 m rail, a carriage, joints A1 to A6 in series, a vacuum tool on the flange, and an overhead and a wrist camera](images/rail-arm-schematic.svg)](images/rail-arm-schematic.svg)

*Block schematic, not to scale. The carriage rides the rail, A1 to A6 follow in series, and
the vacuum tool hangs from the flange after A6. The wrist camera rides the tool; the
overhead camera is fixed to the base.*

### Actuators

Nine entries. `pwm` is 0 throughout.

| # | Device | Kind | Driven by | `normalized` | `measured` | Range | Speed limit | Home |
|---|---|---|---|---|---|---|---|---|
| 0 | rail, a linear track | `LINEAR` | entry 0 of `SET_ARM_JOINTS`, m | position across its range, [-1, 1] | m | 0 to 8 m | 1.2 m/s | 4 m |
| 1 | A1, revolute | `SERVO` | entry 1, rad | position across its range, [-1, 1] | rad | -185 to 185 degrees | 105 deg/s | 0 |
| 2 | A2, revolute | `SERVO` | entry 2, rad | position across its range, [-1, 1] | rad | -140 to -5 degrees | 94 deg/s | -60 degrees |
| 3 | A3, revolute | `SERVO` | entry 3, rad | position across its range, [-1, 1] | rad | -120 to 168 degrees | 100 deg/s | 90 degrees |
| 4 | A4, revolute | `SERVO` | entry 4, rad | position across its range, [-1, 1] | rad | -350 to 350 degrees | 136 deg/s | 0 |
| 5 | A5, revolute | `SERVO` | entry 5, rad | position across its range, [-1, 1] | rad | -122.5 to 122.5 degrees | 129 deg/s | 60 degrees |
| 6 | A6, revolute | `SERVO` | entry 6, rad | position across its range, [-1, 1] | rad | -350 to 350 degrees | 206 deg/s | 0 |
| 7 | VACUUM, a 400 by 300 mm foam pad | `VACUUM` | `SET_ARM_VACUUM` | the latch: 0 off, 1 on | held mass, kg; 0 is nothing held | holds up to 400 N; seals within 0.01 m and 15 degrees of square | switches at once | off |
| 8 | CONTACT | `CONTACT` | nothing: it is a reading | 0 | 0 none, 1 tool contact, 2 collision stop | | | 0 |

No joint has an acceleration ramp: a joint starts and stops at its speed limit within one
physics step. That includes the track, whose acceleration is not modelled: it reaches
1.2 m/s at once, so a 2 m move takes about 1.7 s. `rail_arm_actuators()` decodes the report into `joints`, `joints_normalized`,
`vacuum_on`, `holding`, `held_mass_kg` and `contact`. The table arm reports nine entries
too, laid out differently after the joints, so decode by robot type and never by count;
`properties()` publishes the kind of each entry. See
[Actuator layouts and typed readers](../ch03-reading-state/10-actuator-layouts.md).

CONTACT names the last joint step only. Tool contact, the pad or its load meeting something,
is the normal end of a descent and raises no event. A collision stop on any other part of
the arm also publishes one safety event per episode; see
[Safety events](../ch10-events/00-safety-events.md).

### Sensors

The standard five are fitted, at the origin of the base. The base is fixed, so they publish
constants plus their noise: nothing here measures the joints.

| Sensor | Measures | Units | Rate | Mount | Frame | What to know |
|---|---|---|---|---|---|---|
| Accelerometer | specific force on the body axes | m/s² | every state sample | robot origin, no offset, no rotation | the robot's, `fru` | +1 g at rest and 0 in free fall; clips without warning at ±2 g (19.61 m/s²); resolution about 0.0006 m/s²; white noise, a turn-on bias and a slowly drifting bias |
| Gyroscope | body rates | rad/s | every state sample | robot origin, no offset, no rotation | the robot's, `fru` | clips at ±250 deg/s (4.36 rad/s); resolution about 0.00013 rad/s; a turn-on bias of about 0.3 deg/s, one sigma, plus drift |
| Magnetometer | ambient magnetic field on the body axes | gauss | every state sample | robot origin, no offset, no rotation | the robot's, `fru` | clips at ±49.1 G; resolution 0.0015 G; a fixed turn-on offset of about 0.05 G, one sigma |
| Barometer | static pressure, and the pressure altitude computed from it | Pa, m | every state sample | robot origin | none: the readings are scalars | pressure noise of 3 Pa, about 0.25 m near sea level; the altitude is computed from the noisy pressure |
| GNSS | geodetic position and velocity | deg, deg, m and m/s | every state sample | robot origin, no antenna offset | geographic in every frame: latitude, longitude, altitude, and velocity as north, east, down | position noise of 1.5 m horizontal and 3.0 m vertical, velocity noise of 0.05 m/s horizontal and 0.08 m/s vertical, one sigma |
| Optical flow | horizontal ground velocity on the body axes | m/s | every state sample, once fitted | robot origin | the robot's, `fru` | **not fitted by default**; fit it through sensor config; noise of 0.05 m/s per axis |

The noise figures are the simulator's defaults, and every one of them is settable: see
[Sensor noise](../ch06-services/03-sensor-config.md). The fields each device publishes are on
[Sensors](../ch03-reading-state/03-sensors.md). A device keeps the robot's frame unless you
give it one of its own through [Coordinate frames](../ch06-services/04-frames-config.md).
<!-- VERIFY: GNSS update rate. The Sensors page describes a receiver at about 5 Hz; the simulator's notes describe a fresh fix on every physics step. -->

### Cameras

Two cameras, so this robot's defaults are not the `front_left` and `front_right` pair most
robots carry.

| Name | Mount in `fru`, m | Orientation | Stream | Focal lengths | Field of view | Clip | Rate | Plan |
|---|---|---|---|---|---|---|---|---|
| `overhead` | (-0.90, -3.94, 4.80), fixed to the base | straight down | 720p rgba8, 1280 by 720 | fx 640 px, fy 640 px | 90.0 by 58.7 degrees | 0.3 to 30 m | every rendered frame, about 60 per second | Community and Pro |
| `wrist` | on the vacuum tool, moving with it: 0.04 m beyond one short edge of the pad and 0.02 m behind the pad face | tilted 25 degrees toward the pad axis, so its centre line crosses the axis about 0.5 m below the pad face | 720p rgba8, 1280 by 720 | fx 834 px, fy 834 px | 75.0 by 46.7 degrees | 0.01 to 10 m | every rendered frame, about 60 per second | Pro |

<!-- VERIFY: rail arm camera mounts, focal lengths and clip planes. They are the shipped model's values, not measured live. -->

Open one with `open_camera("overhead", "720p", "rgba8")`; on the Warehouse arm the topics
are `vrobots/2/i/cam/overhead/720p_rgba8` and `vrobots/2/i/cam/wrist/720p_rgba8`. Every
frame carries the intrinsics and the mount pose it was rendered with, in the robot's frame,
so a `wrist` frame never has to be matched to a joint sample: read the pose off the frame.
See [Cameras and images](../ch05-cameras/00-intro.md).

### Lidars

None.

### Body and frame

| Property | Value |
|---|---|
| Arm | a KUKA KR 210 R3100-2 on an ABB IRBT 7004 track |
| Mass on the properties topic | 1551 kg, the whole rig: the track carriage and robot base at 630 kg, plus the arm's links. The robot alone is about 1134 kg by the maker's figure |
| Centre of gravity | follows the pose: moving the rail 1 m moves it 1 m |
| Inertia | not published; see [Robot properties](../ch06-services/13-properties.md#arms-publish-the-whole-rig-and-no-inertia) |
| Track travel | 8 m, at up to 1.2 m/s |
| Reach at home | about 2.3 m from the A1 axis to the pad |
| Robot origin | the base, which never moves |
| Default frame | `fru`: x forward, y right, z up |

Everything is SI: metres on the rail, radians on the axes. Joint values carry no axes, so
nothing is frame-converted. The base publishes `fru`, with the `axis_convention` tag
unspecified because `fru` has no `Axes` constant. The state's kinematics are the base's,
and everything the arm does is in its actuator report.

## Known quirks

> **Gotcha.** Only the end pose of each step is tested. A physics step is 0.02 s. At full
> speed a point on the tool moves up to about 12 cm in one step (A1 at 105 deg/s), and the
> track alone carries the whole arm 2.4 cm, so an obstacle thinner than that can be passed
> through. Stream intermediate setpoints along your own path when clearance matters.

Four more:

- A dropped or too-heavy load is ignored until it leaves the pad's reach or the vacuum goes
  off. To retry a grip, switch the vacuum off and on again.
- A load is destroyed with the arm if the arm is deleted while holding it.
- There is no joint velocity, limit flag or torque on the wire. The report has three arrays
  and the joints use `normalized` and `measured`.
- A grip that never happens is explained only in the simulator's own log and panel. On the
  wire the evidence is `vacuum_on` true with `held_mass_kg` at 0.

## Example

ex39 moves the arm to a pose that keeps the pad facing down, switches the vacuum on and off
in mid air, and drives the joints back to where it found them:

```sh
cargo run -p vrobots-examples --bin ex39_rail_arm
./target/cpp-build/ex39_rail_arm
python examples/python/ex39_rail_arm.py
```

It takes an optional `sys_id` and defaults to 2, the Warehouse rail arm. An arm found
holding a load is left alone, because switching the vacuum off would drop it. The example
never deletes the arm.

**Next:** [Table arm](11-table-arm.md)

**See also:** [Moving the arms](../ch04-commands/13-arms.md), [The arms](../ch06-services/10-arm-config.md), [Safety events](../ch10-events/00-safety-events.md), [Actuators](../ch03-reading-state/05-actuator.md)
