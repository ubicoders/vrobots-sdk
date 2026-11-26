# Arm joints and tools

`SET_ARM_JOINTS` latches one position setpoint per joint, in SI units, and each arm adds one
command for its own tool.

```sh
cargo run -p vrobots-examples --bin ex39_rail_arm
./target/cpp-build/ex39_rail_arm
python examples/python/ex39_rail_arm.py
cargo run -p vrobots-examples --bin ex40_table_arm
./target/cpp-build/ex40_table_arm
python examples/python/ex40_table_arm.py
```

The rail arm is pinned at `sys_id` 2 in the warehouse scene and the table arm at `sys_id` 0
in the table-arm scene. Neither base ever moves, so the kinematics block of the state is the
base's, and everything an arm does is in its actuator report.

## The three commands

| Method | Command | Payload | Units | Acted on by |
|---|---|---|---|---|
| `set_arm_joints(setpoints)` | `SET_ARM_JOINTS` (315) | `float_arr`, one per joint | m for the rail, rad for every other joint | both arms |
| `set_arm_vacuum(on)` | `SET_ARM_VACUUM` (316) | `int_val` 1 or 0 | | rail arm; the table arm ignores it |
| `set_arm_gripper(opening_m)` | `SET_ARM_GRIPPER` (320) | `float_val` | m, the gap between the pads | table arm; the rail arm ignores it |

Joint values carry no axes, so nothing is frame-converted.

## Setpoints in actuator order

| Arm | Count | Order |
|---|---|---|
| Rail arm | 7 | the rail in **metres**, then A1 to A6 in **radians** |
| Table arm | 6 | A1 to A6 in **radians** |

The length must match the arm. The simulator drops a wrong-length command whole and says
nothing, so on a handle connected as a rail arm or a table arm the SDK refuses a wrong
length with `InvalidArgument` before anything is published. It also refuses an empty list
and any value that is not finite.

Rust names the indices `rail_arm::RAIL`, `rail_arm::A1` and so on, and `table_arm::A1` to
`table_arm::A6`; Python spells the same names with a dot, as `rail_arm.RAIL`. C++ has
`VRSDK_RAIL_ARM_*` and `VRSDK_TABLE_ARM_*`. The joint ranges and
home poses are on the [rail arm](../ch07-robots/10-rail-arm.md) and
[table arm](../ch07-robots/11-table-arm.md) pages.

From `ex39_rail_arm`, a pose for the rail arm written the way the command takes it:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex39_rail_arm.rs`:

```rust
let pose = [5.0, deg(30.0), deg(-70.0), deg(90.0), 0.0, deg(70.0), 0.0];
println!("\n-- to {} --", joints(&pose));
move_to(&robot, &pose)?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex39_rail_arm.cpp`:

```cpp
const Joints pose = {5.0, deg(30.0), deg(-70.0), deg(90.0), 0.0, deg(70.0), 0.0};
std::printf("\n-- to %s --\n", joints(pose).c_str());
move_to(robot, pose);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex39_rail_arm.py`:

```python
pose = [5.0, deg(30.0), deg(-70.0), deg(90.0), 0.0, deg(70.0), 0.0]
print(f"\n-- to {joints(pose)} --")
move_to(robot, pose)
```

{{#endtab }}
{{#endtabs }}

`deg` converts degrees to the radians the command takes. `Joints` in the C++ program is a
`std::array<double, 7>`, which `set_arm_joints` takes as it is; it also accepts a
`std::vector<double>`, or a pointer and a count. Rust's `set_arm_joints` takes a slice, `&[f64]`, and Python's
takes one sequence or the values as separate arguments. The program prints the pose before it moves:

```text
-- to rail 5.000 m  A1  +30.0  A2  -70.0  A3  +90.0  A4   +0.0  A5  +70.0  A6   +0.0 deg --
```

## A latch with a speed limit per joint

Each joint is clamped to its range and driven toward its setpoint at its own top speed until
a newer command replaces it. There is no acceleration ramp and no coordination between the
joints, so one distant setpoint starts every joint at full speed and they arrive at
different times. For a smooth path, stream intermediate setpoints along your own trajectory.

## Three ways a move ends

Nothing replies, so the actuator report is the only proof. `rail_arm_actuators()` and
`table_arm_actuators()` decode it.

| Outcome | How it reads |
|---|---|
| arrived | every entry of `joints` is within a tolerance of its setpoint |
| at a limit | the setpoint was past the joint's range, so `joints_normalized` sits at -1 or 1 and `joints` never matches what you sent |
| stopped | `contact` is not `NO_CONTACT`, and the arm is held short until a setpoint backs it off |

Values cross the wire as 32-bit floats in both directions, so compare with a tolerance and
never with `==`. The loop in both examples tests the first and third outcomes on every
sample:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex39_rail_arm.rs`:

```rust
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
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex39_rail_arm.cpp`:

```cpp
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
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex39_rail_arm.py`:

```python
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
```

{{#endtab }}
{{#endtabs }}

`contact` is an `ArmContact` enum value in Rust and C++ and the integer code in Python, where
`contact_name` gives the text. A move that completes prints its progress and ends with
`-- arrived`:

```text
   t= 0.01s  rail 4.060 m  A1   +2.0  A2  -61.8  A3  +90.0  A4   +0.0  A5  +62.4  A6   +0.0 deg  worst error 0.9400
   t= 0.23s  rail 4.660 m  A1  +22.0  A2  -70.0  A3  +90.0  A4   +0.0  A5  +70.0  A6   +0.0 deg  worst error 0.3400
   t= 0.35s  rail 5.000 m  A1  +30.0  A2  -70.0  A3  +90.0  A4   +0.0  A5  +70.0  A6   +0.0 deg  -- arrived
```

## The protective stop and the CONTACT reading

Every joint step is tested against the scene before it is taken. A step that would drive the
arm into something is refused: the joints stay where they are and the setpoint stays
latched. Moving away is never refused, so a setpoint that backs off always moves.

| Code | Name | Meaning |
|---|---|---|
| 0 | `no_contact` | the step went through, or nothing moved |
| 1 | `tool_contact` | the vacuum pad or the gripper, or the load it holds, met something: a guarded stop, and the normal end of "descend until contact" |
| 2 | `collision_stop` | any other part of the arm met something: a protective stop |

The code names the last step only. It stays non-zero while the latched setpoint keeps
pushing into the obstacle, and reads `no_contact` again after the first step that goes
through. A collision stop against an object or another robot also publishes a
[safety event](../ch10-events/00-safety-events.md); a tool contact does not.

> **Gotcha.** Only the end pose of each step is tested. At full speed, something thinner than
> one step's travel can be passed through. Approach thin obstacles with small setpoint
> increments.

The table arm also stops before hitting itself, which reads as code 2, or as code 1 when the
gripper or its load is the part that would hit. The thresholds are set through
[`configure_arm`](../ch06-services/10-arm-config.md).

## Vacuum on the rail arm

`set_arm_vacuum(true)` latches the suction on. While on and empty, the pad grips a loose
object once it is within reach of the object's top, square to it, and within the pad's
rating: by default 1 cm, 15 degrees and 400 N. Switching on before the descent is therefore
fine, because the pad grips as it arrives. `set_arm_vacuum(false)` releases the load.

| Decoded field | Meaning |
|---|---|
| `vacuum_on` | your last command, echoed back |
| `holding` | whether a load is on the pad |
| `held_mass_kg` | the held load's mass in kg, 0 when nothing is held |

The joints have no acceleration ramp, so a fast start can tear a heavy load off. A load that
was dropped or was too heavy is then ignored until it leaves the pad's reach or the vacuum
goes off. To retry, switch off, then on.

## Gripper on the table arm

`set_arm_gripper(opening_m)` takes the gap between the pads in **metres**, from 0 (closed) to
0.085 (fully open). Python has the limit as `table_arm.GRIPPER_STROKE_M` and C++ as
`VRSDK_TABLE_ARM_GRIPPER_STROKE_M`.

> **Gotcha.** The simulator clamps the opening instead of refusing it, so a millimetre figure
> such as `40.0` silently opens the gripper fully. Forty millimetres is `0.040`.

Closing on an object stops the pads at its width. The object is held when both pads press it
and it is a loose object within the payload, 5 kg by default. It then rides with the gripper
until you command an opening at least 2 mm wider than the object.

| Decoded field | Meaning |
|---|---|
| `gripper_command_m` | your last command after the clamp, m |
| `gripper_opening_m` | the gap between the pads now, m: the held object's width while holding |
| `holding`, `held_mass_kg` | the HOLD entry: whether a body is held, and its mass |

A finger stopping on an object is not a contact. `contact` speaks only for the joint steps.

## Decode by robot type, never by count

Both arms report nine actuator entries, laid out differently after the joints. The rail arm
has VACUUM then CONTACT at indices 7 and 8. The table arm has GRIPPER, CONTACT and HOLD at
6, 7 and 8. Read as the wrong arm, a report decodes without complaint into wrong numbers,
which is why each typed getter refuses a handle connected as the other type. Reading
[the properties topic](../ch06-services/13-properties.md) once makes that check stronger.

## What reset re-latches

`reset()` lets go of whatever the tool holds, where it is and at rest, switches the vacuum
off or opens the gripper, clears the contact code, and teleports every joint home **with its
setpoint**. A client that keeps publishing setpoints drives the arm away again on the next
step.

**Next:** [Aiming the turret](14-turret.md)

**See also:** [Commands latch](01-latching.md), [Arm configuration](../ch06-services/10-arm-config.md), [Rail arm](../ch07-robots/10-rail-arm.md), [Table arm](../ch07-robots/11-table-arm.md), [Safety events](../ch10-events/00-safety-events.md)
