# Actuator layouts and typed readers

What each index of the actuator arrays is on each robot type, and the readers that decode a report into named fields.

[Actuators](05-actuator.md) describes the three index-parallel arrays and says that
nothing in a snapshot names the device behind an index. This page is where the names come
from: the layout each robot publishes, the layouts you can rely on, and the typed readers
that spare you the indexing.

## The layout is published

Every robot publishes a description of itself on `vrobots/{sys_id}/z/properties`, and
one field of it, `actuator_types`, names the kind of device behind each index. It is
index-parallel to the three arrays, so kind *i* tells you what `measured[i]` is.

| Kind | Value | `normalized` | `measured` |
|---|---|---|---|
| unspecified | 0 | | no kind fits, such as a force in N |
| rotor | 1 | | rotor speed, rad/s |
| servo | 2 | | joint or steering angle, rad |
| wheel | 3 | | wheel speed, rad/s |
| surface | 4 | | surface deflection, rad |
| linear | 5 | | position along a prismatic axis, m |
| vacuum | 6 | commanded on (1) or off (0) | held mass, kg; 0 = nothing |
| laser | 7 | commanded on (1) or off (0) | range to the hit, m; 0 = off or no hit |
| gripper | 8 | commanded opening over the stroke | gap between the pads, m |
| contact | 9 | 0 | an arm's stop code: 0 none, 1 tool, 2 protective stop |
| hold | 10 | 1 while holding | held mass, kg; 0 = nothing |

`properties()` reads the topic once for the robot a handle is connected to, and
`read_properties(sys_id)` reads it for any id with no handle; see
[Robot properties](../ch06-services/13-properties.md#reading-by-sys-id-with-no-handle).
ex44 walks the kinds with the latest snapshot beside them: `p` is the result of
`robot.properties()` and `s` is the state.

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex44_robot_properties.rs`:

```rust
for (i, kind) in p.actuator_types.iter().enumerate() {
    let measured = s.actuator.measured.get(i).copied().unwrap_or(f64::NAN);
    println!(
        "   [{i}] {:<12} {measured:+10.3}  {}",
        kind.name(),
        unit(*kind)
    );
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex44_robot_properties.cpp`:

```cpp
for (std::size_t i = 0; i < p.actuator_types.size(); ++i) {
    const vrsdk::ActuatorKind kind = p.actuator_types[i];
    const double measured = i < s.actuator().measured_count
                                ? s.actuator().measured[i]
                                : std::numeric_limits<double>::quiet_NaN();
    std::printf("   [%zu] %-12s %+10.3f  %s\n", i, vrsdk::to_string(kind), measured,
                unit(kind));
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex44_robot_properties.py`:

```python
values = s.actuator.measured
for i, kind in enumerate(p.actuator_types):
    measured = values[i] if i < len(values) else math.nan
    print(f"   [{i}] {ActuatorKind.name(kind):<12} {measured:+10.3f}  {unit(kind)}")
```

{{#endtab }}
{{#endtabs }}

`unit` is the example's own lookup from a kind to the text in the last column. On the
Warehouse forklift the loop prints four lines:

```text
   [0] wheel            +0.000  rad/s, wheel speed
   [1] servo            +0.000  rad, a joint or steering angle
   [2] linear           -0.215  m, along a prismatic axis
   [3] linear           +0.453  m, along a prismatic axis
```

That is a forklift nobody has commanded yet: parked, wheels straight, the carriage at the
bottom of its travel and the forks 0.453 m apart.

In Rust `actuator_types` is a `Vec<ActuatorKind>` and `ActuatorKind::name()` gives the
string; in C++ it is a `std::vector<vrsdk::ActuatorKind>`; in Python it is a tuple of
integers, with `actuator_type_names` as the same entries in strings. A robot type that
publishes no layout gives an empty one, and a kind from a newer simulator decodes as
`unknown` rather than failing. [Robot properties](../ch06-services/13-properties.md)
covers the rest of the message.

## The layouts of the warehouse robots, the arms and the turret

| Robot | Index | Entry | Kind | `measured` |
|---|---|---|---|---|
| forklift | 0 | drive | wheel | front-axle speed, rad/s, positive forward |
| forklift | 1 | steering | servo | steering angle, rad, positive turning right |
| forklift | 2 | lift | linear | carriage height, m |
| forklift | 3 | fork width | linear | fork spacing, centre to centre, m |
| pallet robot | 0 to 3 | wheels FL, FR, RL, RR | wheel | wheel speed, rad/s, positive forward |
| pallet robot | 4 | pad lift | linear | pad height above rest, m |
| pallet robot | 5 | pad turn | servo | pad angle relative to the body, rad |
| rail arm | 0 | rail | linear | carriage position along the rail, m |
| rail arm | 1 to 6 | A1 to A6 | servo | joint angle, rad |
| rail arm | 7 | VACUUM | vacuum | held mass, kg |
| rail arm | 8 | CONTACT | contact | stop code |
| table arm | 0 to 5 | A1 to A6 | servo | joint angle, rad |
| table arm | 6 | GRIPPER | gripper | gap between the pads, m |
| table arm | 7 | CONTACT | contact | stop code |
| table arm | 8 | HOLD | hold | held mass, kg |
| turret | 0 | heading | servo | pylon angle, rad |
| turret | 1 | pitch | servo | head angle, rad |
| turret | 2 | laser | laser | range to the hit, m |

The forklift's report is not in command order. The command leads with the steer and
the report leads with the drive, so `pwm[0]` echoes the throttle and `pwm[1]` the
steer, and the optional brake has no entry at all. The pallet robot uses one order
for both. The lift height reads negative near the bottom of its travel by design.
Ranges and speeds are on each robot's reference page: the
[forklift](../ch07-robots/08-forklift.md), the
[pallet robot](../ch07-robots/09-palletrobot.md), the
[rail arm](../ch07-robots/10-rail-arm.md), the
[table arm](../ch07-robots/11-table-arm.md) and the
[turret](../ch07-robots/12-turret.md).

> **Gotcha.** Both arms report **nine** entries, and after the joints the entries
> mean different things. Read as a rail arm, a table arm's report decodes without
> any error into wrong numbers: its pad gap as joint A6, its stop code as a held
> mass. Decode by robot type, never by count.

## Named layouts

Each layout in the table above is also a constant, so you compare a published layout
with a named one instead of spelling the kinds out.

| Robot | Rust | C++ | Python |
|---|---|---|---|
| forklift | `forklift_actuator::KINDS` | `vrsdk::forklift_actuator::KINDS` | `vrsdk.forklift_actuator.KINDS` |
| pallet robot | `palletrobot_channel::KINDS` | `vrsdk::palletrobot_channel::KINDS` | `vrsdk.palletrobot_channel.KINDS` |
| rail arm | `rail_arm::ACTUATOR_KINDS` | `vrsdk::rail_arm::ACTUATOR_KINDS` | `vrsdk.rail_arm.ACTUATOR_KINDS` |
| table arm | `table_arm::ACTUATOR_KINDS` | `vrsdk::table_arm::ACTUATOR_KINDS` | `vrsdk.table_arm.ACTUATOR_KINDS` |
| turret | `turret_actuator::KINDS` | `vrsdk::turret_actuator::KINDS` | `vrsdk.turret_actuator.KINDS` |

The comparison is `==` or `!=` against `actuator_types` in all three languages, entry
for entry and of the same length. Rust compares the `Vec<ActuatorKind>` with an array,
the C++ header defines both operators between the published vector and a `std::array`
of kinds, and in Python both sides are tuples. ex44 uses it to decide whether the robot
it read is a forklift:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex44_robot_properties.rs`:

```rust
if p.actuator_types != forklift_actuator::KINDS {
    println!(
        "\nnot a forklift's layout, so there is no srv/forklift to call here; the header \
         lists the other robots' configure_* calls. Nothing was sent."
    );
    return Ok(());
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex44_robot_properties.cpp`:

```cpp
if (p.actuator_types != vrsdk::forklift_actuator::KINDS) {
    std::printf(
        "\nnot a forklift's layout, so there is no srv/forklift to call here; the header "
        "lists the other robots' configure_* calls. Nothing was sent.\n");
    return 0;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex44_robot_properties.py`:

```python
if p.actuator_types != forklift_actuator.KINDS:
    print(
        "\nnot a forklift's layout, so there is no srv/forklift to call here; the header "
        "lists the other robots' configure_* calls. Nothing was sent."
    )
    sys.exit(0)
```

{{#endtab }}
{{#endtabs }}

On any robot that is not a forklift the program prints one line after an empty one, and
ends:

```text
not a forklift's layout, so there is no srv/forklift to call here; the header lists the other robots' configure_* calls. Nothing was sent.
```

The same comparison against the turret's layout is how ex41 finds the turret among the
robots on the wire.

## Typed readers

Each of these robots has a reader that returns the latest report as named fields, so
your code never indexes the arrays. The names are the same in all three languages.

| Robot | Reader | Fields |
|---|---|---|
| forklift | `forklift_actuators()` | `drive_speed`, `steering_angle`, `lift_height`, `fork_width` |
| pallet robot | `palletrobot_actuators()` | `wheel_speeds`, `pad_height`, `pad_angle` |
| rail arm | `rail_arm_actuators()` | `joints`, `joints_normalized`, `vacuum_on`, `holding`, `held_mass_kg`, `contact` |
| table arm | `table_arm_actuators()` | `joints`, `joints_normalized`, `gripper_command_m`, `gripper_opening_m`, `holding`, `held_mass_kg`, `contact` |
| turret | `turret_actuators()` | `heading`, `pitch`, `heading_rate_fraction`, `pitch_rate_fraction`, `laser_on`, `laser_range` |

A reader takes the latest snapshot, so it never blocks. It refuses a handle that was
connected as another robot type with an invalid-argument error
(`VrError::InvalidArgument` in Rust), because the decode is
by index and another robot's report would read as wrong numbers.

The type you pass to `connect` is still only a claim. Once a handle has read
`properties()`, its reader also compares the published layout with its own and fails
with a decode error (`VrError::Decode`) when they contradict, which is the one way to tell the two arms
apart. The rail arm example attaches that way:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex39_rail_arm.rs`:

```rust
fn attach(sys_id: u32) -> Result<VirtualRobot, VrError> {
    let robot = VirtualRobot::connect(RobotType::RailArm, Some(sys_id))?;
    robot.properties()?;
    robot.rail_arm_actuators()?;
    Ok(robot)
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex39_rail_arm.cpp`:

```cpp
vrsdk::VirtualRobot attach(std::uint32_t sys_id) {
    vrsdk::VirtualRobot robot(vrsdk::RobotType::RailArm, sys_id);
    robot.connect();
    (void)robot.properties();
    (void)robot.rail_arm_actuators();
    return robot;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex39_rail_arm.py`, the body of its `attach(sys_id)` function:

```python
robot = VirtualRobot(RobotType.RAILARM, sys_id=sys_id)
robot.connect()
robot.properties()
robot.rail_arm_actuators()
```

{{#endtab }}
{{#endtabs }}

The function prints nothing. Pointed at a table arm, the `rail_arm_actuators()` call
fails instead of handing back a robot whose report decodes into nonsense. Without the
`properties()` call the reader checks the handle's type and the report's length only.

For a snapshot you already hold, each reader has a `from_actuator` counterpart, such
as `RailArmActuators::from_actuator(&state.actuator)` in Rust and
`RailArmActuators.from_actuator(state.actuator)` in Python. It checks the length and
nothing else: Rust and Python return `None` for a report that is too short, and C++
throws.

## Where the shape differs: the Global Hawk

The three-array layout is uniform, but what the indices mean is not, and the fixed-wing
Global Hawk is the case that breaks a naive reading.

| Index | Device | Units in `measured` |
|---|---|---|
| 0 to 5 | control panels | deflection in radians |
| 6 | engine | thrust in newtons |

Code that assumes `measured` is a homogeneous array of rotor speeds produces a thrust
figure in the wrong units at index 6 and silently mixes radians with newtons in any
aggregate. Read the robot type first.

**Next:** [Sending commands](../ch04-commands/00-intro.md)

**See also:** [Actuators](05-actuator.md), [Robot properties](../ch06-services/13-properties.md), [Supported virtual robots](../ch07-robots/00-intro.md), [Global Hawk](../ch07-robots/06-globalhawk.md)
