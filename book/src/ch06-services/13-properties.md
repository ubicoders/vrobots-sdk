# Robot properties

The one topic that says what a robot is: its mass, centre of gravity and inertia, and the
kind of device behind every actuator index.

```sh
cargo run -p vrobots-examples --bin ex44_robot_properties
./target/cpp-build/ex44_robot_properties
python examples/python/ex44_robot_properties.py
```

With no argument the example reads the warehouse forklift at `sys_id` 0. Pass any other id
to read that robot instead.

## A topic, not a service

Properties are not something you request. Every robot publishes them on
`vrobots/{sys_id}/z/properties`, and `properties()` subscribes, takes the first sample and
unsubscribes. They sit in this chapter because they are the nearest thing the simulator has
to reading configuration back.

| Published | When |
|---|---|
| once | when the robot's topics open |
| again | on any parameter change |
| keepalive | at 1 Hz |

Because of the keepalive, a read normally returns within a second. If nothing arrives within
the probe timeout (15 seconds by default) the call fails with a timeout error, which is also
what a simulator too old to publish this topic gives you.

There are two ways to read the topic. `robot.properties()` reads it for a robot you are
already connected to, and the handle keeps the layout it read. `read_properties(sys_id)`
reads it for any id with no handle at all; see
[Reading by sys id, with no handle](#reading-by-sys-id-with-no-handle).

## What a sample carries

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `mass` | float | kg | 0 | the only field that does not depend on a frame |
| `cg` | 3 floats | m | zeros | centre of gravity from the robot origin |
| `inertia` | 3 x 3 floats | kg·m² | zeros | inertia tensor about the centre of gravity, row-major |
| `moment_arms` | list of float | m | empty | each actuator's moment arm from the centre of gravity, index-parallel to the state's actuator arrays; empty for a robot type that publishes none |
| `actuator_types` | list of kinds | | empty | what each actuator entry is, index-parallel to the state's actuator arrays; empty for a robot type that publishes no layout |
| `coord_frame_id` | string | | empty | the frame `cg` and `inertia` are expressed in, which is the robot's own |
| `axis_convention` | enum tag | | unspecified | the same frame as a tag; the string wins when they disagree |

`cg` and `inertia` are on body axes in the robot's own frame, the one its state reports in.
To re-express the tensor in another frame, conjugate it with the rotation between the two
(`R * I * R^T`); the rule for vectors does not apply to a tensor.

The mass properties are read off the simulated body, so a skin swap or a moving mast shows on
the next keepalive. What the sample does **not** carry is how the robot is tuned: no lift
speed, no wheel geometry, no joint range. No configuration service reads anything back.

## Arms publish the whole rig and no inertia

The two arms are posed, not driven by forces, and their sample differs from a rigid
robot's in three ways.

| Arm | `mass` | What it counts | `cg` | `inertia` |
|---|---|---|---|---|
| [Rail arm](../ch07-robots/10-rail-arm.md) | 1551 kg | the whole rig: the track carriage and robot base at 630 kg, plus the arm's links | follows the pose: moving the rail 1 m moves it 1 m | not published |
| [Table arm](../ch07-robots/11-table-arm.md) | 33.6 kg | every body of the arm, gripper included | follows the pose | not published |

The mass is the sum over every body of the rig, so it is larger than the maker's figure for
the robot alone. The centre of gravity is recomputed at the pose of each sample, so read it
beside the joints it belongs to.

An absent tensor reads as zeros in every language, and nothing else marks it:

| Language | `inertia` when none is published |
|---|---|
| Rust | `[[0.0; 3]; 3]` |
| C++ | a `std::array<Vec3, 3>` of zeros |
| Python | three lists of three `0.0` |

A physical tensor is never all zeros, so test for that before you use the field.

## The actuator kinds

[`actuator.pwm`, `normalized` and `measured`](../ch03-reading-state/05-actuator.md) are
index-parallel arrays, and nothing in a state message says what index *i* is. The kind does,
and with it the unit of `measured` at that index.

| Code | Kind | `normalized` | `measured` |
|---|---|---|---|
| 0 | `unspecified` | | no kind fits, such as the force on a mass spring damper or a cart pole |
| 1 | `rotor` | | propeller speed, rad/s |
| 2 | `servo` | | angle of a revolute joint or a steering servo, rad |
| 3 | `wheel` | | speed of a driven wheel, rad/s |
| 4 | `surface` | | deflection of an aerodynamic control surface, rad |
| 5 | `linear` | | position of a prismatic joint (a lift, a fork width, a linear rail), m |
| 6 | `vacuum` | commanded on (1) or off (0) | held mass, kg |
| 7 | `laser` | commanded on (1) or off (0) | range to the hit, m; 0 for none |
| 8 | `gripper` | commanded opening as a fraction of the stroke | gap between the pads, m |
| 9 | `contact` | | an arm's stop report, not a device: a [contact code](../ch04-commands/13-arms.md#the-protective-stop-and-the-contact-reading) |
| 10 | `hold` | 1 while holding | held mass, kg |
| -1 | `unknown` | | a kind newer than your SDK |

Python exposes the codes as `ActuatorKind.ROTOR` and so on, with `ActuatorKind.name(code)`
for the text. C++ has the `vrsdk::ActuatorKind` enum and `vrsdk::to_string(kind)`, and Rust
the `ActuatorKind` enum and `kind.name()`.

## The layout of each robot

| Robot | `actuator_types`, by index |
|---|---|
| Forklift | wheel, servo, linear, linear |
| Pallet robot | wheel, wheel, wheel, wheel, linear, servo |
| Rail arm | linear, servo x 6, vacuum, contact |
| Table arm | servo x 6, gripper, contact, hold |
| Turret | servo, servo, laser |
| Truck | wheel, wheel, wheel, wheel, servo |
| Multirotor, half drone | rotor, one per rotor |
| Mass spring damper, cart pole | unspecified |

## Reading it

From `ex44_robot_properties`, the walk through the layout with the state's measured value beside each kind:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex44_robot_properties.rs`:

```rust
// ===== the actuator layout, index by index =====
// Index-parallel to the state's actuator arrays: kind i is what measured[i] is.
println!(
    "\n{} actuator entries; the state's measured value beside each:",
    p.actuator_types.len()
);
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
// ===== the actuator layout, index by index =====
// Index-parallel to the state's actuator arrays: kind i is what measured[i] is.
std::printf("\n%zu actuator entries; the state's measured value beside each:\n",
            p.actuator_types.size());
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
# ===== the actuator layout, index by index =====
# Index-parallel to the state's actuator arrays: kind i is what measured[i] is.
print(f"\n{len(p.actuator_types)} actuator entries; the state's measured value beside each:")
values = s.actuator.measured
for i, kind in enumerate(p.actuator_types):
    measured = values[i] if i < len(values) else math.nan
    print(f"   [{i}] {ActuatorKind.name(kind):<12} {measured:+10.3f}  {unit(kind)}")
```

{{#endtab }}
{{#endtabs }}

`p` is the result of `robot.properties()` and `s` is a state snapshot taken beside it. In
Python `actuator_types` is a tuple of integer codes, with `actuator_type_names` as the same
entries in text. In Rust it is a `Vec<ActuatorKind>` and in C++ a
`std::vector<vrsdk::ActuatorKind>`, and in both the struct also carries `truncated`, which is true when the robot published more entries than the fixed-size C
record holds. `unit` is a helper in the example that maps a kind to the text in the last
column. For the forklift the walk prints:

```text
sys_id 0 'forklift': 4000.0 kg, centre of gravity (-1.222, +0.000, +0.600) m, in "fru"
inertia about the centre of gravity, kg*m^2:
   [      2849.7         -0.1       -574.8]
   [        -0.1       4963.8         -0.1]
   [      -574.8         -0.1       3415.0]
moment arms: none published for this type

4 actuator entries; the state's measured value beside each:
   [0] wheel            +0.000  rad/s, wheel speed
   [1] servo            +0.000  rad, a joint or steering angle
   [2] linear           -0.215  m, along a prismatic axis
   [3] linear           +0.453  m, along a prismatic axis
```

Python prints the frame id in single quotes. The forklift publishes no moment arms, so the
run prints `moment arms: none published for this type`; a robot type that publishes them
prints the list in metres on that line instead.

## The layout is the type check

`connect` takes the robot type on trust. Nothing in a state message names the type, so a
truck at the id you thought was a forklift attaches without complaint, and the count of
actuator entries does not settle it either: a cart pole and a turret both report three, and
both arms report nine.

The layout is the robot's own word. Once a handle has called `properties()`, it keeps the
layout it read, and from then on the typed getters (`forklift_actuators()`,
`palletrobot_actuators()`, `rail_arm_actuators()`, `table_arm_actuators()`,
`turret_actuators()`) refuse a report whose published layout contradicts theirs, with a
decode error, instead of decoding it into wrong numbers. Without a read they decode by index
as before. The examples for the five robots all open with that pair of calls:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex37_hello_forklift.rs`:

```rust
fn attach(sys_id: u32) -> Result<VirtualRobot, VrError> {
    let robot = VirtualRobot::connect(RobotType::Forklift, Some(sys_id))?;
    robot.properties()?;
    robot.forklift_actuators()?;
    Ok(robot)
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex37_hello_forklift.cpp`:

```cpp
vrsdk::VirtualRobot attach(std::uint32_t sys_id) {
    vrsdk::VirtualRobot robot(vrsdk::RobotType::Forklift, sys_id);
    robot.connect();
    (void)robot.properties();
    (void)robot.forklift_actuators();
    return robot;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex37_hello_forklift.py`:

```python
robot = VirtualRobot(RobotType.FORKLIFT, sys_id=sys_id)
robot.connect()
robot.properties()
robot.forklift_actuators()
return robot
```

{{#endtab }}
{{#endtabs }}

None of the listings prints anything. On a robot that is not a forklift the last call fails, and
the program stops before it has sent a command.

A kind your SDK does not know never counts as a contradiction, so a newer simulator that
names a device more precisely does not break the check.

## Reading by sys id, with no handle

`read_properties` reads the same topic for a robot you have not connected to. It is the
cheap way to learn what is at an id, and so the way to find a robot by type:
[`list_topics`](../ch08-tooling/02-discovery-from-code.md) says which ids are on the wire,
and this call reads each one's layout.

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`crates/vrobots-sdk/src/properties.rs`:

```rust
pub fn read_properties(sys_id: u32) -> VrResult<RobotProperties>
pub fn read_properties_with(sys_id: u32, options: &ConnectOptions) -> VrResult<RobotProperties>
```

{{#endtab }}
{{#tab name="C++" }}

`include/vrobots_sdk.hpp`:

```cpp
[[nodiscard]] inline RobotProperties read_properties(
    std::uint32_t sys_id, const vrsdk_connect_options_t* options = nullptr)
```

{{#endtab }}
{{#tab name="Python" }}

`vrsdk/_vrsdk.pyi`:

```python
def read_properties(
    sys_id: int,
    timeout: Optional[float] = None,
    router: Optional[str] = None,
    connect_timeout: Optional[float] = None,
) -> RobotProperties: ...
```

{{#endtab }}
{{#endtabs }}

The call opens a zenoh session of its own, takes one sample and closes the session before
it returns. A fresh session needs a moment of discovery and the topic is a 1 Hz keepalive,
so a read takes about a second. Of the options, only the connection settings and the probe
timeout are used, and Python takes them as keyword arguments: `timeout` is the wait for a
sample in seconds, 15 by default. With no robot at the id the call fails with a timeout
error once that wait has passed.

Nothing is kept. A handle you connect afterwards has read no layout, so its typed getters
check nothing until that handle calls `properties()` itself.

`ex41_hello_turret` finds the sandbox turret this way, whose id is not pinned.
`candidates` holds the ids that published a state topic:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex41_hello_turret.rs`:

```rust
// Step 2: the layout on z/properties is what a robot says it is, and
// reading it takes no handle.
for sys_id in candidates {
    let p = match read_properties(sys_id) {
        Ok(p) => p,
        Err(e) => {
            println!("   sys_id {sys_id}: no layout: [{}] {e}", e.code());
            continue;
        }
    };
    let kinds: Vec<&str> = p.actuator_types.iter().map(|k| k.name()).collect();
    if p.actuator_types == turret_actuator::KINDS {
        println!("   sys_id {sys_id}: {kinds:?} -- the turret");
        return attach(sys_id).map(Some);
    }
    println!("   sys_id {sys_id}: {kinds:?} -- not a turret");
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex41_hello_turret.cpp`:

```cpp
// Step 2: the layout on z/properties is what a robot says it is, and
// reading it takes no handle.
for (const std::uint32_t sys_id : candidates) {
    vrsdk::RobotProperties p;
    try {
        p = vrsdk::read_properties(sys_id);
    } catch (const vrsdk::Error& e) {
        std::printf("   sys_id %u: no layout: [%d] %s\n", sys_id, e.code(), e.what());
        continue;
    }
    const std::string kinds = kinds_of(p.actuator_types);
    if (p.actuator_types == vrsdk::turret_actuator::KINDS) {
        std::printf("   sys_id %u: %s -- the turret\n", sys_id, kinds.c_str());
        return attach(sys_id);
    }
    std::printf("   sys_id %u: %s -- not a turret\n", sys_id, kinds.c_str());
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex41_hello_turret.py`:

```python
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
```

{{#endtab }}
{{#endtabs }}

`attach` is the example's own function: the connect, `properties()` and
`turret_actuators()` sequence, as in the forklift listing above. In the sandbox scene, with
six robots on the wire, the Rust program prints:

```text
listening for 1.5s for the robots on the wire ...
robots on the wire: [3, 4, 5, 6, 7, 8]
   sys_id 3: ["rotor", "rotor"] -- not a turret
   sys_id 4: ["unspecified", "unspecified"] -- not a turret
   sys_id 5: ["rotor", "rotor", "rotor", "rotor"] -- not a turret
   sys_id 6: ["unspecified", "unspecified", "unspecified"] -- not a turret
   sys_id 7: ["wheel", "wheel", "wheel", "wheel", "servo"] -- not a turret
   sys_id 8: ["servo", "servo", "laser"] -- the turret
```

C++ prints the same text, and Python prints the names in single quotes. An id whose layout
cannot be read prints `no layout` with the error, and the search moves on.
[Actuator layouts and typed readers](../ch03-reading-state/10-actuator-layouts.md#named-layouts)
lists the named layouts to compare against.

**Next:** [Supported virtual robots](../ch07-robots/00-intro.md)

**See also:** [Actuators](../ch03-reading-state/05-actuator.md), [Mass and inertia](02-physical-params.md), [The topic namespace](../ch02-concepts/02-topics.md), [Discovery from code](../ch08-tooling/02-discovery-from-code.md)
