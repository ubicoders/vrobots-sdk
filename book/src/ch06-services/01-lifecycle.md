# Robot lifecycle

Create, attach, activate, reset and delete, and what each is confirmed by.

```sh
cargo run -p vrobots-examples --bin ex04_hello_service
./target/cpp-build/ex04_hello_service
python examples/python/ex04_hello_service.py
cargo run -p vrobots-examples --bin ex21_reset -- 1
./target/cpp-build/ex21_reset 1
python examples/python/ex21_reset.py 1
cargo run -p vrobots-examples --bin ex21_reset
./target/cpp-build/ex21_reset
python examples/python/ex21_reset.py
```

`ex21_reset` takes an **optional** `sys_id` in all three languages. With it, the example
attaches to the scene's own multirotor and leaves it running; without it, the example creates
one and deletes it at the end. Prefer the argument: a client-created multirotor does not
integrate physics in simulator v3.0.0, which is
[a known issue](../ch07-robots/07-known-issues.md).

## Five verbs, five different confirmations

| Verb | How you issue it | Confirmed by |
|---|---|---|
| create | `connect(type, None)` | the new robot's state topic starting to publish |
| attach | `connect(type, Some(id))` | the first state snapshot, which `connect` blocks for |
| activate | inside `connect` when `ConnectOptions::activate_after_create` is set, or `activate()` | the ack, and then the state topic |
| reset | `reset()` | the position in the state stream, one step later |
| delete | `delete()` | the state topic going silent for one second |

Activation happens as step 3 of the create sequence when `activate_after_create` is true,
which it is by default, so most programs never call it. Attaching never activates, because
the scene already did. `activate()` is also a public method in all three languages, for the
one program that needs it: create a robot with `start_active` and `activate_after_create`
both false, configure it while it is held, then call `activate()` to start its dynamics.
The call is idempotent, and an already-active robot acks and does nothing.

## Create and delete

`connect(type, None)` asks the manager to spawn a robot and the reply carries its
[`sys_id`](../appendix-d-glossary.md). Create is the only non-idempotent service in the
system, so the SDK sends it exactly once and never retries: a retry that lands spawns a second
robot.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex04_hello_service.rs`:

```rust
    // Create a NEW robot in the sim (no sys_id -> manager create; reply carries the id).
    let robot = VirtualRobot::connect(ROBOT_TYPE, None)?;
    let sys_id = robot.sys_id();
    println!("created sys_id = {sys_id}");
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex04_hello_service.cpp`:

```cpp
// Create a NEW robot in the sim: `create` means "no sys_id", so the
// manager assigns one and the reply carries it. (A constructor would be
// ambiguous with the attach form -- see the header.)
vrsdk::VirtualRobot robot = vrsdk::VirtualRobot::create(vrsdk::RobotType::Multirotor);
robot.connect();
const std::uint32_t sys_id = robot.sys_id();
std::printf("created sys_id = %u\n", sys_id);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex04_hello_service.py`:

```python
# No sys_id: the manager creates a robot and assigns one.
robot = VirtualRobot(RobotType.MULTIROTOR)
robot.connect()
sys_id = robot.sys_id
print(f"created sys_id = {sys_id}")
```

{{#endtab }}
{{#endtabs }}

C++ and Python build the handle first and call `connect()` on it, where Rust's
`VirtualRobot::connect` does both in one call. C++ spells the create form
`VirtualRobot::create(type)` because a one-argument constructor would be ambiguous with the
attach form, and Python reads `sys_id` as a property rather than a method.

By the time `connect` returns, the robot's state topic has published at least once, so the
snapshot the next lines read is real data rather than a placeholder:

```text
created sys_id = <id>
first state: t=<seconds> seq=<n> name=<robot name>
its state topic: vrobots/<id>/z/state
```

Deletion is explicit and never implicit. Dropping a `VirtualRobot` closes the session and
leaves the robot running, which is the point of the fourth rule: robots outlive the process.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    // Deletion is explicit and never implicit. delete() waits for the state topic
    // to fall silent: the manager's ack is only a receipt, absence is the proof.
    robot.delete()?;
    println!(
        "deleted sys_id = {sys_id} (is_deleted={})",
        robot.is_deleted()
    );
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex04_hello_service.cpp`:

```cpp
// Deletion is explicit and never implicit. The manager's ack is only a
// receipt, so remove() also waits for the robot's state topic to fall
// silent -- that is the real confirmation.
robot.remove();
std::printf("deleted sys_id = %u (removed=%s)\n", sys_id,
            robot.removed() ? "true" : "false");
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex04_hello_service.py`:

```python
# Robots outlive the process; only delete() removes one from the scene.
robot.delete()
print(f"deleted sys_id = {sys_id} (is_deleted={robot.is_deleted})")
```

{{#endtab }}
{{#endtabs }}

Only the names differ. C++ spells the pair `remove()` and `removed()` because `delete` is a
keyword, and Python's `is_deleted` is a property where Rust's is a method.

The call returns once the state topic has been quiet for one second, which at 25 Hz is 25
missing samples:

```text
deleted sys_id = <id> (is_deleted=true)
```

C++ prints `(removed=true)` and Python `(is_deleted=True)`. The Python script ends here.

> **Note.** `delete()` is the one service the SDK deliberately does not retry. A re-send after
> a delete the manager already applied comes back as `ok = false` for an unknown `sys_id`,
> which would turn a successful delete into a reported failure.

After that the handle is spent. Every command and every service on it fails with
`VrError::Deleted` rather than doing nothing quietly.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    match robot.set_mr_pwm([1500.0; 4]) {
        Ok(()) => println!("unexpected: a deleted robot accepted a command"),
        Err(e) => println!("the handle is spent, as expected: [{}] {e}", e.code()),
    }
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex04_hello_service.cpp`:

```cpp
try {
    robot.set_mr_pwm({1500.0, 1500.0, 1500.0, 1500.0});
    std::printf("unexpected: a deleted robot accepted a command\n");
} catch (const vrsdk::Error& e) {
    std::printf("the handle is spent, as expected: [%d] %s\n", e.code(), e.what());
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex04_hello_service.py` stops at the delete and does not make this call. The
same call on a spent handle raises `vrsdk.VrError` with `code == vrsdk.err.DELETED`.

{{#endtab }}
{{#endtabs }}

Rust returns the refusal as a `Result` you match on, while C++ throws `vrsdk::Error`, so the
C++ call sits inside a `try`.

```text
the handle is spent, as expected: [<code>] <message>
```

## What can be created

Create is not available for every robot type, and it is not available on every plan.

| Catalog key | Robot | Python | Rust and C++ |
|---|---|---|---|
| `multirotor` | Multirotor | `RobotType.MULTIROTOR` | `RobotType::Multirotor` |
| `truck` | Truck | `RobotType.TRUCK` | `RobotType::Truck` |
| `msd` | Mass spring damper | `RobotType.MSD` | `RobotType::Msd` |
| `forklift` | Forklift | `RobotType.FORKLIFT` | `RobotType::Forklift` |
| `palletrobot` | Pallet robot | `RobotType.PALLETROBOT` | `RobotType::PalletRobot` |
| `kr210_rail` | Rail arm | `RobotType.RAILARM` | `RobotType::RailArm` |
| `tablearm` | Table arm | `RobotType.TABLEARM` | `RobotType::TableArm` |
| `turret` | Turret | `RobotType.TURRET` | `RobotType::Turret` |

Those eight keys are the spawn catalog. The manager matches a key exactly, case included, and
the SDK sends the right one for the robot type you name, so you never type a key yourself.
The cart pole, the half drone and the Global Hawk are not in the catalog: they exist only as
scene-authored robots, and you attach to them by `sys_id`.

The catalog belongs to the scene, not to the SDK. A scene that does not offer a type refuses
the create with `ok = false` and a message that lists the keys it does know, which the SDK
carries word for word inside the [`VrError::Service`](../appendix-c-errors.md) it returns.

> **Gotcha.** Create needs the Pro plan. On the Community plan the manager refuses every
> create, whatever the type, and the refusal arrives as `VrError::Service` with the
> simulator's reason. Attaching to a robot the scene already contains never touches the
> create service, so it works on either plan.

A robot created without a `robot_name` is named after its catalog key and its id:
`forklift_11`, `kr210_rail_13`, `turret_15`. A created table arm starts with every joint at
zero, which is not the pose the table-arm scene's own arm loads in, and it should be created
with `robot_name` set to `tablearm`, the name the scene's own arm carries; the simulator
accepts that name on a create.
<!-- VERIFY: what depends on a created table arm keeping the name "tablearm", and what happens when robot_name is set to something else. -->

## Where a created robot appears

Without a pose, a created robot spawns at the world origin, unrotated, so two robots created
that way overlap. Give the connection an initial pose to place it.

The pose is in **raw Unity world coordinates**, whatever frame your connection names. The
simulator applies it exactly as sent and converts nothing.

| Part | Type | Units | Default | Notes |
|---|---|---|---|---|
| position | 3 floats | m | the world origin | left-handed world: +x right, +y up, +z forward |
| orientation | quaternion `[x, y, z, w]` | | unrotated | normalised before sending; a heading of `yaw` radians about the up axis is `[0, sin(yaw / 2), 0, cos(yaw / 2)]`, and a positive `yaw` turns +z toward +x, clockwise seen from above |

No example program sets a spawn pose. The call, placing a forklift 4 m to the right of the
origin and a quarter turn clockwise seen from above:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

Written against the `vrobots-sdk` crate:

```rust
let half = 0.5 * yaw; // yaw = std::f64::consts::FRAC_PI_2
let options = ConnectOptions::default()
    .with_initial_pose(SpawnPose::new([4.0, 0.0, 0.0], [0.0, half.sin(), 0.0, half.cos()]));
let robot = VirtualRobot::connect_with(RobotType::Forklift, None, options)?;
```

{{#endtab }}
{{#tab name="C++" }}

Written from `include/vrobots_sdk.hpp`:

```cpp
vrsdk_connect_options_t options{};
vrsdk_options_default(&options);
const double half = 0.5 * yaw;  // yaw = pi / 2
vrsdk::set_initial_pose(options, {4.0, 0.0, 0.0},
                        {0.0, std::sin(half), 0.0, std::cos(half)});
auto robot = vrsdk::VirtualRobot::create(vrsdk::RobotType::Forklift, &options);
robot.connect();
```

{{#endtab }}
{{#tab name="Python" }}

Written from `vrsdk/_vrsdk.pyi`:

```python
half = 0.5 * yaw  # yaw = math.pi / 2
robot = VirtualRobot(
    RobotType.FORKLIFT,
    initial_position=[4.0, 0.0, 0.0],
    initial_orientation=[0.0, math.sin(half), 0.0, math.cos(half)],
)
robot.connect()
```

{{#endtab }}
{{#endtabs }}

None of the listings prints anything. The orientation is optional in all three languages
and defaults to unrotated; in Rust, `SpawnPose::at(position)` is the form without it.

The spawn pose is also the robot's home, the pose `reset()` returns it to. It is ignored when
you attach to an existing `sys_id`. A value that is not finite, or a zero-length rotation, is
refused by `connect` with `InvalidArgument` before anything is sent.

Checked against a live simulator, a forklift, a pallet robot, a rail arm and a table arm each
came up within 1 mm of the position asked for, and each was deleted cleanly afterwards. A
turret came to rest about 0.08 m above it, where its base seats on the floor. The turret is
also the one type whose `kin.quat` does not show the spawn rotation: its attitude is its
joints, so a turret created a quarter turn round still reports the identity quaternion at
rest.

> **Gotcha.** The spawn pose is the one place where you write Unity axes whatever your
> connection's frame is. The state stream reports in the robot's own frame, so the position
> you read back is the same place on different axes. Read `coord_frame_id` off the snapshot
> before comparing the two; [Frames, axes and units](../ch02-concepts/07-frames-and-units.md)
> has the mapping.

## Reset

`reset()` teleports the robot to the pose captured at its first physics step, zeroes linear
and angular velocity, rests the actuators and re-latches the robot's initial command. It is
what the simulator's own Reset button does.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex21_reset.rs`:

```rust
    let before = robot.states();
    println!("\n-- reset() (a bare GET) --");
    robot.reset()?;
    println!(
        "acked. That is a RECEIPT: the teleport lands in phase 0 of the next \
         physics step, and the state stream is the proof."
    );
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex21_reset.cpp`:

```cpp
const vrsdk::State before = robot.states();
std::printf("\n-- reset() (a bare GET) --\n");
robot.reset();
std::printf(
    "acked. That is a RECEIPT: the teleport lands in phase 0 of the next physics step, "
    "and the state stream is the proof.\n");
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex21_reset.py`:

```python
before = robot.states
robot.reset()  # a bare GET; the teleport lands in phase 0 of the next physics step
for _ in range(50):
    robot.rate(HZ)
after = robot.states
```

{{#endtab }}
{{#endtabs }}

`reset()` takes no arguments and returns nothing in any of the three, so the only difference
is the snapshot beside it: `robot.states` is a property in Python where Rust and C++ call
`states()`. Rust and C++ print the receipt as a line of output; the Python script keeps it as
the comment on the call and goes straight to the 50 settling samples.

The example stops commanding across the reset, so the effect is visible: with nothing sent,
the actuator echo falls from the climb pulse back to the robot's initial 1100 us idle, and the
distance from home collapses to roughly zero.

```text
-- reset() (a bare GET) --
acked. That is a RECEIPT: the teleport lands in phase 0 of the next physics step, and the state stream is the proof.
home? seq=<n> t=<seconds>s pos=(...) [frd] alt=<metres> |v|=<speed> echo=[...]  d(home)=<metres> m
```

The Python script prints no per-sample lines. It ends with the same three before-and-after
comparisons the Rust and C++ runs end with, in a shorter form, followed by
`deleted sys_id=<id>` when it created the robot:

```text
position:  <metres> m from home before, <metres> m after
actuators: echo [...] -> [...]
time:      seq <n> -> <n>, elapsed <seconds>s -> <seconds>s
```

A live publisher wins one step later. A control loop that keeps sending 1700 us climbs
straight back out of the reset and barely registers that it happened, which is exactly what
you want when the loop under test is the thing you are resetting around.

> **Gotcha.** `srv/reset` is one of two service keys where a payload-less GET **performs the
> action** instead of probing it. The simulator's vendored C# zenoh client cannot attach a
> payload, so an empty query had to mean something. The other is `srv/activate`, where a bare
> GET releases a dormant robot's dynamics and does nothing to a robot already running. Probe
> every other key freely; never probe `srv/reset`. Because twice home is still home, `reset()`
> is idempotent and its discovery-retry loop is safe.

## Home is not where you found it

Home is the pose at the robot's first physics step, not the pose you attached at. A scene
robot that has been flying since the scene loaded can be a long way from it: measured live,
one attach found a multirotor 27 m from its home, and a program that treated the attach
position as home reported the reset as having moved the robot away from where it belonged.

Nothing reads the home pose out, so the only honest way to learn it is to go there.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex21_reset.rs`:

```rust
fn learn_home(robot: &VirtualRobot, created: bool) -> Result<State, VrError> {
    if created {
        return Ok(robot.states());
    }

    println!("attached: resetting once to find out where home actually is");
    robot.reset()?;
    for _ in 0..SETTLE_SAMPLES {
        robot.rate(HZ);
    }
    Ok(robot.states())
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex21_reset.cpp`:

```cpp
vrsdk::State learn_home(vrsdk::VirtualRobot& robot, bool created) {
    if (created) {
        return robot.states();
    }
    std::printf("attached: resetting once to find out where home actually is\n");
    robot.reset();
    for (int i = 0; i < SETTLE_SAMPLES; ++i) {
        robot.rate(HZ);
    }
    return robot.states();
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex21_reset.py`:

```python
# Home is the pose captured at the robot's first physics step, not where you found
# it: reset once, settle, and read it off the state stream.
robot.reset()
for _ in range(50):
    robot.rate(HZ)
home = robot.states
print(f"home pos={home.kin.lin_pos}")
```

{{#endtab }}
{{#endtabs }}

Rust and C++ wrap the step in a `learn_home` function that skips it on the create path,
because nothing has happened to a new robot yet and its first sample already is home. Both
hand the snapshot back by value, an owned `State` in Rust and a `vrsdk::State` in C++. The
Python script runs the same three steps inline on both paths, which costs nothing on a
created robot, since twice home is still home. The same trick finds a cart pole's rail
centre, which is the one number that plant needs and does not publish.

## What survives a reset

| Set by | Survives a reset? |
|---|---|
| `set_physical_params` (mass, inertia) | yes |
| `configure_sensors` (noise models) | yes |
| `configure_rotors`, `configure_drive`, `configure_msd`, `configure_cartpole` | yes |
| `configure_forklift`, `configure_palletrobot`, `configure_arm`, `configure_turret`, `configure_lidars` | yes |
| `set_frames`, `set_skin` | yes |
| position, orientation, velocity | no, teleported home |
| the latched command and the actuator echo | no, re-latched to the initial command |
| fixed-wing control mode and estimate source | no, reverted to onboard and truth |
| an arm's joints, its held load and its tool | no, joints home with their setpoints, the load released, vacuum off or gripper open |
| a turret's pose and laser | no, both joints to 0 or the nearest limit, laser off |
| `seq` and `elapsed` | untouched, the clock never restarts |

A frozen `elapsed` means the simulator stopped, never that something reset.

Configuration surviving is what makes attaching to a scene robot a one-way door. There is no
getter for mass, inertia or rotor geometry, so the SDK cannot read the old value and put it
back, and `reset()` will not do it for you. Whatever you configure on a shared robot stays
configured for every other client until the scene is reloaded.

**Next:** [Mass and inertia](02-physical-params.md)

**See also:** [What connect actually does](../ch02-concepts/05-connect.md), [System ids, and the two kinds of robot](../ch02-concepts/03-sys-id.md), [Known simulator issues](../ch07-robots/07-known-issues.md)
