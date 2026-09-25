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

`ex21_reset` takes an **optional** `sys_id`. With it, the example attaches to the scene's own
multirotor; without it, the example creates one. Prefer the argument: a client-created
multirotor does not integrate physics in simulator v3.0.0, which is
[a known issue](../ch07-robots/07-known-issues.md).

## Five verbs, five different confirmations

| Verb | How you issue it | Confirmed by |
|---|---|---|
| create | `connect(type, None)` | the new robot's state topic starting to publish |
| attach | `connect(type, Some(id))` | the first state snapshot, which `connect` blocks for |
| activate | `ConnectOptions::activate_after_create`, inside `connect` | the ack, and then the state topic |
| reset | `reset()` | the position in the state stream, one step later |
| delete | `delete()` | the state topic going silent for one second |

There is no public `activate()` method. Activation happens as step 3 of the create sequence
when `activate_after_create` is true, which it is by default. Attaching never activates,
because the scene already did.

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
# Create a NEW robot in the sim (no sys_id -> manager create; the reply
# carries the assigned id).
robot = VirtualRobot(ROBOT_TYPE)
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
# Deletion is explicit and never implicit. delete() waits for the state topic
# to fall silent: the manager's ack is only a receipt, absence is the proof.
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

`examples/python/ex04_hello_service.py`:

```python
try:
    robot.set_mr_pwm(1500, 1500, 1500, 1500)
    print("unexpected: a deleted robot accepted a command")
except vrsdk.VrError as e:
    print(f"the handle is spent, as expected: [{e.code} {e.kind}] {e.detail}")
```

{{#endtab }}
{{#endtabs }}

Rust returns the refusal as a `Result` you match on, while C++ throws `vrsdk::Error` and
Python raises `vrsdk.VrError`, so both need the call inside a `try`. Python also accepts the
four pulse widths as separate arguments rather than one array.

```text
the handle is spent, as expected: [<code>] <message>
```

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
print("\n-- reset() (a bare GET) --")
robot.reset()
print(
    "acked. That is a RECEIPT: the teleport lands in phase 0 of the next "
    "physics step, and the state stream is the proof."
)
```

{{#endtab }}
{{#endtabs }}

`reset()` takes no arguments and returns nothing in any of the three, so the only difference
is the snapshot beside it: `robot.states` is a property in Python where Rust and C++ call
`states()`.

The example stops commanding across the reset, so the effect is visible: with nothing sent,
the actuator echo falls from the climb pulse back to the robot's initial 1100 us idle, and the
distance from home collapses to roughly zero.

```text
-- reset() (a bare GET) --
acked. That is a RECEIPT: the teleport lands in phase 0 of the next physics step, and the state stream is the proof.
home? seq=<n> t=<seconds>s pos=(...) [frd] alt=<metres> |v|=<speed> echo=[...]  d(home)=<metres> m
```

A live publisher wins one step later. A control loop that keeps sending 1700 us climbs
straight back out of the reset and barely registers that it happened, which is exactly what
you want when the loop under test is the thing you are resetting around.

> **Gotcha.** `srv/reset` is the one service key where a payload-less GET **performs the
> action** instead of probing it. The simulator's vendored C# zenoh client cannot attach a
> payload, so an empty query had to mean something. Probe every other key freely; never probe
> this one. Because twice home is still home, `reset()` is idempotent and its discovery-retry
> loop is safe.

## Home is not where you found it

Home is the pose at the robot's first physics step, not the pose you attached at. A scene
robot that has been flying since the scene loaded can be a long way from it: measured live,
one attach found a multirotor 27 m from its home, and a program that treated the attach
position as home reported the reset as having moved the robot away from where it belonged.

Nothing reads the home pose out, so the only honest way to learn it is to go there.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
fn learn_home(robot: &VirtualRobot, created: bool) -> Result<Arc<State>, VrError> {
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
def learn_home(robot: VirtualRobot, created: bool):
    if created:
        return robot.states

    print("attached: resetting once to find out where home actually is")
    robot.reset()
    for _ in range(SETTLE_SAMPLES):
        robot.rate(HZ)
    return robot.states
```

{{#endtab }}
{{#endtabs }}

The snapshot each one hands back differs in ownership, not in content: Rust returns an
`Arc<State>`, C++ returns a `vrsdk::State` by value, and Python returns whatever the property
yields.

On the create path this is unnecessary, because nothing has happened to the robot yet and the
first sample already is home. The same trick finds a cart pole's rail centre, which is the one
number that plant needs and does not publish.

## What survives a reset

| Set by | Survives a reset? |
|---|---|
| `set_physical_params` (mass, inertia) | yes |
| `configure_sensors` (noise models) | yes |
| `configure_rotors`, `configure_drive`, `configure_msd`, `configure_cartpole` | yes |
| `set_frames`, `set_skin` | yes |
| position, orientation, velocity | no, teleported home |
| the latched command and the actuator echo | no, re-latched to the initial command |
| fixed-wing control mode and estimate source | no, reverted to onboard and truth |
| `seq` and `elapsed` | untouched, the clock never restarts |

A frozen `elapsed` means the simulator stopped, never that something reset.

Configuration surviving is what makes attaching to a scene robot a one-way door. There is no
getter for mass, inertia or rotor geometry, so the SDK cannot read the old value and put it
back, and `reset()` will not do it for you. Whatever you configure on a shared robot stays
configured for every other client until the scene is reloaded.

**Next:** [Mass and inertia](02-physical-params.md)

**See also:** [What connect actually does](../ch02-concepts/05-connect.md), [System ids, and the two kinds of robot](../ch02-concepts/03-sys-id.md), [Known simulator issues](../ch07-robots/07-known-issues.md)
