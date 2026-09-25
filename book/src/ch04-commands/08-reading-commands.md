# Reading someone else's commands

A robot's command topic is a shared bus, so you can subscribe to what other publishers are
sending it and use that as your own controller's input.

```sh
cargo run -p vrobots-examples --bin ex32_fw_rate_controller -- <sys_id>
./target/cpp-build/ex32_fw_rate_controller <sys_id>
python examples/python/ex32_fw_rate_controller.py <sys_id>
```

## The one place the SDK reads z/cmd

Everywhere else a command is write-only, and rightly so: it has no reply and the state
stream is the proof. `subscribe_setpoint` is the exception. Zenoh's `vrobots/{sys_id}/z/cmd`
is many-to-many, so the setpoints the simulator's in-game IMU panel publishes at 50 Hz are
readable by anyone who subscribes to the same key.

That enables the experiment the fixed wing was built for. From
`examples/rust/src/bin/ex32_fw_rate_controller.rs`, three streams meet in one program:

```text
z/cmd    ->  SET_ANGVEL from the sim's IMU panel   the setpoint (read, not written)
z/state  ->  kin.ang_vel                           the measurement
z/cmd    <-  SET_FW_SURFACES + SET_FW_THRUST       your output
```

The operator keeps flying with the stick, the simulator's rate PIDs are bypassed, and your
gains are in their place.

## The two subscriptions

| Method | Signature | Notes |
|---|---|---|
| `subscribe_setpoint` | `(&self) -> VrResult<SetpointStream>` | shorthand for `subscribe_command(cmd::SET_ANGVEL)` |
| `subscribe_command` | `(&self, cmd_id: u32) -> VrResult<SetpointStream>` | any id; only commands carrying a `vec3` yield a `Setpoint`, and the rest are counted as filtered |

Both return `VrError::Deleted` if the robot was deleted and `VrError::Session` if zenoh will
not declare the subscriber. Subscribe before you take the aircraft, so an input during the
handover is not missed:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    // Subscribe BEFORE taking the aircraft: a stick input during the handover
    // would otherwise be missed, and the loop would start from "no setpoint".
    let setpoints = robot.subscribe_setpoint()?;
    println!(
        "attached to sys_id={} ({:?}); watching {} for {} (id {}), ignoring src_id={own_src_id}",
        robot.sys_id(),
        robot.robot_type(),
        setpoints.key(),
        cmd::name(setpoints.cmd_id()),
        setpoints.cmd_id()
    );
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex32_fw_rate_controller.cpp`:

```cpp
// Subscribe BEFORE taking the aircraft: a stick input during the
// handover would otherwise be missed, and the loop would start from "no
// setpoint".
vrsdk::SetpointStream setpoints = robot.subscribe_setpoint();
std::printf(
    "attached to sys_id=%u (GlobalHawk); watching %s for SET_ANGVEL (id %u), ignoring "
    "src_id=%u\n",
    robot.sys_id(), setpoints.key().c_str(), setpoints.cmd_id(), OWN_SRC_ID);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex32_fw_rate_controller.py`:

```python
own_src_id = robot.options["src_id"]

# Subscribe BEFORE taking the aircraft: a stick input during the handover
# would otherwise be missed, and the loop would start from "no setpoint".
setpoints = robot.subscribe_setpoint()
print(
    f"attached to sys_id={robot.sys_id} ({robot.robot_type!r}); watching "
    f"{setpoints.key} for {cmd.name(setpoints.cmd_id)} (id {setpoints.cmd_id}), "
    f"ignoring src_id={own_src_id}"
)
```

{{#endtab }}
{{#endtabs }}

Knowing your own `src_id` is where the surfaces diverge, and it matters here because it is
how you filter your own traffic off the bus. Rust and Python can read the options back
(`robot.options["src_id"]` in Python); the C++ surface has no accessor for them, so the
example sets `src_id` explicitly in the connect options and keeps its own constant.

```text
attached to sys_id=<id> (GlobalHawk); watching vrobots/<id>/z/cmd for SET_ANGVEL (id 51), ignoring src_id=122
```

Dropping the stream undeclares the subscription and nothing else. It does not stop anyone
publishing, and the robot never learns that you were listening.

## What a setpoint carries

| Field | Type | Units | Notes |
|---|---|---|---|
| `cmd_id` | `u32` | | the id this stream filtered on |
| `value` | `[f64; 3]` | rad/s for `SET_ANGVEL` | the `vec3` payload **unconverted**, in the sender's frame |
| `axis_convention` | `Axes` | | the convention `value` is expressed in |
| `coord_frame_id` | `String` | | the frame id; authoritative when it and `axis_convention` disagree |
| `src_id` | `u32` | | who sent it; the in-game IMU panel is **108** |
| `sys_id` | `u32` | | which robot it was addressed to |
| `seq` | `u64` | | the sender's per-topic sequence number |
| `t_ns` | `i64` | ns since the unix epoch | the sender's capture time |
| `elapsed` | `f64` | s | the same clock as `State::elapsed`, so a setpoint and a state are directly subtractable |

The vector is taken exactly as the sender stamped it. The simulator converts its own copy,
so a controller that skips the conversion disagrees with the simulator by a permutation and
a sign or two. For `SET_ANGVEL` off the IMU panel the frame is the target robot's own,
which for a fixed wing is FRD, so it reads as `[p, q, r]`.

## Filtering out your own traffic

Everything anyone sends to the robot arrives on this stream, your own commands included.
Compare `Setpoint::src_id` against `ConnectOptions::src_id`, which defaults to 122:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
        // --- the setpoint: latched, so read the current one every iteration ---
        let setpoint = setpoints.latest();
        let demand = match &setpoint {
            // Our own traffic comes back on this bus too. It is not a setpoint.
            Some(sp) if sp.src_id == own_src_id => [0.0; 3],
            Some(sp) => {
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex32_fw_rate_controller.cpp`:

```cpp
// --- the setpoint: latched, so read the current one every iteration
const std::optional<vrsdk::Setpoint> setpoint = setpoints.latest();
double demand[3] = {0.0, 0.0, 0.0};
if (setpoint && setpoint->src_id() != OWN_SRC_ID) {
    // Our own traffic comes back on this bus too. It is not a
    // setpoint.
    const std::array<double, 3> value = setpoint->value();
    demand[0] = value[0];
    demand[1] = value[1];
    demand[2] = value[2];
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex32_fw_rate_controller.py`:

```python
# --- the setpoint: latched, so read the current one every iteration ---
setpoint = setpoints.latest
if setpoint is None or setpoint.src_id == own_src_id:
    # Nobody has ever published one, or it is our own traffic coming back
    # on this bus. "Hold zero rates" is a decision.
    demand = (0.0, 0.0, 0.0)
else:
    demand = setpoint.value
```

{{#endtab }}
{{#endtabs }}

Two absences collapse into one test in every surface: no setpoint has ever arrived, and the
only setpoint is your own echo. Both mean "hold zero rates" here. Note that `latest` is a
property in Python and a method in the other two.

Skipping that arm makes a controller chase its own output, which reads as a loop that will
not settle rather than as a bug in the filter.

## latest, not fresh

| Method | Returns | Use it when |
|---|---|---|
| `latest()` | the current setpoint, new or not, `None` before the first ever arrives | a rate loop: a setpoint latches, so "the current command" is what you want every iteration |
| `fresh()` | the setpoint only if it is new since the last call, handed out exactly once | something that must not act twice on one operator input |
| `wait_new_setpoint(timeout)` | blocks until a newer one arrives | `VrError::Timeout` means nobody is publishing, which for a hand-flown panel is most of the time |

A `None` from `fresh()` does not mean the setpoint went away. The bus latches, and a
publisher that stopped sending has not commanded zero, which is why `latest()` is almost
always the right read here.

> **Gotcha.** Before the first setpoint ever arrives, `latest()` is `None`. Whatever you
> substitute is a decision, not a default: ex32 holds zero rates, which is safe on an
> aircraft that is already trimmed and would not be on every plant.

## Stream health

`SetpointStats` carries `received`, `filtered`, `decode_errors`, `seq_gaps` and `last_seq`.
`filtered` climbing fast is normal: it counts every other peer's traffic to the same robot,
which is everything that is not the one id you asked for. `seq_gaps` is only meaningful
with a single publisher, since two senders interleaving their own sequences look like gaps.
Decode errors are counted rather than returned and never tear the subscription down.

**Next:** [Publishing estimates](09-publishing-estimates.md)

**See also:** [Fixed wing control](05-fixed-wing.md), [Stream health](../ch03-reading-state/08-health.md), [Frames, axes and units](../ch02-concepts/07-frames-and-units.md)
