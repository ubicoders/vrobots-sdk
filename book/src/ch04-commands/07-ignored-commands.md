# Commands nothing acts on

Several ids are fully defined on the wire and implemented by no robot type, and from the
client side they look exactly like a command you got wrong.

```sh
cargo run -p vrobots-examples --bin ex06_hello_throttle
./target/cpp-build/ex06_hello_throttle
python examples/python/ex06_hello_throttle.py
cargo run -p vrobots-examples --bin ex07_body_wrench
./target/cpp-build/ex07_body_wrench
python examples/python/ex07_body_wrench.py
```

## The worked example: SET_MR_THROTTLE

`SET_MR_THROTTLE` is normalised per-rotor throttle: four values on 0 to 1 instead of four
pulse widths. It is the command you would reach for to hover without thinking in
microseconds, and no robot type acts on it. `examples/rust/src/bin/ex06_hello_throttle.rs`
is therefore a lesson in what that looks like rather than a way to fly:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    loop {
        // Published exactly like set_mr_pwm: one put on vrobots/<id>/z/cmd, no
        // reply, latched until the next one arrives.
        robot.set_mr_throttle(THROTTLE)?;

        let s = robot.states();
        // The state frame is the robot's, not yours -- "frd" here, so lin_pos[2]
        // is DOWN, and altitude above the start point is its negation.
        let [_, _, down] = s.kin.lin_pos;
        println!(
            "sent {THROTTLE:?} -> alt={:.2} m  pwm={:?} normalized={:?} measured={:?}",
            -down,
            s.actuator.pwm,
            round3(&s.actuator.normalized),
            round3(&s.actuator.measured),
        );

        robot.rate(HZ);
    }
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex06_hello_throttle.cpp`:

```cpp
// ===== loop =====
for (;;) {
    // Published exactly like set_mr_pwm: one put on vrobots/<id>/z/cmd,
    // no reply, latched until the next one arrives.
    robot.set_mr_throttle(throttle);

    const vrsdk::State s = robot.states();
    // The state frame is the robot's, not yours -- "frd" here, so
    // lin_pos[2] is DOWN and altitude is its negation.
    const double alt = -s.kin().lin_pos[2];
    const vrsdk_actuator_t& a = s.actuator();

    std::printf("sent %.2f x4 -> alt=%.2f m  ", THROTTLE, alt);
    std::printf("pwm=[");
    for (std::uint32_t i = 0; i < a.pwm_count; ++i) {
        std::printf(i ? ",%u" : "%u", a.pwm[i]);
    }
    std::printf("] ");
    print_array("normalized", a.normalized, a.normalized_count);
    print_array("measured", a.measured, a.measured_count);
    std::printf("\n");

    robot.rate(HZ);
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex06_hello_throttle.py`:

```python
# ===== loop =====
while True:
    # Published exactly like set_mr_pwm: one put on vrobots/<id>/z/cmd, no
    # reply, latched until the next one arrives. Both call shapes work:
    # set_mr_throttle(a, b, c, d) and set_mr_throttle([a, b, c, d]).
    mr.set_mr_throttle(THROTTLE)

    s = mr.states
    # The state frame is the robot's, not yours -- "frd" here, so lin_pos[2]
    # is DOWN, and altitude above the start point is its negation.
    down = s.kin.lin_pos[2]
    norm = [round(v, 3) for v in s.actuator.normalized]
    meas = [round(v, 3) for v in s.actuator.measured]
    print(
        f"sent {THROTTLE} -> alt={-down:.2f} m  pwm={s.actuator.pwm} "
        f"normalized={norm} measured={meas}"
    )

    mr.rate(HZ)
```

{{#endtab }}
{{#endtabs }}

The call succeeds in all three and nothing moves in any of them, which is the point of the
example: an ignored command is indistinguishable from a delivered one at the call site.

Every line reports the same three actuator channels, and none of them moves in response to
what was sent:

```text
sent [0.6, 0.6, 0.6, 0.6] -> alt=<metres> m  pwm=<latched pulse widths> normalized=<latched normalised command> measured=<rotor rad/s>
```

There is no reply and no error, because the id space is shared across robot types and "not
mine" is correct behaviour rather than a fault. Run
[ex02](02-multirotor.md) alongside it: identical loop, an id the simulator implements, and
the echo moves.

> **Not yet.** `set_mr_throttle` is on the wire and acted on by nothing. Its payload field
> is the one the schema provides, but unlike `set_mr_pwm` it has never been confirmed
> against a consumer, so treat it as unverified until the simulator implements it. Today
> the way to fly a multirotor is `set_mr_pwm`.

## The body wrench group

The wrench group is the disturbance-injection channel the schema reserves for wind gusts,
payload drops and contact pushes. Three typed methods cover it, and today they are in the
same category as `SET_MR_THROTTLE`.

| Method | Command | Units | Wire payload |
|---|---|---|---|
| `set_body_force(f)` | `SET_BODY_FORCE` (200) | N | `vec3` = force |
| `set_body_torque(t)` | `SET_BODY_TORQUE` (201) | N·m | `vec3` = torque |
| `set_body_ft(f, t)` | `SET_BODY_FT` (202) | N and N·m | `vec3` = force, **`vec3_arr[0]`** = torque |

`SET_BODY_FT` is asymmetric because the schema is. The typed method hides it; building the
message by hand with [`CmdArgs`](06-generic-cmd.md) does not.

From `examples/rust/src/bin/ex07_body_wrench.rs`, one verb per iteration so each printed
line names exactly what went out:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
        let sent = match step % 3 {
            0 => {
                robot.set_body_force(GUST_N)?;
                format!("set_body_force({GUST_N:?})")
            }
            1 => {
                robot.set_body_torque(TWIST_NM)?;
                format!("set_body_torque({TWIST_NM:?})")
            }
            _ => {
                robot.set_body_ft(GUST_N, TWIST_NM)?;
                format!("set_body_ft({GUST_N:?}, {TWIST_NM:?})")
            }
        };
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex07_body_wrench.cpp`:

```cpp
// One verb per iteration, so each printed line names exactly what
// went out on the wire.
const char* sent = nullptr;
switch (step % 3) {
    case 0:
        robot.set_body_force(GUST_N);
        sent = "set_body_force(5, 0, 0)";
        break;
    case 1:
        robot.set_body_torque(TWIST_NM);
        sent = "set_body_torque(0, 0, 0.2)";
        break;
    default:
        robot.set_body_ft(GUST_N, TWIST_NM);
        sent = "set_body_ft((5,0,0), (0,0,0.2))";
        break;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex07_body_wrench.py`:

```python
# One verb per iteration, so each printed line names exactly what went
# out on the wire. Each takes three scalars or one sequence.
if step % 3 == 0:
    mr.set_body_force(GUST_N)
    sent = f"set_body_force({GUST_N})"
elif step % 3 == 1:
    mr.set_body_torque(TWIST_NM)
    sent = f"set_body_torque({TWIST_NM})"
else:
    mr.set_body_ft(GUST_N, TWIST_NM)
    sent = f"set_body_ft({GUST_N}, {TWIST_NM})"
step += 1
```

{{#endtab }}
{{#endtabs }}

The three verbs are one for one across the surfaces. C++ takes `std::array<double, 3>` and
Python takes either three scalars or one sequence; both hide the same schema asymmetry, in
which `set_body_ft` puts the force in `vec3` and the torque in `vec3_arr[0]`.

The state block it prints afterwards is `state.wrench`, the total force and torque the
**simulator** has on the body. That is where the effect will appear the day the simulator
implements these ids. Until then it shows the robot's own actuators and nothing of yours:

```text
set_body_force([5.0, 0.0, 0.0])
    state.wrench force=(<fx>,<fy>,<fz>) N  torque=(<tx>,<ty>,<tz>) N.m  in "<frame>"
```

The vectors are still frame-tagged on the way out, using the `coord_frame_id` from your
connect options. An untagged vector would be taken at face value and would silently flip
sign between opposite-handed conventions, which is why the SDK always tags.

## Telling ignored from wrong

You cannot, from outside. There is no acknowledgement to inspect and no error to catch, so
every one of these produces the same symptom:

| What you observe | Possible causes |
|---|---|
| `Ok(())`, and the state stream does not change | the id is not implemented; the id is implemented by a different robot type; the `sys_id` is not the robot you meant; the array length does not match the airframe; the value was clamped to what was already latched |

The one tool that separates them is the echo. Print `actuator.pwm`, `actuator.normalized`
and `actuator.measured` every iteration, and compare their contents and their **lengths**
against what you sent. A channel that never moves while its neighbours do points at the id;
an actuator block that never moves at all while `elapsed` keeps advancing points at the
robot, which is either of a type that does not implement the command or not the robot you
meant to attach to.

Confirm which robot is which with `vrobots topic list` before blaming a command.

**Next:** [Reading someone else's commands](08-reading-commands.md)

**See also:** [The generic command](06-generic-cmd.md), [When nothing happens](../ch01-getting-started/08-troubleshooting.md), [Actuators](../ch03-reading-state/05-actuator.md)
