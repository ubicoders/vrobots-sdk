# Actuators

The three index-parallel arrays that carry your last command back to you beside what the devices actually did.

## Command in, motion out

`State::actuator` is the only block in a snapshot that is neither truth, measurement nor
belief. Two of its arrays are an echo of the command you sent, and the third is the
physical response of the devices. It is published on every state sample, at the state
rate, whether or not you ever send a command.

The three arrays are **index-parallel**: entry *i* is the same device in each of them.
There is no name, no id and no length field beyond the arrays' own lengths, so the
mapping from index to physical device comes from the robot type, not from the snapshot.

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `pwm` | `Vec<u32>` | µs | empty | commanded pulse widths, your last command echoed back |
| `normalized` | `Vec<f64>` | | empty | the same command mapped to [-1, 1] |
| `measured` | `Vec<f64>` | varies by device | empty | what the device actually did |

`measured` is the array to read carefully, because its unit is a property of the device
at that index rather than of the array: a rotor reports rad/s, a wheel reports rad/s,
and a servo reports rad. Nothing in the snapshot tells you which, so the robot type
decides. Chapter 7 gives the per-robot layouts.

> **Note.** All three arrays default to empty rather than to a fixed length. A robot
> that has never been commanded, or a nested table missing from the wire, decodes to
> empty vectors. Index them with `get`, or check the length, before assuming a rotor
> count.

## The echo is the only proof a command landed

Commands are fire and forget. They latch, they get no reply, and publishing to a topic
nobody is subscribed to is not an error in zenoh, so a command sent while the simulator
is closed still returns `Ok`. There is no acknowledgement anywhere in the command path.

That leaves exactly one way to confirm that a robot received what you sent: read
`actuator.pwm` in the next state sample and compare it with what you commanded. The
robust-loop example makes the point by printing the echo beside the sequence number in
its status line.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex19_robust_loop.rs`:

```rust
let s = robot.states();
if samples.is_multiple_of(REPORT_EVERY) {
    let [x, y, z] = s.kin.lin_pos;
    let stats = robot.stats();
    println!(
        "ok  seq={} t={:.2}s pos=({x:.2},{y:.2},{z:.2}) echo={:?} \
         received={} gaps={} decode_errors={}",
        s.seq,
        s.elapsed,
        s.actuator.pwm,
        stats.received,
        stats.seq_gaps,
        stats.decode_errors
    );
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex19_robust_loop.cpp`:

```cpp
const vrsdk::State s = robot.states();
if (samples % REPORT_EVERY == 0) {
    const double* p = s.kin().lin_pos;
    const vrsdk_state_stats_t st = robot.stats();
    std::printf(
        "ok  seq=%llu t=%.2fs pos=(%.2f,%.2f,%.2f) received=%llu gaps=%llu "
        "decode_errors=%llu\n",
        static_cast<unsigned long long>(s.seq), s.elapsed, p[0], p[1], p[2],
        static_cast<unsigned long long>(st.received),
        static_cast<unsigned long long>(st.seq_gaps),
        static_cast<unsigned long long>(st.decode_errors));
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex19_robust_loop.py`:

```python
s = mr.states
if samples % REPORT_EVERY == 0:
    x, y, z = s.kin.lin_pos
    st = mr.stats
    print(
        f"ok  seq={s.seq} t={s.elapsed:.2f}s pos=({x:.2f},{y:.2f},{z:.2f}) "
        f"echo={s.actuator.pwm} received={st.received} gaps={st.seq_gaps} "
        f"decode_errors={st.decode_errors}"
    )
```

{{#endtab }}
{{#endtabs }}

Rust and Python print `actuator.pwm` as a list; C++ has it as `s.raw.actuator.pwm` with a
separate `pwm_count`, and `s.pwm()` is the convenience that turns the pair into a
`std::vector<std::uint32_t>`. That count matters: the array is fixed-size and only the first
`pwm_count` entries mean anything.

The `echo=` field is the `pwm` array verbatim, so a multirotor commanded to 1501 µs on
all four rotors reports it back:

<!-- VERIFY: printout reconstructed from the format string; the values are illustrative, not captured from a run. -->

```text
ok  seq=125 t=5.00s pos=(0.03,0.85,-1.20) echo=[1501, 1501, 1501, 1501] received=125 gaps=0 decode_errors=0
```

An echo that does not match what you sent is informative in itself. Values that stay at
their previous setting across several samples mean your command did not reach the
robot. An empty array means the robot has never been commanded at all. Values that
differ from yours but do change when you change yours mean the robot received the
command and did something to it, which is a question for the command chapter rather
than this one.

## Three arrays, three questions

| Array | Answers | Use it when |
|---|---|---|
| `pwm` | what did I command, in the units I commanded it | confirming a command landed, and comparing against your own setpoint |
| `normalized` | what fraction of the range was that | logging or plotting across robots with different pulse-width ranges |
| `measured` | what did the hardware do | detecting saturation, lag between command and response, and stalled devices |

The gap between `normalized` and `measured` is where actuator dynamics would show. A
step in the command appears in `pwm` and `normalized` as soon as the robot has it,
because those two are the command, while `measured` can only follow as fast as the
modelled device does. Diffing the two over a step is how you observe that behaviour
without instrumenting the simulator.

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

> **Gotcha.** Index-parallel does not mean equal length in general. Read the length of
> the array you are about to index rather than the length of `pwm`, particularly on
> robot types whose command surface is not a pulse width at all.

**Next:** [Timestamps and sequence numbers](06-timestamps.md)

**See also:** [Commands latch](../ch04-commands/01-latching.md), [Global Hawk](../ch07-robots/06-globalhawk.md), [Rotors and thrust curves](../ch06-services/06-rotor-config.md)
