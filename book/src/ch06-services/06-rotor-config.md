# Rotors and thrust curves

Rotor geometry and thrust polynomials, with no mixing matrix anywhere.

```sh
cargo run -p vrobots-examples --bin ex27_rotor_config -- 1
./target/cpp-build/ex27_rotor_config 1
python examples/python/ex27_rotor_config.py 1
cargo run -p vrobots-examples --bin ex27_rotor_config
./target/cpp-build/ex27_rotor_config
python examples/python/ex27_rotor_config.py
```

`ex27_rotor_config` takes an **optional** `sys_id`. With it, the example attaches to the
scene's own multirotor; without it, the example creates one and every climb run reads 0.00 m/s,
because a client-created multirotor does not integrate physics in simulator v3.0.0
([known issue](../ch07-robots/07-known-issues.md)). Prefer the argument, and read the warning
about attaching below before you do.

## Multirotor only

`srv/rotors` is the multirotor's own service. Anything else returns
[`VrError::NoResponder`](../appendix-c-errors.md).

## The three curves

From the `examples/rust/src/bin/ex27_rotor_config.rs` header, with `pwm` the commanded pulse
width in microseconds and `g` the scene's gravity magnitude:

```text
thrust = (thrust_a*pwm^2 + thrust_b*pwm + thrust_c) * g            [N]
torque = spin_dir * (torque_a*pwm^2 + torque_b*pwm + torque_c) * g [N.m]
omega  = ang_vel_slope*pwm + ang_vel_intercept                     [rad/s]
```

Roll, pitch and yaw are not in that list because **there is no mixing matrix anywhere in the
simulator**. Moments fall out of the rotor positions, so moving a rotor really does change the
airframe's response, and an asymmetric aircraft is a different rotor list rather than a
different mixer.

## `RotorSpec`

`RotorSpec::default()` is the simulator's own reference rotor, and it is the base to build on:
there are no per-field flags inside an entry, so a zero is a zero coefficient and not "leave it
alone".

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `position` | `[f64;3]` | m | `[0.0, 0.0, 0.0]` | hub position **from the robot origin, not the centre of mass**; read in your header frame |
| `spin_dir` | `f64` | | `0.0` | sign of the yaw reaction torque: `+1` clockwise, `-1` counter-clockwise, **`0` lets the simulator alternate by index** with even indices clockwise |
| `thrust_a` | `f64` | | `7.5e-7` | quadratic term |
| `thrust_b` | `f64` | | `-0.001325` | linear term |
| `thrust_c` | `f64` | | `0.55` | constant term |
| `torque_a` | `f64` | | `7.5e-8` | quadratic term |
| `torque_b` | `f64` | | `-0.0001325` | linear term |
| `torque_c` | `f64` | | `0.055` | constant term |
| `ang_vel_slope` | `f64` | rad/s per µs | `1.33` | slope of the **reported** propeller speed |
| `ang_vel_intercept` | `f64` | rad/s | `-1466.67` | intercept of the same line |
| `pwm_min_us` | `u32` | µs | `1100` | bottom of this rotor's band |
| `pwm_max_us` | `u32` | µs | `2000` | top of the band; the simulator substitutes 1100 to 2000 if it is not above `pwm_min_us` |

> **Gotcha.** A bare `RotorSpec::default()` puts the rotor at `[0, 0, 0]`. A list of those is an
> aircraft with thrust and no control authority at all, because every moment arm is zero.
> Whatever you send **is** the airframe now.

## Positions are from the origin

The simulator subtracts the centre-of-mass offset itself, so a position measured from the
centre of mass gets it subtracted twice. Measure from the robot's origin.

Positions are read in your header frame, which is `unity` by default: +x right, +y up, +z
forward, so a flat rotor ring lives in the x-z plane at y = 0. That is how `ex27` lays one out.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex27_rotor_config.rs`:

```rust
fn ring(n: usize) -> Vec<RotorSpec> {
    (0..n)
        .map(|i| {
            let angle = std::f64::consts::FRAC_PI_4
                + (i as f64) * std::f64::consts::TAU / (n.max(1) as f64);
            RotorSpec::default().with_position([ARM_M * angle.sin(), 0.0, ARM_M * angle.cos()])
        })
        .collect()
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex27_rotor_config.cpp`:

```cpp
std::vector<vrsdk_rotor_spec_t> ring(std::size_t n) {
    std::vector<vrsdk_rotor_spec_t> out;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double angle =
            PI / 4.0 + static_cast<double>(i) * 2.0 * PI / static_cast<double>(n > 0 ? n : 1);
        vrsdk_rotor_spec_t rotor = vrsdk::rotor_spec();
        rotor.position[0] = ARM_M * std::sin(angle);
        rotor.position[1] = 0.0;
        rotor.position[2] = ARM_M * std::cos(angle);
        out.push_back(rotor);
    }
    return out;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex27_rotor_config.py`:

```python
def ring(n: int) -> list[RotorSpec]:
    """``n`` reference rotors laid out on a flat ring of radius ``ARM_M``.

    In the default ``"unity"`` header frame the horizontal plane is x-z and +y is
    up, so the ring sits at y = 0. ``spin_dir`` is left at 0 so the simulator
    alternates clockwise/counter-clockwise by index, which is what keeps the yaw
    torques cancelling.
    """
    out = []
    for i in range(n):
        angle = math.pi / 4.0 + i * math.tau / max(n, 1)
        out.append(
            RotorSpec(position=(ARM_M * math.sin(angle), 0.0, ARM_M * math.cos(angle)))
        )
    return out
```

{{#endtab }}
{{#endtabs }}

Each entry starts from the simulator's own reference rotor on every surface, `RotorSpec::default()`,
`vrsdk::rotor_spec()` and `RotorSpec()`, because an entry carries no `has_*` flags: a field left at
zero is a zero coefficient rather than an untouched one. C++ writes the three position components
into a fixed array, where Rust and Python pass the whole triple.

`spin_dir` is left at 0 so the simulator alternates clockwise and counter-clockwise by index,
which is what keeps the yaw torques cancelling.

## The list replaces the list

One verb, no upsert: the slice you send becomes the whole rotor list. Two consequences follow.

**It must describe every rotor, in index order.** The rotor count is fixed when the airframe
spawns and is `actuator.pwm.len()` in the state stream. Read it rather than assuming four.

**A wrong-length slice drops the entire request.** Never a partial apply, and acked `ok`
anyway. `ex27` proves it by sending one entry too few, with a curve that would put the aircraft
on the ground:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    let short: Vec<RotorSpec> = ring(rotors.saturating_sub(1))
        .into_iter()
        .map(|r| r.with_thrust_curve(0.0, 0.0, 0.02))
        .collect();
    println!(
        "configure_rotors with {} entries for {rotors} rotors ...",
        short.len()
    );
    robot.configure_rotors(&short)?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex27_rotor_config.cpp`:

```cpp
std::vector<vrsdk_rotor_spec_t> shortlist = ring(rotors > 0 ? rotors - 1 : 0);
for (vrsdk_rotor_spec_t& r : shortlist) {
    r.thrust_a = 0.0;
    r.thrust_b = 0.0;
    r.thrust_c = 0.02;
}
std::printf("configure_rotors with %zu entries for %zu rotors ...\n", shortlist.size(),
            rotors);
robot.configure_rotors(shortlist);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex27_rotor_config.py`:

```python
short = [
    RotorSpec(position=r.position, thrust_a=0.0, thrust_b=0.0, thrust_c=0.02)
    for r in ring(max(rotors - 1, 0))
]
print(f"configure_rotors with {len(short)} entries for {rotors} rotors ...")
robot.configure_rotors(short)
```

{{#endtab }}
{{#endtabs }}

Rust sets all three coefficients through one `with_thrust_curve` call, C++ assigns them on each
struct in place, and Python rebuilds each entry around the position the ring produced. The wire
result is identical, and so is the outcome: a list one entry short is dropped whole.

The aircraft climbs exactly as it did before, which is the proof that nothing was applied:

```text
configure_rotors with <n-1> entries for <n> rotors ...
... returned Ok. That is a receipt, and the request was dropped:
```

The one length the SDK does refuse is zero, which returns `VrError::InvalidArgument`.

## What the state stream will not tell you

`actuator.measured` is rotor speed in rad/s, computed from `ang_vel_slope` and
`ang_vel_intercept` and the commanded pulse width, and nothing else. It is a reported line, not
a measurement. Run 3 of `ex27` cuts the thrust curve to 70% and leaves that line alone:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    let weak: Vec<RotorSpec> = ring(rotors)
        .into_iter()
        .map(|r| {
            let d = RotorSpec::default();
            r.with_thrust_curve(
                d.thrust_a * THRUST_SCALE,
                d.thrust_b * THRUST_SCALE,
                d.thrust_c * THRUST_SCALE,
            )
        })
        .collect();
    robot.configure_rotors(&weak)?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex27_rotor_config.cpp`:

```cpp
std::vector<vrsdk_rotor_spec_t> weak = ring(rotors);
const vrsdk_rotor_spec_t reference = vrsdk::rotor_spec();
for (vrsdk_rotor_spec_t& r : weak) {
    r.thrust_a = reference.thrust_a * THRUST_SCALE;
    r.thrust_b = reference.thrust_b * THRUST_SCALE;
    r.thrust_c = reference.thrust_c * THRUST_SCALE;
}
robot.configure_rotors(weak);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex27_rotor_config.py`:

```python
reference = RotorSpec()
weak = [
    RotorSpec(
        position=r.position,
        thrust_a=reference.thrust_a * THRUST_SCALE,
        thrust_b=reference.thrust_b * THRUST_SCALE,
        thrust_c=reference.thrust_c * THRUST_SCALE,
    )
    for r in ring(rotors)
]
robot.configure_rotors(weak)
```

{{#endtab }}
{{#endtabs }}

All three read the reference coefficients back off a fresh default rotor rather than repeating the
numbers from the table above, so 70% stays 70% of whatever the SDK's reference rotor is. The list
is full length here, so this one applies.

The aircraft stops climbing while the rotor-speed echo does not move a digit:

```text
1800 us on every rotor, three times:
  as spawned               climb=<rate> m/s   rotor speed echo=[...]
  after the short list     climb=<rate> m/s   rotor speed echo=[...]
  70% thrust curve         climb=<rate> m/s   rotor speed echo=[...]
```

Height is the evidence about thrust. The echo is not.

## Attaching is a one-way door

This service replaces the rotor list and there is no read-back. Run it against the scene's
multirotor and that aircraft flies whatever geometry and curves you sent, for every other
client, until the scene is reloaded. `reset()` will not undo it, and the SDK cannot restore
what it was never able to read.

**Next:** [Mass spring damper and cart pole](07-msd-cartpole-config.md)

**See also:** [Driving a multirotor](../ch04-commands/02-multirotor.md), [Multirotor](../ch07-robots/01-multirotor.md), [Mass and inertia](02-physical-params.md)
