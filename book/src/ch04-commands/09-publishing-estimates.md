# Publishing estimates

Putting your filter's belief on the wire so the fixed wing can fly it instead of the simulator's truth.

```sh
cargo run -p vrobots-examples --bin ex35_publish_estimate -- <sys_id>
./target/cpp-build/ex35_publish_estimate <sys_id>
python examples/python/ex35_publish_estimate.py <sys_id>
```

The Global Hawk is scene-authored and lives in the IMU scene, so the example takes the live
`sys_id` as an argument. Find it with `vrobots topic list`.

## The simulator deliberately leaves the estimate empty

Every snapshot carries an `estimate` block, and it is always zeroed with `valid = false`.
That is not a gap. The simulator runs no filter of its own and omits the field on purpose,
because an estimate mirroring truth would make `estimate.kin - kin` a tautology instead of
an estimator error. [Truth, measured and
believed](../ch03-reading-state/01-truth-measured-believed.md) is that split from the
reading side. The two topics divide along the same line and travel in opposite directions.

| Topic | Direction | Carries |
|---|---|---|
| `vrobots/{sys_id}/z/state` | the simulator publishes | `State`, with truth in `kin` and measurements in `sensors` |
| `vrobots/{sys_id}/z/estimate` | the simulator subscribes | `EstimateState`, your belief, published by you |

The loop is therefore: read `sensors`, run your filter, publish the result, repeat.

## The two entry points

Both build the same wire message. The quaternion form:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`crates/vrobots-sdk/src/commands.rs`:

```rust
    pub fn publish_estimate(
        &self,
        quat: [f64; 4],
        angular_rates: Option<[f64; 3]>,
        valid: bool,
    ) -> VrResult<()>
```

{{#endtab }}
{{#tab name="C++" }}

`include/vrobots_sdk.hpp`:

```cpp
    void publish_estimate(const Quat& quat, std::optional<Vec3> angular_rates = std::nullopt,
                          bool valid = true)
```

{{#endtab }}
{{#tab name="Python" }}

`vrsdk/_vrsdk.pyi`:

```python
def publish_estimate(
    self,
    quat: Sequence[float],
    angular_rates: Optional[Sequence[float]] = None,
    valid: bool = True,
) -> None: ...
```

{{#endtab }}
{{#endtabs }}

C++ and Python default `angular_rates` to none and `valid` to true; Rust asks for all
three. `publish_estimate_euler(euler, order, angular_rates, valid)` is the same call from
angles: it runs `rotations::euler_to_quat` and hands the result to `publish_estimate`, and
everything after the conversion is identical.

| Argument | Meaning | Notes |
|---|---|---|
| `quat` | the believed attitude, `[x, y, z, w]` | sent exactly as given; the SDK will not normalise it for you, because a filter drifting off the unit sphere is a bug worth seeing |
| `angular_rates` | believed body rates, rad/s, or none | none leaves `twist` off the wire entirely, which says "my filter does not estimate rates"; zeros claim the body is not rotating. No consumer reads it today |
| `valid` | the convergence flag | a gate, not a hint. See below |

`InvalidArgument` comes back for a non-finite component or a quaternion whose norm is too
near zero to be an attitude. There is no reply and no ack beyond that: the confirmation is
the simulator's `_Est` cockpit gauges moving.

## Publishing one

The Rust and C++ versions of ex35 wrap the call in a helper so all four phases publish
identically. From `examples/rust/src/bin/ex35_publish_estimate.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    // valid = true throughout. The gyro rates go along for the ride; nothing
    // reads them yet.
    robot.publish_estimate(quat, Some(s.sensors.gyroscope.angular_velocity), true)?;
    Ok(Some(quat))
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex35_publish_estimate.cpp`:

```cpp
// valid = true throughout. The gyro rates go along for the ride; nothing
// reads them yet.
robot.publish_estimate(quat, gyro_of(s), true);
return quat;
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex35_publish_estimate.py`:

```python
if estimator == "copy":
    robot.publish_estimate(s.kin.quat, s.sensors.gyroscope.angular_velocity, True)
elif estimator == "bias":
    # post-multiply: the bias is applied about the BODY pitch axis
    quat = rotations.quat_multiply(s.kin.quat, bias)
    robot.publish_estimate(quat, s.sensors.gyroscope.angular_velocity, True)
# "silent": nothing on the wire; after 0.5 s the loop falls back to truth
```

The Python script has no helper. It calls `publish_estimate` inline in each phase's loop,
with the gyro rates and `True` for `valid` every time, and publishes nothing in the silent
phase.

{{#endtab }}
{{#endtabs }}

It prints nothing of its own and returns once zenoh has accepted the put, exactly like a
command; the only observable difference is what the aircraft does next.

## It must keep coming

A command latches: send `set_angvel` once and the setpoint stands until you change it. An
estimate does the opposite. The simulator ages it **from arrival**, in sim time, and stops
trusting it after 0.5 seconds, so a publisher that pauses has handed the aircraft back to
truth. Publish it every control iteration, at 20 Hz or better, which is why every loop in
ex35 pairs one publish with one `rate` call:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    for _ in 0..SETTLE_SAMPLES {
        publish(robot, &robot.states(), estimator)?;
        robot.rate(HZ);
    }
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex35_publish_estimate.cpp`:

```cpp
for (int i = 0; i < SETTLE_SAMPLES; ++i) {
    publish(robot, robot.states(), estimator);
    robot.rate(HZ);
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex35_publish_estimate.py`:

```python
for i in range(SETTLE_SAMPLES + MEASURE_SAMPLES):
    s = robot.states
    if estimator == "copy":
        robot.publish_estimate(s.kin.quat, s.sensors.gyroscope.angular_velocity, True)
    elif estimator == "bias":
        # post-multiply: the bias is applied about the BODY pitch axis
        quat = rotations.quat_multiply(s.kin.quat, bias)
        robot.publish_estimate(quat, s.sensors.gyroscope.angular_velocity, True)
    # "silent": nothing on the wire; after 0.5 s the loop falls back to truth
    if i >= SETTLE_SAMPLES:
        pitch_sum += math.degrees(rotations.quat_to_euler(s.kin.quat, ORDER)[1])
        altitude = -s.kin.lin_pos[2]  # frd: the third component is down
    robot.rate(HZ)
```

The Python script runs the settle and measurement windows as one loop of
`SETTLE_SAMPLES + MEASURE_SAMPLES` iterations and prints nothing inside it; it only
accumulates the true pitch over the last `MEASURE_SAMPLES` of them.

{{#endtab }}
{{#endtabs }}

The example runs at 25 Hz, inside the window. In the Rust and C++ programs the settle loop
prints nothing, and the tracking window after it prints one progress line every two
seconds.

> **Gotcha.** `valid = false` is a gate, not a status flag. The simulator drops the message
> before it reads the quaternion **and does not reset the age counter**, so a stream of
> invalid estimates is indistinguishable, sim-side, from publishing nothing at all. Sending
> it while your filter is unconverged is still the honest thing to do; expect no way to tell
> it apart from a dead publisher.

## The frame comes from the header

The SDK leaves the estimate's own frame pair unset, so it inherits the header's: the
`coord_frame_id` and `axis_convention` from your `ConnectOptions`. Your quaternion must
therefore be expressed in the frame you connected with, not in the robot's, and the two
agree only if you make them. ex35 connects with
`ConnectOptions::default().with_frame("frd", Axes::FRD)` precisely so `s.kin.quat` can go
straight back out without conversion (the Python script passes the same pair as
`coord_frame_id="frd", axis_convention=2`), and the Rust and C++ programs check
`states().coord_frame_id` against the frame they stamped rather than assuming. Where they
differ, convert first: [Rotation conversions](../ch02-concepts/08-rotation-conversions.md)
is the arithmetic, and an attitude is the `M * C * M^T` case.

## Selecting it, and the four phases

Publishing alone changes nothing. `set_fw_est_source(cmd::FW_EST_OBSERVER)` is what makes
the onboard loop read `z/estimate` instead of truth. [Fixed wing
control](05-fixed-wing.md) is that selector, this page is the publisher, and neither works
without the other. ex35 walks all four combinations, measuring each the same way so the
rows compare.

| Phase | Source | Published | What happens |
|---|---|---|---|
| 1 | truth | the robot's own quaternion | nothing changes; the `_Est` gauges move |
| 2 | observer | the robot's own quaternion | still nothing: the estimate is right |
| 3 | observer | the same, pitched up 5 degrees | the nose trims down by about 5 degrees |
| 4 | observer | nothing at all | 0.5 s later the loop is back on truth |

Phase 1 is what makes phase 3 mean anything. A truth-copy estimator is the one whose error
is exactly zero, so if phases 1 and 2 already differed, the difference would be plumbing
rather than estimation. `reset()` runs before phase 4, because the lie costs tens of metres
of altitude and phase 4 is only comparable from a level start; that reset also clears the
estimate source and the rate setpoint, so the example re-sends both. The run ends with one
row per phase:

```text
what each phase flew on, and what the airframe did about it:
  1 truth source, truth-copy estimate    pitch=<deg>  roll=<deg>  r=<rad/s>  alt=<m> (<m> over 6.0 s)
  2 observer source, truth-copy estimate pitch=<deg>  roll=<deg>  r=<rad/s>  alt=<m> (<m> over 6.0 s)
  3 observer source, +5.0 deg pitch lie  pitch=<deg>  roll=<deg>  r=<rad/s>  alt=<m> (<m> over 6.0 s)
  4 observer source, nothing published   pitch=<deg>  roll=<deg>  r=<rad/s>  alt=<m> (<m> over 6.0 s)
```

The Python script prints a shorter table under `what each phase flew on:`, one row per
phase with the mean true pitch and the final altitude,
`<phase label> true pitch=<deg> deg  alt=<m> m`, and closes with
`phase 3 pitches down by about the lie; phases 1, 2 and 4 match`.

The onboard loop reads roll and pitch out of the believed attitude for two assists, a
wings-leveller and an altitude hold that biases the pitch demand. Tell it the nose is five
degrees higher than it is and the pitch assist trims five degrees of nose-down to correct an
error that does not exist. The rate loop tracks its demand exactly as well as in phase 1: it
is being asked for the wrong thing.

> **Note.** Five degrees is deliberately small: the altitude assist is clamped at ten, so a
> lie inside the clamp settles the aircraft lower instead of departing.

**Next:** [The showcase attitude estimator](10-showcase-estimator.md)

**See also:** [Fixed wing control](05-fixed-wing.md), [Truth, measured and believed](../ch03-reading-state/01-truth-measured-believed.md), [Rotation conversions](../ch02-concepts/08-rotation-conversions.md), [The showcase attitude estimator](10-showcase-estimator.md)
