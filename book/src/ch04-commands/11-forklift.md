# Driving the forklift

`SET_FORKLIFT` carries four pulse widths and an optional fifth, and two of the four are
positions on the mast rather than sticks.

```sh
cargo run -p vrobots-examples --bin ex37_hello_forklift
./target/cpp-build/ex37_hello_forklift
python examples/python/ex37_hello_forklift.py
```

The warehouse scene pins its forklift at `sys_id` 0, which is what the example attaches to
when you pass no argument. Pass another id to drive a forklift somewhere else.

## The signature


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`crates/vrobots-sdk/src/commands.rs`:

```rust
pub fn set_forklift(
    &self,
    steer_us: f64,
    throttle_us: f64,
    lift_us: f64,
    fork_width_us: f64,
) -> VrResult<()>

pub fn set_forklift_with_brake(
    &self,
    steer_us: f64,
    throttle_us: f64,
    lift_us: f64,
    fork_width_us: f64,
    brake_us: f64,
) -> VrResult<()>
```

{{#endtab }}
{{#tab name="C++" }}

`include/vrobots_sdk.hpp`:

```cpp
void set_forklift(double steer_us, double throttle_us, double lift_us, double fork_width_us,
                  std::optional<double> brake_us = std::nullopt)
```

{{#endtab }}
{{#tab name="Python" }}

`vrsdk/_vrsdk.pyi`:

```python
def set_forklift(
    self,
    steer_us: float,
    throttle_us: float,
    lift_us: float,
    fork_width_us: float,
    brake_us: Optional[float] = None,
) -> None: ...
```

{{#endtab }}
{{#endtabs }}

Rust has no optional argument, so the brake is a second method: `set_forklift` sends the
four-channel frame and `set_forklift_with_brake` the five-channel one. The channels go on
the wire as `int_arr` in that order, under command id `SET_FORKLIFT` (312). A frame holds
four entries or five. The forklift ignores any other length.

## The five channels

| Channel | Units | 1100 | 1500 | 1900 | Notes |
|---|---|---|---|---|---|
| `steer_us` | µs | full left | straight | full right, 45 degrees | centre-sprung, ±20 µs deadband |
| `throttle_us` | µs | full reverse | stop, on the idle brake | full forward, about 5 m/s unloaded | centre-sprung, ±20 µs deadband |
| `lift_us` | µs | -0.215 m | 0.448 m | 1.11 m | position, no deadband |
| `fork_width_us` | µs | 0.158 m | 0.746 m | 1.334 m | position, no deadband |
| `brake_us` | µs | released | half | full | optional, **bottom-anchored** |

The front pair drives and the rear pair steers, so the tail swings out in a turn.

The SDK refuses a value outside 1100 to 2000 µs before publishing. The forklift's own band
tops out at 1900, so a value between 1900 and 2000 is published and then clamped by the
simulator, and the echo shows the clamped pulse. Nothing is frame-converted: these are
per-channel numbers.

## Two kinds of channel

Steer and throttle are centre-sprung, like the [truck's](03-truck.md): 1500 is neutral, and
the distance from it is the demand. A throttle inside the deadband holds the forklift on its
idle brake.

Lift and fork width have no centre. Every microsecond is a place on the mast, linear across
the band, so 1500 is not "hold": it is the middle of the travel. The lift value is the
carriage height, which is negative near the bottom by design, because 1100 puts the forks at
the pocket height of a pallet standing on the floor. The fork width is measured centre to
centre.

> **Gotcha.** A frame carries all four channels every time. A drive frame that sends 1500 for
> the two mast channels because "1500 is neutral" raises the forks to 0.448 m and spreads them
> to 0.746 m. Send the mast pulses you want held in every frame, as the example does.

## The mast slews, so the echo is early

The carriage travels at 0.5 m/s and the fork width changes at 0.1 m/s, so a full change of
width takes about 12 seconds. `actuator.pwm` echoes a new pulse on the next state sample,
while the forks are still on their way. The move is finished when `actuator.measured`
arrives, and `forklift_actuators()` decodes that array into named values.

From `ex37_hello_forklift`, the loop that streams one mast frame until the measured height
and width match:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex37_hello_forklift.rs`:

```rust
for i in 0..MAST_SAMPLES {
    robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us)?;
    robot.rate(HZ);
    let a = robot.forklift_actuators()?;
    let t = robot.states().elapsed - t0;
    let there = (a.lift_height - want_lift).abs() < TOLERANCE_M
        && (a.fork_width - want_width).abs() < TOLERANCE_M;
    if there {
        println!("   t={t:5.2}s  {}  -- there", report(&a));
        return Ok(());
    }
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex37_hello_forklift.cpp`:

```cpp
for (int i = 0; i < MAST_SAMPLES; ++i) {
    robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us);
    robot.rate(HZ);
    const vrsdk::ForkliftActuators a = robot.forklift_actuators();
    const double t = robot.states().elapsed - t0;
    const bool there = std::fabs(a.lift_height - want_lift) < TOLERANCE_M &&
                       std::fabs(a.fork_width - want_width) < TOLERANCE_M;
    if (there) {
        std::printf("   t=%5.2fs  %s  -- there\n", t, report(a).c_str());
        return;
    }
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex37_hello_forklift.py`:

```python
for i in range(MAST_SAMPLES):
    robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us)
    robot.rate(HZ)
    a = robot.forklift_actuators()
    t = robot.states.elapsed - t0
    there = (
        abs(a.lift_height - want_lift) < TOLERANCE_M
        and abs(a.fork_width - want_width) < TOLERANCE_M
    )
    if there:
        print(f"   t={t:5.2f}s  {report(a)}  -- there")
        return
```

{{#endtab }}
{{#endtabs }}

The frame latches, so sending it once would do. Sending it every iteration costs nothing and
heals a frame lost on the way. The loop prints progress lines and then the arrival:

```text
-- lift 1500 us, width 1500 us: by the band 0.448 m and 0.746 m --
   t= 0.03s  lift -0.205 m  width 0.455 m  drive +0.00 rad/s (+0.00 m/s)  steering  +0.0 deg
   t= 0.54s  lift +0.055 m  width 0.507 m  drive +0.00 rad/s (+0.00 m/s)  steering  +0.0 deg
   t= 1.03s  lift +0.295 m  width 0.555 m  drive +0.00 rad/s (+0.00 m/s)  steering  +0.0 deg
   t= 1.54s  lift +0.447 m  width 0.607 m  drive +0.00 rad/s (+0.00 m/s)  steering  +0.0 deg
   t= 2.03s  lift +0.447 m  width 0.655 m  drive +0.00 rad/s (+0.00 m/s)  steering  +0.0 deg
   t= 2.54s  lift +0.447 m  width 0.707 m  drive +0.00 rad/s (+0.00 m/s)  steering  +0.0 deg
   t= 2.89s  lift +0.447 m  width 0.741 m  drive +0.00 rad/s (+0.00 m/s)  steering  +0.0 deg  -- there
```

The carriage is up within a second and a half, and the forks are still spreading a second
and a half after that.

## The report is not in command order

The command is `[steer, throttle, lift, width]`. The actuator report leads with the drive,
so `actuator.pwm[0]` echoes the **throttle**, and the report has no brake entry.

| Index | Entry | `pwm` | `normalized` | `measured` | Decoded as |
|---|---|---|---|---|---|
| 0 | drive | throttle | demand in [-1, 1], 0 in the deadband | front-axle speed, rad/s | `drive_speed` |
| 1 | steering | steer | [-1, 1] | angle, rad, positive turning right | `steering_angle` |
| 2 | lift | lift | fraction of the range, [0, 1] | carriage height, m | `lift_height` |
| 3 | fork width | fork width | fraction of the range, [0, 1] | centre to centre, m | `fork_width` |

Rust and Python name the two orders `forklift_channel` (the command) and
`forklift_actuator` (the report). C++ has the same indices as `VRSDK_FORKLIFT_CH_*` and
`VRSDK_FORKLIFT_ACT_*`.

`forklift_actuators()` reads the latest snapshot and never blocks. It refuses a handle that
was not connected as a forklift, because the decode is by index and another robot's report
would read as wrong numbers.

## The brake

The fifth channel is bottom-anchored: 1100 is released and 1900 is full, with a 20 µs
deadband above 1100. It brakes all four wheels whatever the throttle says, where the idle
brake engages only with the throttle centred.

The brake latches with the rest of the frame, and a four-channel frame releases it. There is
no brake entry in the report, so the proof is the drive speed falling:

```text
   brake 1900 us: drive +1.95 -> +0.00 rad/s in 0.06 s
```

> **Gotcha.** Omitting `brake_us`, or calling `set_forklift` in Rust, is not "leave the
> brake alone". The four-channel frame releases a brake that an earlier five-channel frame
> latched. A controller that brakes must
> keep sending the fifth channel for as long as it wants the brake held.

## What reset re-latches

`reset()` teleports the forklift home and re-latches `[1500, 1500, 1100, 1301]`: straight,
stopped, forks down and 0.453 m apart, brake released. It **snaps** the mast there instead
of slewing it. The four reset pulses are configurable, see
[Forklift and pallet robot configuration](../ch06-services/09-warehouse-config.md).

A Ctrl-C mid-drive leaves the throttle latched and the forklift keeps driving, so let a
program run to its stop frame, or send one yourself.

**Next:** [Driving the pallet robot](12-palletrobot.md)

**See also:** [Commands latch](01-latching.md), [Forklift and pallet robot configuration](../ch06-services/09-warehouse-config.md), [Forklift](../ch07-robots/08-forklift.md), [Actuators](../ch03-reading-state/05-actuator.md)
