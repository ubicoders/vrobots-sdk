# Driving the pallet robot

`SET_PALLETROBOT` carries six pulse widths, one per wheel plus two for the pad, and turning a
body velocity into the four wheel values is your job.

```sh
cargo run -p vrobots-examples --bin ex38_hello_palletrobot
./target/cpp-build/ex38_hello_palletrobot
python examples/python/ex38_hello_palletrobot.py
```

The pallet robot is the low mecanum robot that drives under a pedestal and lifts it on a
turning pad. The warehouse scene pins it at `sys_id` 1.

## The signature


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`crates/vrobots-sdk/src/commands.rs`:

```rust
pub fn set_palletrobot(
    &self,
    fl_us: f64,
    fr_us: f64,
    rl_us: f64,
    rr_us: f64,
    pad_lift_us: f64,
    pad_turn_us: f64,
) -> VrResult<()>
```

{{#endtab }}
{{#tab name="C++" }}

`include/vrobots_sdk.hpp`:

```cpp
void set_palletrobot(double fl_us, double fr_us, double rl_us, double rr_us,
                     double pad_lift_us, double pad_turn_us)
```

{{#endtab }}
{{#tab name="Python" }}

`vrsdk/_vrsdk.pyi`:

```python
def set_palletrobot(
    self,
    fl_us: float,
    fr_us: float,
    rl_us: float,
    rr_us: float,
    pad_lift_us: float,
    pad_turn_us: float,
) -> None: ...
```

{{#endtab }}
{{#endtabs }}

The six values go on the wire as `int_arr` under command id `SET_PALLETROBOT` (314). The
frame is exactly six entries, and the robot ignores any other length.

## The six channels

The command and the actuator report use one order, unlike the
[forklift](11-forklift.md#the-report-is-not-in-command-order).

| Index | Channel | Units | 1100 | 1500 | 1900 | Notes |
|---|---|---|---|---|---|---|
| 0 | front-left wheel | µs | -28 rad/s | stop | +28 rad/s, rolling forward | ±20 µs deadband |
| 1 | front-right wheel | µs | -28 rad/s | stop | +28 rad/s | ±20 µs deadband |
| 2 | rear-left wheel | µs | -28 rad/s | stop | +28 rad/s | ±20 µs deadband |
| 3 | rear-right wheel | µs | -28 rad/s | stop | +28 rad/s | ±20 µs deadband |
| 4 | pad lift | µs | 0 m | 0.03 m | 0.06 m | a position, no deadband |
| 5 | pad turn | µs | -90 deg/s | hold | +90 deg/s, clockwise from above | a rate, ±20 µs deadband |

The SDK refuses a value outside 1100 to 2000 µs. The simulator clamps each value to 1100 to
1900, and the echo shows the clamped pulse. The wheel channels are physical (positive rolls
forward) and are never frame-converted.

Rust names the indices `palletrobot_channel::FL` through `palletrobot_channel::PAD_TURN`,
Python `palletrobot_channel.FL` through `palletrobot_channel.PAD_TURN`, and C++ has
`VRSDK_PALLETROBOT_CH_*`.

## The robot takes no twist

There is no "go right at 0.3 m/s" command. Each wheel has its own speed controller, and
mixing a body twist into four wheel speeds is the sender's job. With wheel radius
`R` = 0.054 m, half wheelbase `lx` = 0.35 m and half track `ly` = 0.30 m, the forward mix
is:

```text
FL = (vx + vy + (lx + ly) * wz) / R        FR = (vx - vy - (lx + ly) * wz) / R
RL = (vx - vy + (lx + ly) * wz) / R        RR = (vx + vy - (lx + ly) * wz) / R
us = round(1500 + 400 * w / 28)            for each wheel speed w in rad/s
```

`vx` is forward and `vy` is right, both in m/s, and `wz` is clockwise seen from above in
rad/s. That is FRU, the frame this robot reports in by default, so the state's
`kin.lin_vel[0]`, `kin.lin_vel[1]` and `kin.ang_vel[2]` are the same three numbers coming
back.

The inverse recovers the body twist from four wheel speeds, for example from the measured
ones in the actuator report:

```text
vx = R * ( FL + FR + RL + RR) / 4
vy = R * ( FL - FR - RL + RR) / 4
wz = R * ( FL - FR + RL - RR) / (4 * (lx + ly))
```

The geometry is virtual, because the model has no rollers, and nothing on the wire reads it
back. If [`configure_palletrobot`](../ch06-services/09-warehouse-config.md) has changed the
radius, the half wheelbase, the half track or the top wheel speed, your constants must
follow.

## Scale all four, never clip one

If any wheel would pass 28 rad/s, scale all four by the same factor. Clipping only the fast
wheel changes the ratio between the wheels, and that ratio is the direction of travel.

From `ex38_hello_palletrobot`, the mix with its scaling step:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex38_hello_palletrobot.rs`:

```rust
fn mix(vx: f64, vy: f64, wz: f64) -> [f64; 4] {
    let k = HALF_WHEELBASE_M + HALF_TRACK_M; // lx + ly
    let mut wheels = [
        (vx + vy + k * wz) / WHEEL_RADIUS_M,
        (vx - vy - k * wz) / WHEEL_RADIUS_M,
        (vx - vy + k * wz) / WHEEL_RADIUS_M,
        (vx + vy - k * wz) / WHEEL_RADIUS_M,
    ];

    // Too fast for one wheel: slow all four by the same factor, which keeps the
    // direction of travel. Clipping only the fast one would bend it.
    let mut fastest = 0.0_f64;
    for w in wheels {
        fastest = fastest.max(w.abs());
    }
    if fastest > MAX_WHEEL_RAD_S {
        for w in &mut wheels {
            *w *= MAX_WHEEL_RAD_S / fastest;
        }
    }
    wheels
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex38_hello_palletrobot.cpp`:

```cpp
std::array<double, 4> mix(double vx, double vy, double wz) {
    const double k = HALF_WHEELBASE_M + HALF_TRACK_M;  // lx + ly
    std::array<double, 4> wheels = {
        (vx + vy + k * wz) / WHEEL_RADIUS_M,
        (vx - vy - k * wz) / WHEEL_RADIUS_M,
        (vx - vy + k * wz) / WHEEL_RADIUS_M,
        (vx + vy - k * wz) / WHEEL_RADIUS_M,
    };

    // Too fast for one wheel: slow all four by the same factor, which keeps the
    // direction of travel. Clipping only the fast one would bend it.
    double fastest = 0.0;
    for (const double w : wheels) {
        fastest = std::fmax(fastest, std::fabs(w));
    }
    if (fastest > MAX_WHEEL_RAD_S) {
        for (double& w : wheels) {
            w *= MAX_WHEEL_RAD_S / fastest;
        }
    }
    return wheels;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex38_hello_palletrobot.py`, the body of its `mix(vx, vy, wz)` function:

```python
k = HALF_WHEELBASE_M + HALF_TRACK_M  # lx + ly
wheels = [
    (vx + vy + k * wz) / WHEEL_RADIUS_M,
    (vx - vy - k * wz) / WHEEL_RADIUS_M,
    (vx - vy + k * wz) / WHEEL_RADIUS_M,
    (vx + vy - k * wz) / WHEEL_RADIUS_M,
]

# Too fast for one wheel: slow all four by the same factor, which keeps the
# direction of travel. Clipping only the fast one would bend it.
fastest = 0.0
for w in wheels:
    fastest = max(fastest, abs(w))
if fastest > MAX_WHEEL_RAD_S:
    wheels = [w * (MAX_WHEEL_RAD_S / fastest) for w in wheels]
return wheels
```

{{#endtab }}
{{#endtabs }}

For a strafe to the right at 0.3 m/s the example prints the mix before it drives:

```text
-- strafe right: vx +0.00 m/s, vy +0.30 m/s, wz +0.00 rad/s --
   mix: [FL, FR, RL, RR] = [+5.56, -5.56, -5.56, +5.56] rad/s -> [1579, 1421, 1421, 1579] us
```

## Send whole microseconds

The wire carries whole microseconds. The SDK rounds a fractional value to the nearest one,
and a half rounds up: 1579.4 goes out as 1579, 1420.6 as 1421 and 1500.5 as 1501, so the two
sides of a strafe stay mirror images whatever you hand it. The band is checked first, on the
value as you gave it: 2000.4 is refused, not rounded down to 2000.

Round each wheel pulse yourself all the same. The SDK would send the same pulse, but then
the number your program prints or logs is the one that went out, and the one the `pwm` echo
shows.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex38_hello_palletrobot.rs`:

```rust
fn to_us(rad_per_s: f64) -> f64 {
    (NEUTRAL_US + 400.0 * rad_per_s / MAX_WHEEL_RAD_S).round()
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex38_hello_palletrobot.cpp`:

```cpp
double to_us(double rad_per_s) {
    return std::round(NEUTRAL_US + 400.0 * rad_per_s / MAX_WHEEL_RAD_S);
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex38_hello_palletrobot.py`:

```python
return round(NEUTRAL_US + 400.0 * rad_per_s / MAX_WHEEL_RAD_S)
```

{{#endtab }}
{{#endtabs }}

All three return a whole number of microseconds, 1579 and 1421 for the strafe above. On an
exact half Python's `round` picks the even neighbour, where `std::round`, Rust's `round`
and the SDK pick the one further from zero; both are whole microseconds.

## Deadband, ramps and confirmation

The deadband on the wheels makes about 0.08 m/s the slowest controllable speed. The wheels
ramp at 60 rad/s² and the pad climbs at 0.03 m/s, so confirm a move through
`palletrobot_actuators()` and not through the echo.

| Decoded field | Units | Meaning |
|---|---|---|
| `wheel_speeds` | rad/s | the four motor speeds, FL, FR, RL, RR, positive rolling forward |
| `pad_height` | m | pad height above its rest, 0 to 0.06 |
| `pad_angle` | rad | pad angle relative to the body, in [-pi, pi), positive clockwise from above |

> **Gotcha.** The wheel speeds are the motors' own. A robot blocked by an obstacle still
> reports them, so compare them against `kin.lin_vel` to learn what the body did. The pad
> angle is likewise the pad's own: turning the pad under a load does not turn the load,
> because the pedestal slips. Turn the whole robot to turn a load.

## Stopping, and what reset re-latches

A zero twist mixes to 1500 on every wheel, so a stop is one more frame. With all four wheels
stopped the robot holds where it parked.

`reset()` re-latches `[1500, 1500, 1500, 1500, 1100, 1500]`: stopped, pad down and square.
It snaps the pad there instead of slewing it. A Ctrl-C leaves the last frame latched and the
wheels turning.

**Next:** [Arm joints and tools](13-arms.md)

**See also:** [Commands latch](01-latching.md), [Forklift and pallet robot configuration](../ch06-services/09-warehouse-config.md), [Pallet robot](../ch07-robots/09-palletrobot.md), [Actuators](../ch03-reading-state/05-actuator.md)
