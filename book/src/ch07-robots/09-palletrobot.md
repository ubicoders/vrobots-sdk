# Pallet robot

A low mecanum robot that drives under a pedestal and lifts it on a turning pad, commanded one wheel at a time.

## Identity

| Property | Value |
|---|---|
| `RobotType` | `PalletRobot` |
| Catalog key | `palletrobot` |
| Synonyms | `pallet_robot` |
| Creatable | yes, on the Pro plan |
| Scene-authored | yes, the Warehouse scene ships one, pinned at `sys_id` 1 |
| Wire name | `palletrobot` |
| Default frame | `fru` |
| Type-specific service | `srv/palletrobot` |

The Warehouse scene pins its robots, so the pallet robot is `sys_id` 1 on every load, beside
the [forklift](08-forklift.md) at 0 and the [rail arm](10-rail-arm.md) at 2. Attach with
`connect(RobotType::PalletRobot, Some(1))`.

## Physical model

A flat body, 0.76 by 0.99 m and 0.134 m tall, of 180 kg. It never lifts a pallet directly:
it drives under a pedestal, raises it on a 0.70 by 0.70 m pad, and the pallet rides on the
pedestal.

It moves mecanum style, in any direction and turning on the spot, from **four wheel
commands**. The robot takes no twist, so the mixing is yours. The wheels are virtual: the
model has no rollers, and the geometry below is a set of chosen values that the robot's own
drive mixes against.

| Symbol | Value | Meaning |
|---|---|---|
| R | 0.054 m | wheel radius |
| lx | 0.35 m | centre to the front and rear axles |
| ly | 0.30 m | centre to the left and right wheels |
| ω max | 28 rad/s | wheel speed at 400 µs off neutral |

With `vx` forward and `vy` right in m/s, and `wz` clockwise seen from above in rad/s:

```text
FL = (vx + vy + (lx+ly)·wz) / R        FR = (vx - vy - (lx+ly)·wz) / R
RL = (vx - vy + (lx+ly)·wz) / R        RR = (vx + vy - (lx+ly)·wz) / R
us = 1500 + 400 · w / 28               for each wheel speed w in rad/s
```

If any wheel would pass 28 rad/s, scale all four by the same factor, because clipping one
alone changes the direction of travel. The limits that follow are 1.51 m/s forward or
sideways, 1.07 m/s along a diagonal, and 2.33 rad/s turning.

Each physics step the drive pushes the body toward the twist those wheel speeds imply, with
an acceleration capped at 6 m/s², about 1080 N of push against an obstacle. With all four
wheels stopped the robot also holds its parked position, like braked motors.

## Commands accepted

| Command | Method | Units and range | Status |
|---|---|---|---|
| `SET_PALLETROBOT` (314) | `set_palletrobot(fl_us, fr_us, rl_us, rr_us, pad_lift_us, pad_turn_us)` | µs per channel | live |

Exactly six pulse widths, all sent every time:

| # | Channel | 1100 | 1500 | 1900 | Notes |
|---|---|---|---|---|---|
| 0 to 3 | wheels FL, FR, RL, RR | -28 rad/s, backward | stop | +28 rad/s, forward | ±20 µs deadband; ramps at 60 rad/s² |
| 4 | `pad_lift_us` | 0 m, down | 0.03 m | 0.06 m, top | a position servo, no deadband; moves at 0.03 m/s |
| 5 | `pad_turn_us` | -90 deg/s | hold | +90 deg/s, clockwise from above | a rate, ±20 µs deadband |

The SDK refuses a value outside 1100 to 2000 µs before publishing, and the simulator clamps
each channel to 1100 to 1900. The frame latches: the wheels keep turning until a frame stops
them. The deadband makes about 0.08 m/s the slowest controllable speed.

The `pad` helper moves the pad with the wheels stopped and waits for the report to say it
arrived:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex38_hello_palletrobot.rs`:

```rust
fn pad(robot: &VirtualRobot, pad_us: f64) -> Result<(), VrError> {
    // A position servo with no deadband: linear from 0 m at 1100 to the top at 1900.
    let want = PAD_MAX_M * (pad_us - 1100.0) / 800.0;
    println!("\n-- pad lift {pad_us} us: {want:.3} m by the band --");
    let t0 = robot.states().elapsed;
    for i in 0..PAD_SAMPLES {
        robot.set_palletrobot(
            NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, pad_us, NEUTRAL_US,
        )?;
        robot.rate(HZ);
        let a = robot.palletrobot_actuators()?;
        let t = robot.states().elapsed - t0;
        if (a.pad_height - want).abs() < PAD_TOLERANCE_M {
            println!("   t={t:5.2}s  {}  -- there", report(&a));
            return Ok(());
        }
        if i % 10 == 0 {
            println!("   t={t:5.2}s  {}", report(&a));
        }
    }
    println!(
        "   not there after {:.0} s: has srv/palletrobot changed the pad?",
        f64::from(PAD_SAMPLES) / HZ
    );
    Ok(())
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex38_hello_palletrobot.cpp`:

```cpp
void pad(vrsdk::VirtualRobot& robot, double pad_us) {
    // A position servo with no deadband: linear from 0 m at 1100 to the top at 1900.
    const double want = PAD_MAX_M * (pad_us - 1100.0) / 800.0;
    std::printf("\n-- pad lift %g us: %.3f m by the band --\n", pad_us, want);
    const double t0 = robot.states().elapsed;
    for (int i = 0; i < PAD_SAMPLES; ++i) {
        robot.set_palletrobot(NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, pad_us, NEUTRAL_US);
        robot.rate(HZ);
        const vrsdk::PalletRobotActuators a = robot.palletrobot_actuators();
        const double t = robot.states().elapsed - t0;
        if (std::fabs(a.pad_height - want) < PAD_TOLERANCE_M) {
            std::printf("   t=%5.2fs  %s  -- there\n", t, report(a).c_str());
            return;
        }
        if (i % 10 == 0) {
            std::printf("   t=%5.2fs  %s\n", t, report(a).c_str());
        }
    }
    std::printf("   not there after %.0f s: has srv/palletrobot changed the pad?\n",
                PAD_SAMPLES / HZ);
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex38_hello_palletrobot.py`:

```python
def pad(robot, pad_us):
    """Move the pad to `pad_us`, wheels stopped, until the report says it is there."""
    # A position servo with no deadband: linear from 0 m at 1100 to the top at 1900.
    want = PAD_MAX_M * (pad_us - 1100.0) / 800.0
    print(f"\n-- pad lift {pad_us:g} us: {want:.3f} m by the band --")
    t0 = robot.states.elapsed
    for i in range(PAD_SAMPLES):
        robot.set_palletrobot(NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, pad_us, NEUTRAL_US)
        robot.rate(HZ)
        a = robot.palletrobot_actuators()
        t = robot.states.elapsed - t0
        if abs(a.pad_height - want) < PAD_TOLERANCE_M:
            print(f"   t={t:5.2f}s  {report(a)}  -- there")
            return
        if i % 10 == 0:
            print(f"   t={t:5.2f}s  {report(a)}")
    print(f"   not there after {PAD_SAMPLES / HZ:.0f} s: has srv/palletrobot changed the pad?")
```

{{#endtab }}
{{#endtabs }}

The whole 0.06 m takes two seconds, and the last line marks the arrival:

```text
-- pad lift 1900 us: 0.060 m by the band --
   t= 0.04s  wheels [ +0.0,  +0.0,  +0.0,  +0.0] rad/s  pad 0.001 m  pad angle +0.0 deg
   t= 0.41s  wheels [ +0.0,  +0.0,  +0.0,  +0.0] rad/s  pad 0.012 m  pad angle +0.0 deg
   t= 0.82s  wheels [ +0.0,  +0.0,  +0.0,  +0.0] rad/s  pad 0.025 m  pad angle +0.0 deg
   t= 1.24s  wheels [ +0.0,  +0.0,  +0.0,  +0.0] rad/s  pad 0.037 m  pad angle +0.0 deg
   t= 1.64s  wheels [ +0.0,  +0.0,  +0.0,  +0.0] rad/s  pad 0.049 m  pad angle +0.0 deg
   t= 1.99s  wheels [ +0.0,  +0.0,  +0.0,  +0.0] rad/s  pad 0.059 m  pad angle +0.0 deg  -- there
```

## Services

The common eight plus `srv/palletrobot`, called through `configure_palletrobot`. Every field
is optional and takes effect on the next physics step. The simulator heals an out-of-range
value and still acknowledges `ok`, and no field can be read back.

| Field | Units | Default | Notes |
|---|---|---|---|
| `pwm_band` | µs | 1100 / 1500 / 1900, deadband 20 | one setting of four numbers; not `min < neutral < max` resets the band |
| `wheel_radius` | m | 0.054 | 0 or less reverts to the default |
| `half_wheelbase` | m | 0.35 | lx; 0 or less reverts to the default |
| `half_track` | m | 0.30 | ly; 0 or less reverts to the default |
| `max_wheel_speed` | rad/s | 28 | at 400 µs off neutral; 0 or less reverts to the default |
| `wheel_accel` | rad/s² | 60 | below 0 becomes 0, a wheel that jumps to its command |
| `max_traction_accel` | m/s² | 6 | 0 or less reverts to the default |
| `parked_hold_gain` | 1/s | 4 | below 0 becomes 0, no hold |
| `pad_lift_max` | m | 0.06 | 0 or less reverts to the default; capped at 0.08 |
| `pad_lift_speed` | m/s | 0.03 | below 0 becomes 0, an instant pad |
| `pad_turn_max_rate` | rad/s | 1.571 | below 0 becomes 0 |

`reset()` teleports the robot home and re-latches the rest frame
`[1500, 1500, 1500, 1500, 1100, 1500]`: stopped, pad down, pad turn holding. The full field
reference is on
[The forklift and the pallet robot](../ch06-services/09-warehouse-config.md).

## Hardware

The drawings are to scale, and every position is measured from the robot origin O, on the
floor under the centre of the body. The labels read up, forward and back, which is also how
the robot's `fru` frame counts: x forward, y right, z up.

[![Pallet robot side view: both lidars and all three cameras in one plane 0.07 m above the floor, under a pedestal](images/palletrobot-side.svg)](images/palletrobot-side.svg)

*Side view, forward to the right. The dashed green line is the scan plane, 0.07 m above the
floor: inside the body's vertical faces and below the pad. The cameras sit in the faces at
the same height, tilted 10 degrees up, and the shaded sectors are what they see, out to the
floor. The dashed frame is a pedestal, whose deck underside is at 0.148 m.*

[![Pallet robot top view: a lidar on the front left corner and one on the rear right corner, the stereo pair in the front face and one camera in the rear face](images/palletrobot-top.svg)](images/palletrobot-top.svg)

*Top view, forward to the right and the robot's left up. Each green sector is one lidar's
270 degree outward view; the 90 degrees it cannot use point into the body. Together the two
cover the full circle. The camera wedges are the 56.1 degree horizontal field of view, the solid outline is the lift pad, and the small squares are the legs of a 1.10 by 1.13 m
pedestal.*

[![Pallet robot rear view: both lidars and the cameras at one height, 0.07 m above the floor](images/palletrobot-rear.svg)](images/palletrobot-rear.svg)

*Rear view, looking forward, the robot's left on the left. Both lidars share one scan
plane and the cameras sit at the same height, between them. The robot is 0.134 m tall, so
nothing can be mounted on top: the pad and the pedestal need that space.*

### Actuators

One order for the command and the report alike.

| # | Device | Kind | Driven by | `pwm` | `normalized` | `measured` | Range and rate limit |
|---|---|---|---|---|---|---|---|
| 0 to 3 | wheels FL, FR, RL, RR | `WHEEL` | command channels 0 to 3 | clamped pulse | demand in [-1, 1], 0 in the deadband | wheel motor speed, rad/s | ±28 rad/s; ramps at 60 rad/s² |
| 4 | pad lift | `LINEAR` | `pad_lift_us`, command channel 4 | clamped pulse | commanded travel in [0, 1] | pad height, m | 0 to 0.06 m; 0.03 m/s |
| 5 | pad turn | `SERVO` | `pad_turn_us`, command channel 5, a **rate** | clamped pulse | rate demand in [-1, 1] | pad angle against the body, rad, wrapped to [-π, π) | ±90 deg/s (1.571 rad/s); the angle wraps |

`palletrobot_actuators()` decodes `measured` into `wheel_speeds`, `pad_height` and
`pad_angle`. The wheel speeds are the motors' own, so a blocked robot still reports them:
`kin` is what the body did. See
[Actuator layouts and typed readers](../ch03-reading-state/10-actuator-layouts.md).

### Sensors

The standard five are fitted, all at the robot origin.

| Sensor | Measures | Units | Rate | Mount | Frame | What to know |
|---|---|---|---|---|---|---|
| Accelerometer | specific force on the body axes | m/s² | every state sample | robot origin, no offset, no rotation | the robot's, `fru` | +1 g at rest and 0 in free fall; clips without warning at ±2 g (19.61 m/s²); resolution about 0.0006 m/s²; white noise, a turn-on bias and a slowly drifting bias |
| Gyroscope | body rates | rad/s | every state sample | robot origin, no offset, no rotation | the robot's, `fru` | clips at ±250 deg/s (4.36 rad/s); resolution about 0.00013 rad/s; a turn-on bias of about 0.3 deg/s, one sigma, plus drift |
| Magnetometer | ambient magnetic field on the body axes | gauss | every state sample | robot origin, no offset, no rotation | the robot's, `fru` | clips at ±49.1 G; resolution 0.0015 G; a fixed turn-on offset of about 0.05 G, one sigma |
| Barometer | static pressure, and the pressure altitude computed from it | Pa, m | every state sample | robot origin | none: the readings are scalars | pressure noise of 3 Pa, about 0.25 m near sea level; the altitude is computed from the noisy pressure |
| GNSS | geodetic position and velocity | deg, deg, m and m/s | every state sample | robot origin, no antenna offset | geographic in every frame: latitude, longitude, altitude, and velocity as north, east, down | position noise of 1.5 m horizontal and 3.0 m vertical, velocity noise of 0.05 m/s horizontal and 0.08 m/s vertical, one sigma |
| Optical flow | horizontal ground velocity on the body axes | m/s | every state sample, once fitted | robot origin | the robot's, `fru` | **not fitted by default**; fit it through sensor config; noise of 0.05 m/s per axis |

The noise figures are the simulator's defaults, and every one of them is settable: see
[Sensor noise](../ch06-services/03-sensor-config.md). The fields each device publishes are on
[Sensors](../ch03-reading-state/03-sensors.md). A device keeps the robot's frame unless you
give it one of its own through [Coordinate frames](../ch06-services/04-frames-config.md).
<!-- VERIFY: GNSS update rate. The Sensors page describes a receiver at about 5 Hz; the simulator's notes describe a fresh fix on every physics step. -->

### Cameras

Three cameras, all 0.07 m above the floor, in the body's vertical faces.

| Name | Mount in `fru`, m | Orientation | Stream | Focal lengths | Field of view | Clip | Rate | Plan |
|---|---|---|---|---|---|---|---|---|
| `front_left` | (0.48, -0.10, 0.07), front face | forward, pitched 10 degrees up: the floor from 0.34 m out, pallet pockets, pedestal legs | 720p rgba8, 1280 by 720 | fx 1200 px, fy 900 px | 56.1 by 43.6 degrees | 0.1 to 1000 m | every rendered frame, about 60 per second | Community and Pro |
| `front_right` | (0.48, 0.10, 0.07), front face | the same view from 0.20 m to the right, the stereo baseline | 720p rgba8, 1280 by 720 | fx 1200 px, fy 900 px | 56.1 by 43.6 degrees | 0.1 to 1000 m | every rendered frame, about 60 per second | Pro |
| `rear` | (-0.486, 0, 0.07), rear face | backward, pitched 10 degrees up: what the robot reverses into | 720p rgba8, 1280 by 720 | fx 1200 px, fy 900 px | 56.1 by 43.6 degrees | 0.1 to 1000 m | every rendered frame, about 60 per second | Pro |

The lenses sit 7 mm inside the body faces, and the 0.1 m near clip is what keeps the body
out of the picture. Open them with `open_camera("front_left", "720p", "rgba8")`. On the Community plan only `front_left`
streams. See [Cameras and images](../ch05-cameras/00-intro.md).

### Lidars

Two 2-D lidars on opposite corners, in one scan plane.

| Name | Kind | Mount in `fru`, m | Orientation | Beam layout | Range | Rate | Points frame | Plan |
|---|---|---|---|---|---|---|---|---|
| `lidar_front_left` | 2-D | (0.40, -0.30, 0.07) | yaw -45 degrees: index 0 points out along the front left diagonal | a full ring of 360 ranges, 1 degree apart; faces outward over the front and the left | 0.15 to 12 m | 10 scans a second | `flu`, whatever the robot's frame | Pro |
| `lidar_rear_right` | 2-D | (-0.40, 0.30, 0.07) | yaw 135 degrees: index 0 points out along the rear right diagonal | a full ring of 360 ranges, 1 degree apart; faces outward over the back and the right | 0.15 to 12 m | 10 scans a second | `flu`, whatever the robot's frame | Pro |

The 90 degrees facing inward look through the robot's own body, so mask any beam whose
azimuth is more than 135 degrees from zero. Together the pair covers the full circle, and
under a pedestal the scan plane meets its four legs. Open one with
`open_lidar_2d("lidar_front_left")`; both stream over zenoh. See
[Lidar](../ch09-lidar/00-intro.md).

### Body and frame

| Property | Value |
|---|---|
| Mass | 180 kg |
| Centre of mass in `fru` | (0, 0, 0.06) m |
| Footprint | 0.76 by 0.99 m |
| Height | 0.134 m |
| Lift pad | 0.70 by 0.70 m, rising 0.06 m |
| Robot origin | on the floor, under the centre of the body |
| Default frame | `fru`: x forward, y right, z up |

Everything is SI except the command, which is microseconds. The `axis_convention` tag reads
unspecified because `fru` has no `Axes` constant. In `fru`, `kin.lin_vel[0]`,
`kin.lin_vel[1]` and `kin.ang_vel[2]` are exactly the `vx`, `vy` and `wz` of the mecanum
formula, so a positive `ang_vel[2]` is a turn to the right, clockwise seen from above.

The wheel channels are physical, positive rolling forward, and are never frame-converted. If
you override the robot's frame with `set_frames`, convert the twist yourself.

## Known quirks

> **Gotcha.** The latched frame survives a pulse band change and is re-mapped onto the new
> band on the next step, so a frame latched at the old neutral can start the wheels. After
> changing `pwm_band`, send a fresh frame or call `reset()`, which re-latches the rest frame
> from the new band.

Four more:

- **Pad turn slips under a load.** The pad rotates and the pedestal stays put. Turning the
  whole robot does carry the load, so turn the robot instead.
- The geometry cannot be read off the wire. `srv/palletrobot` sets R, lx and ly and nothing
  publishes them, so keep the values you send.
- There is no airborne check: a robot off the ground still drives. The centre of mass
  ignores the load.
- The deadband is not capped. One wide enough to reach both ends of the band silences the
  wheels and the pad turn.

## Example

ex38 mixes a strafe and a turn on the spot into wheel pulses, undoes each, then raises and
lowers the pad:

```sh
cargo run -p vrobots-examples --bin ex38_hello_palletrobot
./target/cpp-build/ex38_hello_palletrobot
python examples/python/ex38_hello_palletrobot.py
```

It takes an optional `sys_id` and defaults to 1, the Warehouse pallet robot. It never
deletes the robot, because it did not create it, and its last frame stops the wheels. A
Ctrl-C does not, and leaves the last frame latched.

**Next:** [Rail arm](10-rail-arm.md)

**See also:** [Driving the pallet robot](../ch04-commands/12-palletrobot.md), [The forklift and the pallet robot](../ch06-services/09-warehouse-config.md), [Lidar](../ch09-lidar/00-intro.md), [Actuators](../ch03-reading-state/05-actuator.md)
