# Forklift

A counterbalance forklift driven by pulse widths, with a mast whose lift and fork width are position servos.

## Identity

| Property | Value |
|---|---|
| `RobotType` | `Forklift` |
| Catalog key | `forklift` |
| Synonyms | none |
| Creatable | yes, on the Pro plan |
| Scene-authored | yes, the Warehouse scene ships one, pinned at `sys_id` 0 |
| Wire name | `forklift` |
| Default frame | `fru` |
| Type-specific service | `srv/forklift` |

The Warehouse scene pins its robots, so the forklift is `sys_id` 0 on every load, with the
[pallet robot](09-palletrobot.md) at 1 and the [rail arm](10-rail-arm.md) at 2. Attach with
`connect(RobotType::Forklift, Some(0))`. A forklift you create gets whatever id the manager
assigns and appears at the scene origin unless you give it a spawn pose; see
[Known simulator issues](07-known-issues.md).

## Physical model

A rigid body of 4000 kg on four wheels, with a wheelbase of 1.83 m. The **front pair drives
and the rear pair steers**, so the tail swings out in a turn. Steering reaches 45 degrees
each way and the unloaded top speed is about 5 m/s.

The mast carries two position servos. Lift is the carriage height, and fork width is the
distance between the forks, centre to centre. Neither is a rate: every microsecond of the
pulse band is a place on the mast, and the mast **slews** there, the lift at 0.5 m/s and the
width at 0.1 m/s. A full change of width therefore takes about 12 s.

The lift range starts below zero on purpose. At 1100 µs the carriage sits at -0.215 m, which
puts the blades 4.0 to 8.7 cm above the floor, the height of a floor pallet's pockets.

Mass and inertia are settable through `set_physical_params`, and the live values are on
`properties()`; see [Robot properties](../ch06-services/13-properties.md).

## Commands accepted

| Command | Method | Units and range | Status |
|---|---|---|---|
| `SET_FORKLIFT` (312) | `set_forklift(steer_us, throttle_us, lift_us, fork_width_us)` | µs per channel | live |
| `SET_FORKLIFT` (312) | the same call with a fifth `brake_us` argument | µs per channel | live |

The frame is four pulse widths, or five with the brake:

| # | Channel | 1100 | 1500 | 1900 | Notes |
|---|---|---|---|---|---|
| 0 | `steer_us` | full left | straight | full right, 45 degrees | ±20 µs deadband around 1500 |
| 1 | `throttle_us` | full reverse | stop, idle brake | full forward, about 5 m/s | ±20 µs deadband around 1500 |
| 2 | `lift_us` | -0.215 m | 0.448 m | 1.11 m | linear, no deadband |
| 3 | `fork_width_us` | 0.158 m | 0.746 m | 1.334 m | linear, no deadband |
| 4 | `brake_us`, optional | released | half | full | **bottom-anchored**: 1100 is off |

The SDK refuses a value outside 1100 to 2000 µs before publishing; the simulator clamps
anything above 1900, and the echo shows the clamped pulse. Nothing is frame-converted.

The frame latches as a whole. A four-channel frame **releases the brake**, and the brake
stays on only while a five-channel frame is latched. Every drive frame carries the mast
pulses too, so a frame that sent different ones would move the forks.

The `drive` helper holds one throttle with the forks where they are, and reads the decoded
report back:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex37_hello_forklift.rs`:

```rust
fn drive(
    robot: &VirtualRobot,
    label: &str,
    throttle_us: f64,
    lift_us: f64,
    width_us: f64,
) -> Result<(), VrError> {
    println!(
        "\n-- {label}: throttle {throttle_us} us for {:.1} s --",
        f64::from(DRIVE_SAMPLES) / HZ
    );
    for i in 0..DRIVE_SAMPLES {
        robot.set_forklift(NEUTRAL_US, throttle_us, lift_us, width_us)?;
        robot.rate(HZ);
        if i % 10 == 9 {
            println!("   {}", report(&robot.forklift_actuators()?));
        }
    }
    Ok(())
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex37_hello_forklift.cpp`:

```cpp
void drive(vrsdk::VirtualRobot& robot, const char* label, double throttle_us, double lift_us,
           double width_us) {
    std::printf("\n-- %s: throttle %g us for %.1f s --\n", label, throttle_us, DRIVE_SAMPLES / HZ);
    for (int i = 0; i < DRIVE_SAMPLES; ++i) {
        robot.set_forklift(NEUTRAL_US, throttle_us, lift_us, width_us);
        robot.rate(HZ);
        if (i % 10 == 9) {
            std::printf("   %s\n", report(robot.forklift_actuators()).c_str());
        }
    }
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex37_hello_forklift.py`:

```python
def drive(robot, label, throttle_us, lift_us, width_us):
    """Hold one throttle, straight ahead, with the forks where they are.

    A frame carries all four channels, so the mast pulses ride along in every
    drive frame: one that left them out would move the forks as well.
    """
    print(f"\n-- {label}: throttle {throttle_us:g} us for {DRIVE_SAMPLES / HZ:.1f} s --")
    for i in range(DRIVE_SAMPLES):
        robot.set_forklift(NEUTRAL_US, throttle_us, lift_us, width_us)
        robot.rate(HZ)
        if i % 10 == 9:
            print(f"   {report(robot.forklift_actuators())}")
```

{{#endtab }}
{{#endtabs }}

Each printed line is the report in readable units:

```text
-- forward: throttle 1600 us for 2.5 s --
   lift +0.447 m  width 0.746 m  drive +0.58 rad/s (+0.21 m/s)  steering  +0.0 deg
   lift +0.447 m  width 0.746 m  drive +1.02 rad/s (+0.37 m/s)  steering  +0.0 deg
   lift +0.447 m  width 0.746 m  drive +1.41 rad/s (+0.51 m/s)  steering  +0.0 deg
   lift +0.447 m  width 0.746 m  drive +1.70 rad/s (+0.61 m/s)  steering  +0.0 deg
   lift +0.447 m  width 0.746 m  drive +1.95 rad/s (+0.70 m/s)  steering  +0.0 deg
```

## Services

The common eight plus `srv/forklift`, called through `configure_forklift`. Every field is
optional and takes effect on the next physics step, except the `initial_*_us` pulses, which
wait for the next reset. The simulator heals an out-of-range value and still acknowledges
`ok`, and no field can be read back.

| Field | Units | Default | Notes |
|---|---|---|---|
| `max_motor_torque` | N·m | 1200 | per front wheel; below 0 becomes 0 |
| `no_load_wheel_speed` | rad/s | 13.93 | sets top speed; 0 or less reverts to the default |
| `idle_brake_torque` | N·m | 800 | applied at neutral throttle; below 0 becomes 0 |
| `max_brake_torque` | N·m | 2000 | at full brake; below 0 becomes 0 |
| `max_steer_angle` | rad | 0.785 | clamped to 0 to 1.396 rad (80 degrees) |
| `steer_slew_rate` | rad/s | 1.047 | below 0 becomes 0, an instant servo |
| `lift_min`, `lift_max` | m | -0.215, 1.11 | `max <= min` resets both |
| `lift_speed` | m/s | 0.5 | 0 or less reverts to the default; capped at 1 |
| `fork_width_min`, `fork_width_max` | m | 0.158, 1.334 | `min < 0` or `max <= min` resets both |
| `fork_width_speed` | m/s | 0.1 | 0 or less reverts to the default; capped at 0.5 |
| `pwm_band` | µs | 1100 / 1500 / 1900, deadband 20 | one setting of four numbers; not `min < neutral < max` resets the band |
| `initial_steer_us`, `initial_throttle_us`, `initial_lift_us`, `initial_fork_width_us` | µs | 1500, 1500, 1100, 1301 | the frame a reset re-latches; 0 means the default; clamped into the band |

A range is judged one request at a time, so to move a range past its other end, send both
ends together. A band or range change re-maps the latched pulses on the next step: lift and
width slew to the new target, while steer, throttle and brake re-map at once.

`reset()` teleports the forklift home and re-latches `[1500, 1500, 1100, 1301]` with the
brake released. The mast is **snapped** to that frame, not slewed: 1301 µs is about 0.453 m,
the spacing of a pallet's pocket centres. The full field reference is on
[The forklift and the pallet robot](../ch06-services/09-warehouse-config.md).

## Hardware

The drawings are to scale, on a 0.5 m grid, and every position is measured from the robot
origin O, which is on the floor under the front axle. The labels read up, forward and back,
which is also how the robot's `fru` frame counts: x forward, toward the forks, y right,
z up.

[![Forklift side view: a 3-D lidar above the mast, the stereo cameras at the mast top looking 42 degrees down, and a rear camera on the overhead guard](images/forklift-side.svg)](images/forklift-side.svg)

*Side view, forward to the right. The mast is drawn part raised, at a lift of 0.6 m with
the forks 1.03 m apart; the robot spawns with the forks lowered. The dashed lines are the
blade tops at spawn (0.09 m) and at full lift (1.41 m), and the highest the backrest
reaches (2.58 m). The shaded sectors are what the cameras see, out to the floor, and the
green wedges are the lidar's ring fan, drawn 1.5 m long. The two front cameras share one
position in this view.*

[![Forklift top view: the lidar on the centreline above the mast, the front cameras 0.15 m either side of it, and the rear camera on the centreline](images/forklift-top.svg)](images/forklift-top.svg)

*Top view, forward to the right and the robot's left up. The dashed circle marks the
lidar's 360 degree sweep; the unit sits on a bracket across the top of the fixed outer
mast. The camera wedges are the 56.1 degree horizontal field of view.*

[![Forklift rear view: the lidar at 2.90 m, the front cameras below its lowest ring, and the rear camera at 2.15 m](images/forklift-rear.svg)](images/forklift-rear.svg)

*Rear view, looking forward, the robot's left on the left. Nothing on the forklift is
higher than the lidar, so no ring is blocked in any direction. The front cameras sit 0.14 m
below its lowest ring.*

### Actuators

The report is **not in command order**. It leads with the drive, so `actuator.pwm[0]` echoes
the throttle, and it has no brake entry.

| # | Device | Kind | Driven by | `pwm` | `normalized` | `measured` | Range and rate limit |
|---|---|---|---|---|---|---|---|
| 0 | drive, front pair | `WHEEL` | `throttle_us`, command channel 1 | throttle µs | demand in [-1, 1], 0 in the deadband | front axle speed, rad/s | no-load 13.93 rad/s, about 5 m/s; up to 1200 N·m per front wheel |
| 1 | steering, rear pair | `SERVO` | `steer_us`, command channel 0 | steer µs | [-1, 1] | rad, positive turns right | ±45 degrees (0.785 rad); slews at 60 deg/s (1.047 rad/s) |
| 2 | lift | `LINEAR` | `lift_us`, command channel 2 | lift µs | [0, 1] of the range | carriage height, m | -0.215 to 1.11 m; 0.5 m/s |
| 3 | fork width | `LINEAR` | `fork_width_us`, command channel 3 | width µs | [0, 1] of the range | centre to centre, m | 0.158 to 1.334 m; 0.1 m/s, so 0.05 m/s per fork |

`forklift_actuators()` decodes `measured` into `drive_speed`, `steering_angle`, `lift_height`
and `fork_width`. That decoded report is the confirmation of a mast move, because `pwm`
echoes a new pulse on the next sample while the forks are still travelling. The brake, command
channel 4, reaches 2000 N·m at full and has no entry: its proof is `drive_speed` falling to
zero. See [Actuator layouts and typed readers](../ch03-reading-state/10-actuator-layouts.md).

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

Three cameras, all fixed to the body, so none rides the lift.

| Name | Mount in `fru`, m | Orientation | Stream | Focal lengths | Field of view | Clip | Rate | Plan |
|---|---|---|---|---|---|---|---|---|
| `front_left` | (0.55, -0.15, 2.70), at the mast top | forward, pitched 42 degrees down: the floor ahead and both fork tips | 720p rgba8, 1280 by 720 | fx 1200 px, fy 900 px | 56.1 by 43.6 degrees | 0.1 to 1000 m | every rendered frame, about 60 per second | Community and Pro |
| `front_right` | (0.55, 0.15, 2.70) | the same view from 0.30 m to the right, the stereo baseline | 720p rgba8, 1280 by 720 | fx 1200 px, fy 900 px | 56.1 by 43.6 degrees | 0.1 to 1000 m | every rendered frame, about 60 per second | Pro |
| `rear` | (-1.78, 0, 2.15), on the overhead guard | backward, pitched 38 degrees down: the floor behind, with the counterweight edge at the bottom of the picture | 720p rgba8, 1280 by 720 | fx 1200 px, fy 900 px | 56.1 by 43.6 degrees | 0.1 to 1000 m | every rendered frame, about 60 per second | Pro |

Open them with `open_camera("front_left", "720p", "rgba8")`. On the Community plan only
`front_left` streams. See [Cameras and images](../ch05-cameras/00-intro.md).

### Lidars

| Name | Kind | Mount in `fru`, m | Orientation | Beam layout | Range | Rate | Points frame | Plan |
|---|---|---|---|---|---|---|---|---|
| `lidar` | 3-D | (0.37, 0, 2.90), on a bracket across the fixed outer mast top | spin axis up, no rotation | 16 rings from -15 to +15 degrees, 2 degrees apart; 900 columns per revolution, 0.4 degrees apart; 14 400 ranges a scan | 0.4 to 100 m | 20 scans a second | `flu`, whatever the robot's frame | Pro |

The lidar never moves with the lift, and nothing on the forklift is higher. On a flat floor
the lowest ring reads 11.2 m and first meets the ground about 10.8 m out, so the fork tips
and a pallet directly ahead are the cameras' job. Open it with `open_lidar_3d("lidar")`.
See [Lidar](../ch09-lidar/00-intro.md).

### Body and frame

| Property | Value |
|---|---|
| Mass | 4000 kg |
| Centre of mass in `fru` | (-1.222, 0, 0.6) m: 1.22 m behind the front axle, so the load splits about one third front, two thirds rear |
| Wheelbase | 1.83 m, front axle at the robot origin |
| Wheel radius | 0.36 m front, 0.29 m rear |
| Outer mast top | 2.791 m |
| Robot origin | on the floor, under the front axle |
| Default frame | `fru`: x forward, toward the forks, y right, z up |

Everything is SI except the command, which is microseconds. `fru` has no `Axes` constant, so
the `axis_convention` tag beside it reads unspecified. Change the frame with `set_frames`,
and read `State::coord_frame_id` for the frame in force. Mass and inertia are settable
through `set_physical_params`, and the live values are on `properties()`.

## Known quirks

> **Gotcha.** The brake channel is bottom-anchored. 1100 µs is released and 1900 µs is full,
> so a brake of 1500 is half applied, not off. The report has no brake entry: the proof that
> a brake landed is `drive_speed` falling to zero.

Four more:

- Everything latches and there is no watchdog. A program interrupted mid-drive leaves the
  throttle latched and the forklift keeps driving.
- The centre of mass does not move when the mast lifts or when the forks carry a load.
- Mast tilt and side shift are not modelled, and the visible wheels show spin and steer but
  not suspension travel.
- The forklift stands on four rigid wheel stations, so its footprint is the full wheel
  rectangle and not a real forklift's stability triangle.

## Example

ex37 raises and widens the forks, drives forward for 2.5 s and back for 2.5 s with the brake
between, and puts the mast back as it found it. The drive is open loop, so it does not end
where it began: one live run finished 0.56 m from its start.

```sh
cargo run -p vrobots-examples --bin ex37_hello_forklift
./target/cpp-build/ex37_hello_forklift
python examples/python/ex37_hello_forklift.py
```

It takes an optional `sys_id` and defaults to 0, the Warehouse forklift. ex44 reads the
forklift's properties and halves its lift speed through `srv/forklift`, then restores it:

```sh
cargo run -p vrobots-examples --bin ex44_robot_properties
./target/cpp-build/ex44_robot_properties
python examples/python/ex44_robot_properties.py
```

Neither deletes the forklift, because neither created it. Let both run to the end.

**Next:** [Pallet robot](09-palletrobot.md)

**See also:** [Driving the forklift](../ch04-commands/11-forklift.md), [The forklift and the pallet robot](../ch06-services/09-warehouse-config.md), [Lidar](../ch09-lidar/00-intro.md), [Actuators](../ch03-reading-state/05-actuator.md)
