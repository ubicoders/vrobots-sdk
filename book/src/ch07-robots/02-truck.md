# Truck

A four-wheeled ground vehicle with a configurable drivetrain.

## Identity

| Property | Value |
|---|---|
| `RobotType` | `Truck` |
| Catalog key | `truck` |
| Synonyms | `car` |
| Creatable | yes, in the sandbox catalog |
| Scene-authored | yes, the sandbox scene ships one |
| Type-specific service | `srv/drive` |

On a fresh boot straight into the Flatworld scene the scene truck is `sys_id` 0, with the
multirotor at 1. Confirm with `vrobots topic list`, because ids keep incrementing across
scene loads.

## Physical model

A rigid body on four wheel colliders, driven by a steering servo and a drive motor whose
parameters are all live: a change bites from the next physics step with no rebuild and no
dropout. Steering is limited in both angle and rate; drive is a torque per driven wheel
with a no-load wheel speed that sets top speed; braking is split into an idle brake, applied
at neutral throttle, and a commanded brake.

Drive mode selects which wheels the motor torque reaches: 2 for rear wheel drive, 4 for all
wheel drive.

The truck weighs 1500 kg by default, and its inertia is the one the simulator computes from
its collision shapes. As on every robot, a mass of zero or less sent through
`set_physical_params` leaves the default in place and an inertia triple that is not strictly
positive on all three axes leaves the computed tensor in place. The full list is under
[Hardware](#hardware).

There is nothing in the state message to read a drivetrain setting back from, so the way to
confirm a change is to measure: drive a steady full-lock circle and compare the turn radius,
`speed / yaw_rate`. A steering limit that halves must roughly double the radius.

## Commands accepted

| Command | Method | Units and range | Status |
|---|---|---|---|
| `SET_CAR` (304) | `set_car(steer, throttle, brake)` | µs per channel | live |

Every channel is a pulse width, and each has its own meaning at the ends of the band:

| Channel | 1100 | 1500 | 1900 |
|---|---|---|---|
| `steer` | full left | centre | full right |
| `throttle` | full reverse | stop, idle brake | full forward |
| `brake` | released | | full |

The brake channel is **bottom-anchored**: 1100 is released, not 1500. Passing `brake = None`
sends the two-channel form of the command, which brakes nothing. The SDK checks every
channel is finite and inside 1100 to 2000 microseconds before publishing.

Commands latch and are never acknowledged. The last `SET_CAR` stays in effect until the
next one arrives, so a truck left by a stopped controller keeps its last steer and throttle.

> **Gotcha.** The truck's factory pulse band is 1100 / 1500 / 1900, which is not the
> multirotor's 1100 to 2000. The SDK validates against the wider band because it does not
> know the airframe, so 1950 is accepted here and treated as full scale there.

## Services

The common eight plus `srv/drive`. Every field is optional, and the simulator substitutes
silently for anything out of range while still acknowledging `ok`.

| Field | Type | Units | Simulator behaviour |
|---|---|---|---|
| `drive_mode` | `Option<u32>` | | must be 2 or 4, else ignored; the SDK refuses anything else first |
| `max_steer_deg` | `Option<f64>` | deg | hard-clamped to 0 to 60 |
| `steer_rate_dps` | `Option<f64>` | deg/s | 0 means an ideal instantaneous servo |
| `max_motor_torque_nm` | `Option<f64>` | N·m | per driven wheel |
| `no_load_wheel_rpm` | `Option<f64>` | rpm | at full throttle with no load, so it sets top speed; zero or less becomes 200 |
| `idle_brake_torque_nm` | `Option<f64>` | N·m | per wheel |
| `max_brake_torque_nm` | `Option<f64>` | N·m | per wheel |
| `pwm_band` | `Option<PwmBand>` | µs | factory 1100 / 1500 / 1900 |

`PwmBand { min_us, neutral_us, max_us, deadband_us }` is four `u32` microsecond values. If
`min < neutral < max` does not hold, the simulator replaces the whole band with the factory
values.

Skins are available: `black`, `blue`, `camouflage`, `gray`, `red`. A key from another
robot's catalog, `gold` for instance, is acknowledged `ok` and dropped with a log line, so a
typo looks like success.

## Hardware

The drawings are to scale, on a 0.5 m grid, and every position is measured from the robot
origin O, which is on the ground between the wheels. All five skins share this outline. The
labels read up and forward, which is also how the robot's `fru` frame counts: x forward,
y right, z up.

[![Truck side view: the stereo cameras above the windshield and a 3-D lidar on the roof](images/truck-side.svg)](images/truck-side.svg)

*Side view, forward to the right. The green wedges are the lidar's ring fan, 15 degrees
above and below level, drawn 1.5 m long; the lowest ring clears the roof, the roof fin and
the rack behind the cab. Both cameras share one position in this view.*

[![Truck top view: the stereo pair 0.50 m either side of the centreline and the lidar on the centreline](images/truck-top.svg)](images/truck-top.svg)

*Top view, forward to the right and the robot's left up. The dashed circle marks the
lidar's 360 degree sweep, and the camera wedges are the 56.1 degree horizontal field of
view.*

[![Truck rear view: both cameras at 2.00 m and the lidar at 2.05 m above the ground](images/truck-rear.svg)](images/truck-rear.svg)

*Rear view, looking forward, the robot's left on the left. The antenna on the left
A-pillar, the truck's highest point at 1.97 m, is the only part that reaches into the
lidar's rings.*

### Actuators

Five entries: the four wheels, then the steering servo. There is no brake entry.

| # | Device | Kind | Driven by | `measured` | Range and rate limit |
|---|---|---|---|---|---|
| 0 to 3 | wheels front left, front right, rear left, rear right | wheel | the `throttle` channel of `SET_CAR`, on the driven wheels only | wheel speed, rad/s | no-load speed 200 rpm (20.9 rad/s) at full throttle; up to 800 N·m per driven wheel |
| 4 | steering | servo | the `steer` channel of `SET_CAR` | steering angle, rad | ±30 degrees by default, settable up to 60; slews at 300 deg/s |

The default drive mode is 2, rear wheel drive. An undriven wheel reports a `pwm` of 0 and a
demand of 0, and still reports its measured speed, because the road turns it. Idle braking
is 400 N·m per wheel at neutral throttle and the commanded brake reaches 2500 N·m per wheel.
Every one of these numbers is a default of [The truck drivetrain](../ch06-services/05-drive-config.md).

### Sensors

The standard five are fitted, all at the robot origin. Naming a sensor the robot does not
carry in a sensor config request is skipped with a simulator log line and still
acknowledged `ok`.

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

A forward stereo pair above the windshield, 0.25 m above the roof surface and 1.00 m apart.

| Name | Mount in `fru`, m | Orientation | Stream | Focal lengths | Field of view | Clip | Rate | Plan |
|---|---|---|---|---|---|---|---|---|
| `front_left` | (0.70, -0.50, 2.00) | straight ahead along +x, no tilt | 720p rgba8, 1280 by 720 | fx 1200 px, fy 900 px | 56.1 by 43.6 degrees | 0.1 to 1000 m | every rendered frame, about 60 per second | Community and Pro |
| `front_right` | (0.70, 0.50, 2.00) | straight ahead along +x, no tilt | 720p rgba8, 1280 by 720 | fx 1200 px, fy 900 px | 56.1 by 43.6 degrees | 0.1 to 1000 m | every rendered frame, about 60 per second | Pro |

`open_camera` attaches to either; mount more with `mount_camera` when that pair cannot
serve. On the Community plan only the first camera of the focused robot streams. Each frame
carries the intrinsics and the mount pose it was rendered with, so read them from
`Frame::intrinsics` and do not hard-code the table. See
[Cameras and images](../ch05-cameras/00-intro.md).

### Lidars

| Name | Kind | Mount in `fru`, m | Orientation | Beam layout | Range | Rate | Points frame | Plan |
|---|---|---|---|---|---|---|---|---|
| `lidar` | 3-D | (0.10, 0, 2.05) | spin axis up, no rotation | 16 rings from -15 to +15 degrees, 2 degrees apart; 900 columns per revolution, 0.4 degrees apart; 14 400 ranges a scan | 0.4 to 100 m | 20 scans a second | `flu`, whatever the robot's frame | Pro |

Open it with `open_lidar_3d("lidar")`; it streams over iceoryx2. A lidar point (x, y, z) is
at (x + 0.10, -y, z + 2.05) in the truck's `fru` frame. On flat ground the lowest ring first
meets the ground 7.65 m from the sensor. The whip antenna crosses the rings from -15 to
about -4 degrees in a sliver about 3 degrees wide, 48 degrees left of forward; the rest of
the truck's own body is looked through. See [Lidar](../ch09-lidar/00-intro.md).

### Body and frame

| Property | Value |
|---|---|
| Mass | 1500 kg |
| Centre of mass in `fru` | (0.16, 0, 0.50) m |
| Inertia | computed by the simulator from the body's collision shapes; not documented |
| Robot origin | on the ground, between the wheels |
| Highest point | the antenna on the left A-pillar, 1.97 m |
| Default frame | `fru`: x forward, y right, z up |

<!-- VERIFY: default moments of inertia for the truck, and its overall length and width. `properties()` reads the live mass properties. -->

Everything is SI. Pose (`lin_pos`, `quat`) is world frame and twist and acceleration
(`lin_vel`, `ang_vel`, `lin_acc`, `ang_acc`) are body frame. Quaternions are ordered
`[x, y, z, w]`. Moments of inertia sent through `srv/params` are read in your header frame
and permuted into the robot's own.

The truck publishes `fru`, where the third component is up, while the multirotor publishes
`frd`, where it is down. Both were measured live. No per-robot-type native
`coord_frame_id` exists in the SDK source, so the tag on the snapshot is the authority: in
code read `State::coord_frame_id`.

## Known quirks

> **Gotcha.** A skin swap on the truck is not purely cosmetic. The wheel colliders travel
> with the skin prefab, so changing the skin rebinds the physics wheels. Detect it by
> re-measuring a turn radius after the swap rather than assuming the drivetrain is
> untouched.

Two smaller ones:

- `max_steer_deg` is clamped rather than refused. Asking for 90 degrees gets 60, and the
  acknowledgement is `ok` either way; the circle the truck drives is the only evidence.
- Created trucks have real physics, confirmed live. The frozen-dynamics bug that affects
  created multirotors does not affect this type.

## Example

Drive it with ex05:

```sh
cargo run -p vrobots-examples --bin ex05_hello_car
./target/cpp-build/ex05_hello_car
python examples/python/ex05_hello_car.py
```

Retune the drivetrain under a moving vehicle and measure the result with ex26, and try the
skin catalog with ex23:

```sh
cargo run -p vrobots-examples --bin ex26_drive_config
./target/cpp-build/ex26_drive_config
python examples/python/ex26_drive_config.py
cargo run -p vrobots-examples --bin ex23_skins
./target/cpp-build/ex23_skins
python examples/python/ex23_skins.py
```

Neither takes an argument: both create the robot they need.

**Next:** [Mass spring damper](03-msd.md)

**See also:** [Driving the truck](../ch04-commands/03-truck.md), [The truck drivetrain](../ch06-services/05-drive-config.md), [Skins](../ch06-services/08-skins.md)
