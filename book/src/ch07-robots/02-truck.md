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

The prefab's default mass and inertia are not documented in the SDK; confirm against a live
simulator. As on every robot, a mass of zero or less leaves the prefab's value in place and
an inertia triple that is not strictly positive on all three axes leaves Unity's
collider-derived tensor in place.
<!-- VERIFY: default prefab mass and moments of inertia for the truck. Not present anywhere in the repository. -->

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

The common seven plus `srv/drive`. Every field is optional, and the simulator substitutes
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

## Frame and units

Everything is SI. Pose (`lin_pos`, `quat`) is world frame and twist and acceleration
(`lin_vel`, `ang_vel`, `lin_acc`, `ang_acc`) are body frame. Quaternions are ordered
`[x, y, z, w]`. Moments of inertia sent through `srv/params` are read in your header frame
and permuted into the robot's own.

The truck publishes `fru`, where the third component is up, while the multirotor publishes
`frd`, where it is down. Both were measured live. No per-robot-type native
`coord_frame_id` exists in the SDK source, so the tag on the snapshot is the authority: in
code read `State::coord_frame_id`.

Which sensors this vehicle carries is not documented in the SDK; confirm against a live
simulator. Naming a sensor the robot does not carry is skipped with a simulator log line
and still acknowledged `ok`.
<!-- VERIFY: sensor availability matrix for the truck. Nothing in the repository enumerates it. -->

## Cameras

Nothing is camera-specific to this robot type. Like every vrobot it ships `front_left` and
`front_right` at 720p rgba8, which is what `open_camera` attaches to; mount more with
`mount_camera` when that pair cannot serve. Per-robot camera intrinsics are not documented
in the SDK; they are whatever `CameraOptions` requested, read back through
`Frame::intrinsics`. See [Cameras and images](../ch05-cameras/00-intro.md).
<!-- VERIFY: whether the scene-authored truck ships the same two default cameras as the multirotor in the test scene. -->

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
