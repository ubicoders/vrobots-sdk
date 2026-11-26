# Multirotor

Four rotors, direct pulse width control, and configurable thrust curves.

## Identity

| Property | Value |
|---|---|
| `RobotType` | `Multirotor` |
| Catalog key | `multirotor` |
| Synonyms | none |
| Creatable | yes, in the sandbox catalog |
| Scene-authored | yes, the sandbox scene ships one |
| Type-specific service | `srv/rotors` |

On a fresh boot straight into the Flatworld scene the scene multirotor is `sys_id` 1, with
the truck at 0. Ids are allocated at scene load and keep incrementing, so confirm with
`vrobots topic list` rather than relying on that.

## Physical model

A rigid body with one rotor per PWM channel. Each rotor turns its pulse width into a force
and a moment through three curves the simulator evaluates every physics step:

```text
thrust = (thrust_a * pwm^2 + thrust_b * pwm + thrust_c) * g      [N]
torque = spin_dir * (torque_a * pwm^2 + torque_b * pwm + torque_c) * g   [N·m]
omega  = ang_vel_slope * pwm + ang_vel_intercept                  [rad/s]
```

**There is no mixing matrix anywhere in the simulator.** Roll, pitch and yaw fall out of
the rotor positions alone, so moving a rotor genuinely changes the airframe's response and
an asymmetric aircraft is nothing more than a different rotor list. The rotor count is
fixed when the airframe spawns and equals `actuator.pwm.len()` in the state stream: read
it, do not assume four.

Nothing sits between your pulse widths and those curves. There is no attitude
stabilisation and no rate damping, so you are the flight controller.

The airframe weighs 1 kg by default, and its inertia is the one the simulator computes from
its collision shapes. A mass of zero or less sent through `set_physical_params` leaves the
default in place, and an inertia triple which is not strictly positive on all three axes
leaves the computed tensor in place. The full list is under [Hardware](#hardware).

## Commands accepted

| Command | Method | Units and range | Status |
|---|---|---|---|
| `SET_MR_PWM` (300) | `set_mr_pwm([f64;4])` | µs, 1100 to 2000 | live |
| `SET_MR_PWM` (300) | `set_mr_pwm_n(&[f64])` | µs, 1100 to 2000, one per rotor | live |
| `SET_MR_THROTTLE` (301) | `set_mr_throttle([f64;4])` | normalised | on the wire, nothing acts on it |

`[1100; 4]` is idle and a flying aircraft falls. Hover is wherever total thrust crosses
weight for the current mass and curves, so it moves when you change either. `set_mr_pwm`
delegates to `set_mr_pwm_n`, which rejects an empty slice and checks every value is finite
and inside the band.

Commands latch. The last pulse widths received stay in effect until the next ones arrive,
there is no watchdog, and no command is ever acknowledged: proof that one landed is
`actuator.pwm` echoing it back in the state stream.

> **Not yet.** `SET_MR_THROTTLE` exists on the wire and no robot type acts on it. Sending
> it returns `Ok(())` and changes nothing.

## Services

The common eight plus `srv/rotors`, which carries the whole rotor list. The list is
replaced rather than merged, so the slice you send must describe every rotor in index
order; a wrong-length slice makes the simulator drop the entire request and acknowledge
`ok` anyway. There is no read-back and no per-field flags inside an entry, so
`RotorSpec::default()`, the simulator's own reference rotor, is the base you build on.

| Field | Units | Default |
|---|---|---|
| `position` | m, from the robot origin, not the centre of mass | `[0, 0, 0]` |
| `spin_dir` | | `0.0`, meaning alternate by index with even indices clockwise; `+1` clockwise, `-1` counter-clockwise |
| `thrust_a` | | `7.5e-7` |
| `thrust_b` | | `-0.001325` |
| `thrust_c` | | `0.55` |
| `torque_a` | | `7.5e-8` |
| `torque_b` | | `-0.0001325` |
| `torque_c` | | `0.055` |
| `ang_vel_slope` | | `1.33` |
| `ang_vel_intercept` | | `-1466.67` |
| `pwm_min_us` | µs | `1100` |
| `pwm_max_us` | µs | `2000` |

The simulator substitutes 1100 to 2000 if the band is not strictly increasing. A bare
`default()` puts every rotor at `[0, 0, 0]`, which is an aircraft with thrust and no
control authority.

Skins are available: `blue`, `desert`, `gold`, `green`, `mono`, `pink`, `snow`, `white`.
`srv/skin` answers `ok = false` only because the catalog is tier-gated, which is why a
refusal must not be retried.

## Hardware

The drawings are to scale, on a 0.1 m grid, and every position is measured from the robot
origin O. The outline is the blue airframe, which the gold, green and white skins share.
The labels read as you see them, up and forward; the tables below give the same mounts in
the robot's `frd` frame, where z points down, so a height of 0.35 m is z = -0.35.

[![Multirotor side view: two cameras under the front of the airframe and a 3-D lidar on a short mast](images/multirotor-side.svg)](images/multirotor-side.svg)

*Side view, forward to the right. The green wedges are the lidar's ring fan, 15 degrees
above and below level, drawn 0.24 m long. The dashed lines are the rotor discs, whose tops
are 0.200 m above O, and the lowest ring passes over them. Both cameras share one position
in this view.*

[![Multirotor top view: the stereo pair 0.20 m either side of the centreline and the lidar at the centre](images/multirotor-top.svg)](images/multirotor-top.svg)

*Top view, forward to the right and the robot's left up. The four small dashed circles are
the rotor discs, 0.121 m in radius. The large dashed circle marks the lidar's 360 degree
sweep, and the camera wedges are the 56.1 degree horizontal field of view.*

[![Multirotor rear view: both cameras immediately below the airframe and the lidar above the body](images/multirotor-rear.svg)](images/multirotor-rear.svg)

*Rear view, looking forward, the robot's left on the left. The cameras sit immediately below the
airframe's lowest point, which is 0.108 m above O, and the lidar stands on a short mast
above the body.*

### Actuators

One entry per rotor, in rotor order. The shipped airframe has four.

| # | Device | Kind | Driven by | `measured` | Range and rate limit |
|---|---|---|---|---|---|
| 0 to n-1 | rotor | rotor | the `SET_MR_PWM` entry of the same index | rotor speed, rad/s | 1100 to 2000 µs. No slew: thrust, torque and speed follow the pulse on the next physics step |

With the default curves a rotor gives no thrust at 1100 µs and about 8.8 N at 2000 µs, where
it turns at 1193 rad/s. `actuator.pwm` echoes the pulse each rotor is acting on, which is
the only receipt a command has. Where each rotor sits is part of the airframe model and is
not published; `srv/rotors` replaces the whole list.
<!-- VERIFY: the shipped rotor positions and spin directions. They come from the airframe model and nothing in the sources lists them. -->

### Sensors

The standard five are fitted, all at the robot origin. Naming a sensor the robot does not
carry in a sensor config request is skipped with a simulator log line and still
acknowledged `ok`.

| Sensor | Measures | Units | Rate | Mount | Frame | What to know |
|---|---|---|---|---|---|---|
| Accelerometer | specific force on the body axes | m/s² | every state sample | robot origin, no offset, no rotation | the robot's, `frd` | +1 g at rest and 0 in free fall; clips without warning at ±2 g (19.61 m/s²); resolution about 0.0006 m/s²; white noise, a turn-on bias and a slowly drifting bias |
| Gyroscope | body rates | rad/s | every state sample | robot origin, no offset, no rotation | the robot's, `frd` | clips at ±250 deg/s (4.36 rad/s); resolution about 0.00013 rad/s; a turn-on bias of about 0.3 deg/s, one sigma, plus drift |
| Magnetometer | ambient magnetic field on the body axes | gauss | every state sample | robot origin, no offset, no rotation | the robot's, `frd` | clips at ±49.1 G; resolution 0.0015 G; a fixed turn-on offset of about 0.05 G, one sigma |
| Barometer | static pressure, and the pressure altitude computed from it | Pa, m | every state sample | robot origin | none: the readings are scalars | pressure noise of 3 Pa, about 0.25 m near sea level; the altitude is computed from the noisy pressure |
| GNSS | geodetic position and velocity | deg, deg, m and m/s | every state sample | robot origin, no antenna offset | geographic in every frame: latitude, longitude, altitude, and velocity as north, east, down | position noise of 1.5 m horizontal and 3.0 m vertical, velocity noise of 0.05 m/s horizontal and 0.08 m/s vertical, one sigma |
| Optical flow | horizontal ground velocity on the body axes | m/s | every state sample, once fitted | robot origin | the robot's, `frd` | **not fitted by default**; fit it through sensor config; noise of 0.05 m/s per axis |

The noise figures are the simulator's defaults, and every one of them is settable: see
[Sensor noise](../ch06-services/03-sensor-config.md). The fields each device publishes are on
[Sensors](../ch03-reading-state/03-sensors.md). A device keeps the robot's frame unless you
give it one of its own through [Coordinate frames](../ch06-services/04-frames-config.md).
<!-- VERIFY: GNSS update rate. The Sensors page describes a receiver at about 5 Hz; the simulator's notes describe a fresh fix on every physics step. -->

### Cameras

A forward stereo pair slung under the front of the airframe, 0.40 m apart.

| Name | Mount in `frd`, m | Orientation | Stream | Focal lengths | Field of view | Clip | Rate | Plan |
|---|---|---|---|---|---|---|---|---|
| `front_left` | (0.20, -0.20, -0.10) | straight ahead along +x, no tilt | 720p rgba8, 1280 by 720 | fx 1200 px, fy 900 px | 56.1 by 43.6 degrees | 0.1 to 1000 m | every rendered frame, about 60 per second | Community and Pro |
| `front_right` | (0.20, 0.20, -0.10) | straight ahead along +x, no tilt | 720p rgba8, 1280 by 720 | fx 1200 px, fy 900 px | 56.1 by 43.6 degrees | 0.1 to 1000 m | every rendered frame, about 60 per second | Pro |

`open_camera` attaches to either; mount more with `mount_camera` when that pair cannot
serve. On the Community plan only the first camera of the focused robot streams. Each frame
carries the intrinsics and the mount pose it was rendered with, so read them from
`Frame::intrinsics` and do not hard-code the table. See
[Cameras and images](../ch05-cameras/00-intro.md).

### Lidars

| Name | Kind | Mount in `frd`, m | Orientation | Beam layout | Range | Rate | Points frame | Plan |
|---|---|---|---|---|---|---|---|---|
| `lidar` | 3-D | (0, 0, -0.35) | spin axis up, no rotation | 16 rings from -15 to +15 degrees, 2 degrees apart; 900 columns per revolution, 0.4 degrees apart; 14 400 ranges a scan | 0.4 to 100 m | 20 scans a second | `flu`, whatever the robot's frame | Pro |

Open it with `open_lidar_3d("lidar")`; it streams over iceoryx2. The robot's own body is
looked through, and a beam with no return reads `+infinity`. See
[Lidar](../ch09-lidar/00-intro.md).

### Body and frame

| Property | Value |
|---|---|
| Mass | 1 kg |
| Centre of mass | at the robot origin |
| Inertia | computed by the simulator from the airframe's collision shapes; not documented |
| Robot origin | 0.108 m below the airframe's lowest point |
| Rotor discs | 0.121 m in radius, tops 0.200 m above the origin |
| Default frame | `frd`: x forward, y right, z down |

<!-- VERIFY: default moments of inertia for the multirotor. `properties()` reads the live values. -->

Everything is SI. Within `Kinematics`, pose (`lin_pos`, `quat`) is world frame while twist
and acceleration (`lin_vel`, `ang_vel`, `lin_acc`, `ang_acc`) are body frame, in both the
truth block and the estimate block. Quaternions are ordered `[x, y, z, w]`.

Rotor positions and moments of inertia are read in **your** header frame, the one you set
through `ConnectOptions::coord_frame_id` (default `"unity"`), and permuted into the robot's
own. The multirotor publishes `frd`, measured live. No per-robot-type native
`coord_frame_id` exists in the SDK source, so the tag on the snapshot is the authority: read
`State::coord_frame_id` in code.

## Known quirks

> **Sim bug.** A multirotor created through `srv/create` publishes and serves normally and
> its actuator echo is live, but its rigidbody never integrates: it does not fall under
> gravity, does not climb under thrust, and `srv/reset` teleports it and then it freezes
> again. Simulator v3.0.0, open. Scene multirotors are unaffected, so the workaround is to
> attach to one by `sys_id`. The full account, including what attaching costs you, is on
> [Known simulator issues](07-known-issues.md).

Two more worth knowing before you configure anything:

- Rotor positions are measured from the robot's **origin**, not its centre of mass. The
  simulator subtracts the centre of mass itself, so a centre-of-mass-relative value gets
  subtracted twice.
- Neither mass nor inertia appears in the state message. They are quasi-static
  configuration, so the only confirmation that `set_physical_params` landed is behavioural:
  the same pulse width has to produce a different acceleration.

## Example

Fly it open loop with [ex02](../ch01-getting-started/04-hello-control.md):

```sh
cargo run -p vrobots-examples --bin ex02_hello_control
./target/cpp-build/ex02_hello_control
python examples/python/ex02_hello_control.py
```

Rebuild the airframe under itself with ex27, and change the mass under a running controller
with ex22. Both take an **optional** `sys_id` in all three languages. Pass one to attach to
the scene multirotor, which the program then leaves running until the scene reloads, with the
new mass after ex22 and with a 1 kg mass and the rebuilt rotors after ex27. Omit it to create a
robot, which the program deletes at the end.

```sh
cargo run -p vrobots-examples --bin ex27_rotor_config -- 1
./target/cpp-build/ex27_rotor_config 1
python examples/python/ex27_rotor_config.py 1
cargo run -p vrobots-examples --bin ex22_physical_params -- 1
./target/cpp-build/ex22_physical_params 1
python examples/python/ex22_physical_params.py 1
```

Because of the frozen-dynamics bug above, pass the argument.

**Next:** [Truck](02-truck.md)

**See also:** [Driving a multirotor](../ch04-commands/02-multirotor.md), [Rotors and thrust curves](../ch06-services/06-rotor-config.md), [Mass and inertia](../ch06-services/02-physical-params.md)
