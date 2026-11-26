# Half drone

A bar on a pivot with a rotor at each end, controlled by exactly two pulse widths.

## Identity

| Property | Value |
|---|---|
| `RobotType` | `HalfDrone` |
| Catalog key | `halfdrone` |
| Synonyms | `half_drone` |
| Creatable | **no**, scene-authored only |
| Scene | sandbox |
| Type-specific service | none |

Attach with `connect(RobotType::HalfDrone, Some(sys_id))`. The id comes from
`vrobots topic list` and moves between sessions.

## Physical model

A multirotor cut in half: a bar on a pivot with a rotor at each end, free to **roll** about
the FRD forward axis with the two arms lying along the left-right axis, and constrained in
everything else. It exists so that attitude control can be taught without six degrees of
freedom arguing back.

```text
I * theta'' = L2 * F2 - L1 * F1 + g * cos(theta) * m * (L1 - L2) - c * theta'
```

The only input is the difference between the two pulse widths. The bar dead-stops at a
mechanical travel limit of **70 degrees each side** by default, so a large enough difference
parks it against the stop and nothing more.

| Index | Rotor | Effect |
|---|---|---|
| 0 | rotor1, the FRD **left** arm | driving it high rolls FRD roll **positive** |
| 1 | rotor2, the FRD **right** arm | driving it high rolls it back |

Each arm is 3.0 m long by default, with a 1.0 kg motor at its tip, and the pivot has a
damping of 0.5 N·m·s/rad. Mass and inertia are settable through `set_physical_params` like
any other robot's; the arm lengths and the damping have no service at all. The full list is
under [Hardware](#hardware).

## Commands accepted

| Command | Method | Units and range | Status |
|---|---|---|---|
| `SET_MR_PWM` (300) | `set_mr_pwm_n(&[left_us, right_us])` | µs, exactly two entries | live |

That is the entire control surface. `SET_MR_PWM` carries one entry per rotor and the robot
refuses any other count with a warning no client can see, so `set_mr_pwm`, whose fixed
`[f64; 4]` is the quad's shape, is published happily and dropped there. A wrong count leaves
the previous command latched.

> **Gotcha.** This airframe's band tops out at **1900 microseconds**, not the stock rotor's
> 2000. The SDK validates against the wider 1100 to 2000 because it does not know the
> airframe, so 1950 is accepted here and clamped there. Detect it by comparing what you sent
> with `actuator.pwm` in the state stream.

A fresh spawn, and every reset, latches `[1100, 1100]`.

## Services

The common eight and nothing else. Two rotors and a hinge need no configuration service, so
the half drone is the canonical example of a capability probe: asking it for `srv/rotors` is
a GET to a key nobody serves, and after the service timeout it answers `VrError::NoResponder`.

That timeout **is** the type discovery. Nothing in a state message names the robot's type,
and a command for the wrong type is silently ignored rather than refused, so if you need to
know what you are attached to, ask for a service only that type serves and time the answer.
The cost is real, since a probe is a wait for an answer that is not coming: budget it with
`ConnectOptions::service_timeout` rather than taking the eight second default. A probe is
also indistinguishable from a simulator that is not running, which `vrobots topic list`
tells apart.

There is a cheaper route that waits on no timeout. `read_properties(sys_id)` reads a
robot's [properties](../ch06-services/13-properties.md#reading-by-sys-id-with-no-handle)
with no handle, in about a second, and the actuator layout it carries says what kind of
device is behind every index. Where the layout settles which robot is at an id, use that.
The probe stays the tool for two robots that publish the same layout, and for learning
whether a service is served at all.

This type ships no skin catalog, so every skin request is a no-op.

## Hardware

The simulator's sources hold no layout drawing for the half drone. Its geometry is one bar
on a pivot: a rotor 3.0 m out along each arm, and every other device at the pivot, which is
the robot origin.

### Actuators

Two entries, one per rotor.

| # | Device | Kind | Driven by | `measured` | Range and rate limit |
|---|---|---|---|---|---|
| 0 | rotor1, on the `frd` **left** arm | rotor | entry 0 of `SET_MR_PWM` | rotor speed, rad/s | 1100 to 1900 µs, clamped above; no slew |
| 1 | rotor2, on the `frd` **right** arm | rotor | entry 1 of `SET_MR_PWM` | rotor speed, rad/s | 1100 to 1900 µs, clamped above; no slew |

A rotor makes about 1.2 kgf at 1900 µs against the 1 kg motor it carries, so with unequal
arms the longer one hangs low over almost the whole band. `actuator.pwm` echoes the pulse
each rotor is acting on.

### Sensors

The standard five are fitted, all at the pivot.

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

None by default. `mount_camera` is the way to get pixels off it, and `vrobots topic list`
shows what is streaming. See [Cameras and images](../ch05-cameras/00-intro.md).

### Lidars

None.

### Body and frame

| Property | Value |
|---|---|
| Bar mass | 2 kg |
| Motor and propeller mass | 1.0 kg at each arm tip |
| Arm lengths | 3.0 m to rotor1 and 3.0 m to rotor2, pivot to rotor |
| Pivot damping | 0.5 N·m·s/rad, the only thing that bounds the tilt rate |
| Travel | 70 degrees each side of level, a dead stop |
| Inertia | not documented |
| Default frame | `frd`: x forward, y right, z down |

<!-- VERIFY: half drone bar mass (2 kg) and moment of inertia. The mass is the shipped model's value; a scene may place its half drone with another. -->

The arm lengths, the motor mass and the damping have no service; mass and inertia are
settable through `set_physical_params`.

Everything is SI, and the interesting quantities are in `frd`. There are no Euler angles on
the state wire, so every consumer derives them the same way from `kin.quat`, ordered
`[x, y, z, w]`:

```text
roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))     rad, FRD
rate = kin.ang_vel[0]                              rad/s, FRD roll rate
```

Pose is world frame and twist is body frame, as on every robot. The half drone publishes
`frd` by default, and `State::coord_frame_id` is the authoritative name for the frame in
force.

## Known quirks

> **Gotcha.** `set_mr_pwm` returns `Ok(())` on this robot and does nothing. The SDK cannot
> know the rotor count, the simulator drops the wrong-length command with a log line, and
> the previously latched pulse widths stay in effect, so the failure looks exactly like a
> robot that is not responding. Read `actuator.pwm.len()` to learn the count, and use
> `set_mr_pwm_n`.

Two more:

- Because there is no type-specific service, there is no way to change the travel limit,
  the arm lengths or the hinge damping from the SDK.
- The pulse widths latch. A program that exits without idling both rotors leaves the bar
  driven against whatever it was last commanded.

## Example

ex30 sweeps the differential and prints the roll angle it produces. The Rust and C++
programs first run the capability probe and show a four-entry command being dropped. The
`sys_id` argument is required:

```sh
cargo run -p vrobots-examples --bin ex30_hello_halfdrone -- 4
./target/cpp-build/ex30_hello_halfdrone 4
python examples/python/ex30_hello_halfdrone.py 4
```

Take the id from `vrobots topic list`. The Rust and C++ programs shorten the service timeout
to three seconds for the probe. None of the three deletes the robot, and each idles both
rotors on the way out.

**Next:** [Global Hawk](06-globalhawk.md)

**See also:** [Driving a multirotor](../ch04-commands/02-multirotor.md), [Robot lifecycle](../ch06-services/01-lifecycle.md), [Actuators](../ch03-reading-state/05-actuator.md)
