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

The airframe's mass, inertia, arm lengths and damping are not documented in the SDK;
confirm against a live simulator. Mass and inertia are settable through
`set_physical_params` like any other robot's, and the arm lengths and damping have no
service at all.
<!-- VERIFY: default mass, inertia, arm lengths L1 and L2, and damping c for the half drone. -->

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

The common seven and nothing else. Two rotors and a hinge need no configuration service, so
the half drone is the canonical example of a capability probe: asking it for `srv/rotors` is
a GET to a key nobody serves, and after the service timeout it answers `VrError::NoResponder`.

That timeout **is** the type discovery. Nothing in a state message names the robot's type,
and a command for the wrong type is silently ignored rather than refused, so if you need to
know what you are attached to, ask for a service only that type serves and time the answer.
The cost is real, since a probe is a wait for an answer that is not coming: budget it with
`ConnectOptions::service_timeout` rather than taking the eight second default. A probe is
also indistinguishable from a simulator that is not running, which `vrobots topic list`
tells apart.

This type ships no skin catalog, so every skin request is a no-op.

## Frame and units

Everything is SI, and the interesting quantities are in FRD. There are no Euler angles on
the state wire, so every consumer derives them the same way from `kin.quat`, ordered
`[x, y, z, w]`:

```text
roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))     rad, FRD
rate = kin.ang_vel[0]                              rad/s, FRD roll rate
```

Pose is world frame and twist is body frame, as on every robot, and `State::coord_frame_id`
is the authoritative name for the frame this robot publishes in. No per-robot-type native
`coord_frame_id` is defined in the SDK source, so which frame the half drone comes up in is
not documented in the SDK; confirm against a live simulator.
<!-- VERIFY: the half drone's native coord_frame_id. The physical model is described in FRD terms; nothing in the SDK source defines a per-robot default. -->

Which sensors it carries is not documented in the SDK; confirm against a live simulator.
<!-- VERIFY: sensor availability for the HalfDrone. Nothing in the repository enumerates it. -->

## Cameras

Nothing is camera-specific to this robot type. Where a robot carries the default pair,
`front_left` and `front_right` at 720p rgba8, `open_camera` attaches to either without
changing anything; this one may carry none, in which case `mount_camera` is the only way to
get pixels off it. `vrobots topic list` is the authority. See
[Cameras and images](../ch05-cameras/00-intro.md).
<!-- VERIFY: whether the scene-authored half drone ships the two default cameras. -->

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

ex30 runs the capability probe, shows a four-entry command being dropped, then sweeps the
differential and prints the roll angle it produces. The `sys_id` argument is required:

```sh
cargo run -p vrobots-examples --bin ex30_hello_halfdrone -- 4
./target/cpp-build/ex30_hello_halfdrone 4
python examples/python/ex30_hello_halfdrone.py 4
```

Take the id from `vrobots topic list`. The example shortens the service timeout to three
seconds for the probe, never deletes the robot, and idles both rotors on the way out.

**Next:** [Global Hawk](06-globalhawk.md)

**See also:** [Driving a multirotor](../ch04-commands/02-multirotor.md), [Robot lifecycle](../ch06-services/01-lifecycle.md), [Actuators](../ch03-reading-state/05-actuator.md)
