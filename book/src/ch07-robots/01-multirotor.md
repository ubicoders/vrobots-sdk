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

The prefab's default mass and inertia are not documented in the SDK; confirm against a live
simulator. The only documented behaviour is that a mass of zero or less leaves the prefab's
value in place, and that an inertia triple which is not strictly positive on all three axes
leaves Unity's collider-derived tensor in place.
<!-- VERIFY: default prefab mass and moments of inertia for the multirotor. Not present anywhere in the repository. -->

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

The common seven plus `srv/rotors`, which carries the whole rotor list. The list is
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
`srv/skin` is the only service that ever answers `ok = false`, and it does so because the
catalog is tier-gated, which is why a refusal must not be retried.

## Frame and units

Everything is SI. Within `Kinematics`, pose (`lin_pos`, `quat`) is world frame while twist
and acceleration (`lin_vel`, `ang_vel`, `lin_acc`, `ang_acc`) are body frame, in both the
truth block and the estimate block. Quaternions are ordered `[x, y, z, w]`.

Rotor positions and moments of inertia are read in **your** header frame, the one you set
through `ConnectOptions::coord_frame_id` (default `"unity"`), and permuted into the robot's
own. Read `State::coord_frame_id` for the frame the robot publishes in; it is
authoritative.

The multirotor publishes `frd`, measured live. No per-robot-type native `coord_frame_id`
exists in the SDK source, so the tag on the snapshot is the authority: read
`State::coord_frame_id` in code.

Which sensors this airframe carries is not documented in the SDK; confirm against a live
simulator. `SensorConfig` only guarantees that naming a sensor the robot does not carry is
skipped with a simulator log line and still acknowledged `ok`.
<!-- VERIFY: sensor availability matrix for the multirotor. Nothing in the repository enumerates it. -->

## Cameras

Nothing is camera-specific to this robot type. Like every vrobot it ships `front_left` and
`front_right` at 720p rgba8, which is what `open_camera` attaches to; mount more with
`mount_camera` when that pair cannot serve. Per-robot camera intrinsics are not documented
in the SDK; intrinsics are whatever `CameraOptions` requested, read back through
`Frame::intrinsics`. See [Cameras and images](../ch05-cameras/00-intro.md).
<!-- VERIFY: whether the scene-authored multirotor ships the same two default cameras as every other robot in the test scene. -->

## Known quirks

> **Sim bug.** A multirotor created through `srv/create` publishes and serves normally and
> its actuator echo is live, but its rigidbody never integrates: it does not fall under
> gravity, does not climb under thrust, and `srv/reset` teleports it and then it freezes
> again. Simulator v3.0.0, open, `issues/created-multirotor-frozen-dynamics.md`. Scene
> multirotors are unaffected, so the workaround is to attach to one by `sys_id`. The full
> account, including what attaching costs you, is on
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
with ex22. Both take an **optional** `sys_id`: pass one to attach to the scene multirotor,
omit it to create a robot instead.

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
