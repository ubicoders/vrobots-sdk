# Cart pole

A cart on a rail with an unactuated pole, and a service that owns the whole plant.

## Identity

| Property | Value |
|---|---|
| `RobotType` | `CartPole` |
| Catalog key | `cartpole` |
| Synonyms | `cart_pole`, `invpen` |
| Creatable | **no**, scene-authored only |
| Scene | sandbox |
| Type-specific service | `srv/cartpole` |

`connect(RobotType::CartPole, None)` is refused by `srv/create` with a message naming the
eight keys the sandbox scene does know: `multirotor`, `truck`, `msd`, `forklift`,
`palletrobot`, `kr210_rail`, `tablearm` and `turret`. The only route
in is `connect(RobotType::CartPole, Some(sys_id))`, and the id comes from
`vrobots topic list`.

## Physical model

A cart on a rail with a pole hinged on top. One actuator, a force on the cart, and two
things to control with it: that is what underactuated means, and why it is the textbook
problem. The pole is unactuated by design.

The simulator pins the cart's rotation to identity and freezes `y`, `z` and every axis of
rotation, so the rail really is the world `x` axis. `srv/cartpole` owns the entire plant,
cart mass included:

| Field | Units | Default |
|---|---|---|
| `cart_mass` | kg | 1.0 |
| `travel_half_range` | m, each side of spawn | 4.0 |
| `pole_rod_mass` | kg | 0.1 |
| `bob_mass` | kg, a tip point mass | 0.2 |
| `pole_length` | m, hinge to bob | 1.2 |
| `pole_angular_damping` | dimensionless, 0 is frictionless | 0.01 |
| `max_force` | N, the clamp on `set_cartpole_force` | 20 |
| `initial_pole_angle_deg` | degrees | -45 |

<!-- VERIFY: the defaults for pole_angular_damping (0.01) and initial_pole_angle_deg (-45) are the simulator's shipped values; a scene may place its cart pole with others. -->

There is no collider on the pole, so `pole_length` is the whole pendulum geometry.
Out-of-range values are not refused: the simulator silently replaces them with the defaults
above and acknowledges `ok`, which is why `configure_cartpole` refuses non-positive masses,
lengths and forces itself.

## Commands accepted

| Command | Method | Units and range | Status |
|---|---|---|---|
| `SET_INVPEN` (306) | `set_cartpole_force(f64)` | N along the rail's `+x` | live |

This is the **only** actuator on the robot. The SDK checks the value is finite; the
simulator clamps it to `CartPoleConfig::max_force`, 20 N by default.

The force latches, and that matters more here than anywhere else: a controller that stalls
for a second has not commanded zero, it has left its last force applied, and the pole is on
the floor.

## Services

The common eight plus `srv/cartpole`. The division of labour is unusual and worth stating
plainly: `srv/params` cannot set the cart's mass, because the robot re-stamps it from
`cart_mass` on every parameter apply, so a mass sent there is overwritten a step later. Use
`configure_cartpole`.

> **Gotcha.** `initial_pole_angle_deg` is in degrees while the state stream reports the pole
> angle in radians, and setting it re-seats the pole at rest **immediately** rather than at
> the next reset. It is the episode's initial condition, so sending it mid-swing stops the
> swing dead. The value is wrapped to `[-180, 180]`, where 0 is upright and plus or minus
> 180 is hanging.

`reset()` does both halves of a restart at once: the cart returns to the rail centre and the
pole is re-hung at rest at whatever home angle was last configured. This type ships no skin
catalog, so every skin request is a no-op.

## Hardware

The simulator's sources hold no layout drawing for the cart pole. The whole geometry is the
rail, the cart and a pole of `pole_length`, and every device sits at the cart's origin.

### Actuators

Three entries, exactly. None is a device with a kind of its own, so all three read
`unspecified` in the [published layout](../ch03-reading-state/10-actuator-layouts.md).

| # | Device | Kind | Driven by | `measured` | Range and rate limit |
|---|---|---|---|---|---|
| 0 | the force on the cart | unspecified | `SET_INVPEN` | the force applied, after the clamp, N | clamped to the plant's `max_force`, 20 N by default; applied on the next physics step |
| 1 | the pole hinge | unspecified | nothing: the pole is unactuated | pole angle theta, **rad** | wraps into `[-pi, pi]` |
| 2 | the pole hinge | unspecified | nothing | pole rate theta prime, rad/s | none |

Theta is the signed angle from world `+y` to the hinge-to-bob direction, right-handed about
`+z`, wrapped into `[-pi, pi]`. Zero is balanced upright and plus or minus pi is hanging,
both, because that is the wrap seam, so a fallen pole may print either sign. A positive
theta leans the bob toward `-x`.

### Sensors

The standard five are fitted, all at the cart's origin. They measure the **cart**: nothing
measures the pole except the two actuator entries above.

| Sensor | Measures | Units | Rate | Mount | Frame | What to know |
|---|---|---|---|---|---|---|
| Accelerometer | specific force on the body axes | m/s² | every state sample | robot origin, no offset, no rotation | the robot's, `unity` | +1 g at rest and 0 in free fall; clips without warning at ±2 g (19.61 m/s²); resolution about 0.0006 m/s²; white noise, a turn-on bias and a slowly drifting bias |
| Gyroscope | body rates | rad/s | every state sample | robot origin, no offset, no rotation | the robot's, `unity` | clips at ±250 deg/s (4.36 rad/s); resolution about 0.00013 rad/s; a turn-on bias of about 0.3 deg/s, one sigma, plus drift |
| Magnetometer | ambient magnetic field on the body axes | gauss | every state sample | robot origin, no offset, no rotation | the robot's, `unity` | clips at ±49.1 G; resolution 0.0015 G; a fixed turn-on offset of about 0.05 G, one sigma |
| Barometer | static pressure, and the pressure altitude computed from it | Pa, m | every state sample | robot origin | none: the readings are scalars | pressure noise of 3 Pa, about 0.25 m near sea level; the altitude is computed from the noisy pressure |
| GNSS | geodetic position and velocity | deg, deg, m and m/s | every state sample | robot origin, no antenna offset | geographic in every frame: latitude, longitude, altitude, and velocity as north, east, down | position noise of 1.5 m horizontal and 3.0 m vertical, velocity noise of 0.05 m/s horizontal and 0.08 m/s vertical, one sigma |
| Optical flow | horizontal ground velocity on the body axes | m/s | every state sample, once fitted | robot origin | the robot's, `unity` | **not fitted by default**; fit it through sensor config; noise of 0.05 m/s per axis |

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
| Cart mass | 1.0 kg |
| Pole rod mass | 0.1 kg |
| Bob mass | 0.2 kg, a point mass at the tip |
| Pole length | 1.2 m, hinge to bob |
| Cart travel | 4.0 m each side of where it spawned |
| Pole angular damping | 0.01 |
| Initial pole angle | -45 degrees |
| Default frame | `unity`: x right, y up, z forward |

Everything is SI. The plant publishes in the identity `"unity"` frame, and the published
kinematics is the **cart's**:

| Channel | Meaning |
|---|---|
| `kin.lin_pos[0]` | cart position along the rail, m, in **world** coordinates |
| `kin.lin_vel[0]` | cart speed along the rail, m/s |

Note the asymmetry between the two halves of `kin`. The position is world and needs the rail
centre subtracted; the velocity does not, because a twist is a body quantity and the cart's
body is pinned to identity.

## Known quirks

> **Gotcha.** The rail centre is not the world origin and it is not on the wire. The travel
> limits are measured from wherever the scene parked the rig, not from the origin: measured
> live, one scene's cart pole sits at `x = -14.9`. A controller that regulates `lin_pos[0]`
> toward zero is ordering the cart fifteen metres to the world origin, past a dead stop it
> cannot cross, and nothing in its printout says why.

Capture the origin yourself before you try to balance. `reset()` teleports the cart to the
pose captured at its first physics step, which **is** the rail centre, so one reset, one
settle and one read of `lin_pos[0]` gives every later position term a number you measured
rather than one you assumed.

Two more:

- The simulator's own home angle is minus 45 degrees. A balance loop with no swing-up in it
  cannot catch the pole from there; stand the pole up with `initial_pole_angle_deg` first.
- Simulated offline against the linearised plant, a textbook set of gains recovers from
  about 15 degrees of perturbation and no more.

## Example

ex29 stands the pole up, finds the rail centre, and balances. The `sys_id` argument is
required:

```sh
cargo run -p vrobots-examples --bin ex29_hello_cartpole -- 4
./target/cpp-build/ex29_hello_cartpole 4
python examples/python/ex29_hello_cartpole.py 4
```

Take the id from `vrobots topic list`; ids are allocated at scene load and keep incrementing,
so no constant stays true. The example never deletes the robot, because it did not create
it, and it releases the force latch on the way out. A Ctrl-C does not, and leaves the last
force applied.

**Next:** [Half drone](05-halfdrone.md)

**See also:** [Single degree of freedom plants](../ch04-commands/04-single-dof.md), [Mass spring damper and cart pole](../ch06-services/07-msd-cartpole-config.md), [System ids, and the two kinds of robot](../ch02-concepts/03-sys-id.md)
