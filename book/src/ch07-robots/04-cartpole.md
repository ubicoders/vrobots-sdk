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
keys the sandbox scene does know, which are `multirotor`, `truck` and `msd`. The only route
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
| `pole_angular_damping` | dimensionless, 0 is frictionless | not documented |
| `max_force` | N, the clamp on `set_cartpole_force` | 20 |
| `initial_pole_angle_deg` | degrees | not documented |

The two missing defaults are not documented in the SDK; confirm against a live simulator.
<!-- VERIFY: defaults for CartPoleConfig::pole_angular_damping and initial_pole_angle_deg. -->

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

The common seven plus `srv/cartpole`. The division of labour is unusual and worth stating
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

## Frame and units

Everything is SI. The plant publishes in the identity `"unity"` frame, and the published
kinematics is the **cart's**. The pole rides the actuator channels, of which there are
exactly three:

| Channel | Meaning |
|---|---|
| `kin.lin_pos[0]` | cart position along the rail, m, in **world** coordinates |
| `kin.lin_vel[0]` | cart speed along the rail, m/s |
| `actuator.measured[0]` | the force actually applied, N, after the clamp |
| `actuator.measured[1]` | pole angle theta, **radians** |
| `actuator.measured[2]` | pole rate theta prime, rad/s |

Theta is the signed angle from world `+y` to the hinge-to-bob direction, right-handed about
`+z`, wrapped into `[-pi, pi]`. Zero is balanced upright and plus or minus pi is hanging,
both, because that is the wrap seam, so a fallen pole may print either sign. A positive
theta leans the bob toward `-x`.

Note the asymmetry between the two halves of `kin`. The position is world and needs the rail
centre subtracted; the velocity does not, because a twist is a body quantity and the cart's
body is pinned to identity.

Which sensors a cart pole carries is not documented in the SDK; confirm against a live
simulator.
<!-- VERIFY: sensor availability for the CartPole. Nothing in the repository enumerates it. -->

## Cameras

Nothing is camera-specific to this robot type. Where a robot carries the default pair,
`front_left` and `front_right` at 720p rgba8, `open_camera` attaches to either without
changing anything; this one may carry none, in which case `mount_camera` is the only way to
get pixels off it. `vrobots topic list` is the authority. See
[Cameras and images](../ch05-cameras/00-intro.md).
<!-- VERIFY: whether the scene-authored cart pole ships the two default cameras. -->

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
