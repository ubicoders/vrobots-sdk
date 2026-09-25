# Mass spring damper

One mass on one axis: the only plant in the simulator you can predict on paper.

## Identity

| Property | Value |
|---|---|
| `RobotType` | `Msd` |
| Catalog key | `msd` |
| Synonyms | `mass_spring_damper` |
| Creatable | yes, in the sandbox catalog |
| Scene-authored | not required; create one when you need it |
| Type-specific service | `srv/msd` |

Because it is creatable, `connect(RobotType::Msd, None)` spawns one and the reply carries a
fresh `sys_id`. Created mass spring dampers have real physics, confirmed live.

## Physical model

One mass sliding on one axis against a spring and a viscous damper:

```text
m * x'' + c * x' + k * x = F
```

The SDK owns every letter of it. `m` is `set_physical_params`, `k` and `c` are
`configure_msd`, and `F` is `set_msd_force`. That is what makes this the right first target
for control work: the closed-form answers are arithmetic, so a run that disagrees with them
is telling you something about the wire rather than about the plant.

```text
settles at   F / k              metres
period       2*pi*sqrt(m/k)     seconds
damping      c / (2*sqrt(k*m))  below 1 rings, near 1 slides home, above 1 crawls
```

A freshly created plant spawns with `k = 20` N/m and `c = 1` N·s/m. Its mass comes from
`set_physical_params` like any other robot's, and the default is not documented in the SDK;
confirm against a live simulator, or pin it yourself before predicting anything.
<!-- VERIFY: the default mass of a freshly created Msd. ex28 pins it to 1.0 kg rather than reading it. -->

## Commands accepted

| Command | Method | Units and range | Status |
|---|---|---|---|
| `SET_MSD` (305) | `set_msd_force(f64)` | N along the plant's `+x` | live |

The SDK checks the value is finite. The simulator then clamps it silently to the plant's
`max_force`, 100 N by default, so a request for 500 N is acknowledged in exactly the same
way as one for 50.

The force latches like every other command. A step force stays applied until the next
command replaces it, which is why releasing it takes an explicit `set_msd_force(0.0)`
rather than merely stopping the loop.

## Services

The common seven plus `srv/msd`, which carries two optional fields:

| Field | Type | Units | Notes |
|---|---|---|---|
| `spring_k` | `Option<f64>` | N/m | the `k` above |
| `damping_c` | `Option<f64>` | N·s/m | the `c` above |

The simulator commits a negative value as zero and acknowledges `ok`: a spring that quietly
vanished, not an error. `configure_msd` therefore refuses negatives and non-finite values
client-side, as `VrError::InvalidArgument`, before anything is published. For everything it
does send, the committed value may still differ from the asked-for one, and the state
stream is the only place that says which.

The mass belongs to `set_physical_params`, not to this service. That is the opposite of the
cart pole, where the type-specific service owns the mass.

`srv/skin` is served, as it is on every robot, but this type ships no skin catalog, so every
skin request is a no-op.

## Frame and units

Everything is SI. The plant publishes in the identity `"unity"` frame, so `kin.lin_pos[0]`
is the position you watch in the editor and `kin.lin_vel[0]` is `x'`. The other two
components never move.

The actuator block carries the plant's own arithmetic rather than a pulse width:

| Channel | Meaning |
|---|---|
| `actuator.measured[0]` | the **total** force on the mass, `F - k*x - c*x'`, in newtons |
| `actuator.measured[1]` | displacement from equilibrium, in metres |

`measured[0]` is not the force you sent. It is what the spring and the damper left of it,
which is why it crosses zero at every peak.

Which sensors a mass spring damper carries is not documented in the SDK; confirm against a
live simulator. As on every robot, naming a sensor it does not carry is skipped with a
simulator log line and still acknowledged `ok`.
<!-- VERIFY: sensor availability for the Msd. Nothing in the repository enumerates it. -->

## Cameras

Nothing is camera-specific to this robot type. Where a robot carries the default pair,
`front_left` and `front_right` at 720p rgba8, `open_camera` attaches to either without
changing anything; this one may carry none, and a robot you created yourself carries
whatever its prefab does, so `mount_camera` may be the only way to get pixels off it.
`vrobots topic list` is the authority. See
[Cameras and images](../ch05-cameras/00-intro.md).
<!-- VERIFY: whether a created Msd comes up with any default cameras mounted. -->

## Known quirks

> **Gotcha.** `configure_msd` and `set_physical_params` have no getters, so a plant you
> retuned stays retuned. `reset()` is a state reset: it returns the mass to its home
> position and zeroes velocity, and leaves `k`, `c` and the mass exactly as you last set
> them.

Two smaller ones:

- The clamp on `set_msd_force` is invisible from the client. Compare the force you sent
  with `actuator.measured[0]` at the instant `x` and `x'` are near zero if you need to know
  whether you hit it.
- A negative `k` or `c` is refused by the SDK, not by the simulator. Sending one through
  `send_cmd` or another client would leave you with a spring-free plant and an `ok`
  acknowledgement.

## Example

ex28 tunes the plant three ways and prints the measured settling point, period and damping
ratio beside the closed-form predictions:

```sh
cargo run -p vrobots-examples --bin ex28_hello_msd
./target/cpp-build/ex28_hello_msd
python examples/python/ex28_hello_msd.py
```

It takes no argument because it creates the robot it uses, and it deletes it on the way out.

**Next:** [Cart pole](04-cartpole.md)

**See also:** [Single degree of freedom plants](../ch04-commands/04-single-dof.md), [Mass spring damper and cart pole](../ch06-services/07-msd-cartpole-config.md), [Mass and inertia](../ch06-services/02-physical-params.md)
