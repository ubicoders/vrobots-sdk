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

A freshly created plant spawns with `k = 20` N/m, `c = 1` N·s/m and a mass of 1 kg. The
mass is set through `set_physical_params` like any other robot's; pin it yourself before
predicting anything, as ex28 does.

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

The common eight plus `srv/msd`, which carries two optional fields:

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

## Hardware

The simulator's sources hold no layout drawing for this plant, and it needs none: it is one
block on one axis, with every device at the block's origin.

### Actuators

Two entries. Neither is a device with a kind of its own, so both read `unspecified` in the
[published layout](../ch03-reading-state/10-actuator-layouts.md).

| # | Device | Kind | Driven by | `measured` | Range and rate limit |
|---|---|---|---|---|---|
| 0 | the force on the mass | unspecified | `SET_MSD` | the **total** force, `F - k*x - c*x'`, N | the commanded part is clamped to the plant's `max_force`, 100 N; applied on the next physics step |
| 1 | the mass | unspecified | nothing: it is a reading | displacement from equilibrium, m | none |

`measured[0]` is not the force you sent. It is what the spring and the damper left of it,
which is why it crosses zero at every peak.

### Sensors

The standard five are fitted, all at the block's origin. Naming a sensor the robot does not
carry in a sensor config request is skipped with a simulator log line and still
acknowledged `ok`.

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

None by default. `mount_camera` is the way to get pixels off this plant, and
`vrobots topic list` shows what is streaming. See
[Cameras and images](../ch05-cameras/00-intro.md).

### Lidars

None.

### Body and frame

| Property | Value |
|---|---|
| Mass | 1 kg |
| Spring constant `k` | 20 N/m |
| Damping `c` | 1 N·s/m |
| Force clamp | 100 N |
| Dimensions | not documented |
| Default frame | `unity`: x right, y up, z forward |

Everything is SI. The plant publishes in the identity `"unity"` frame, so `kin.lin_pos[0]`
is the position you watch in the editor and `kin.lin_vel[0]` is `x'`. The other two
components never move.

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

ex28 tunes the plant three ways. The Rust and C++ programs print the measured settling point,
period and damping ratio beside the closed-form predictions; the Python script prints samples
of the response and the predictions for you to compare:

```sh
cargo run -p vrobots-examples --bin ex28_hello_msd
./target/cpp-build/ex28_hello_msd
python examples/python/ex28_hello_msd.py
```

It takes no argument because it creates the robot it uses, and it deletes it on the way out.

**Next:** [Cart pole](04-cartpole.md)

**See also:** [Single degree of freedom plants](../ch04-commands/04-single-dof.md), [Mass spring damper and cart pole](../ch06-services/07-msd-cartpole-config.md), [Mass and inertia](../ch06-services/02-physical-params.md)
