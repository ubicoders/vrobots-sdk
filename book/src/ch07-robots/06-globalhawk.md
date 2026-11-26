# Global Hawk

A fixed wing aircraft with six control panels, an onboard rate loop, and thrust in newtons.

## Identity

| Property | Value |
|---|---|
| `RobotType` | `GlobalHawk` |
| Catalog key | `globalhawk` |
| Synonyms | `global_hawk`, `rq4b` |
| Creatable | **no**, scene-authored only |
| Scene | **IMU scene**, not the sandbox |
| Type-specific service | none |

Attach with `connect(RobotType::GlobalHawk, Some(sys_id))`. The id comes from
`vrobots topic list`. This is the only robot in the roster that lives outside the sandbox
scene, so a sandbox session will not show one at all.

## Physical model

An RQ-4B airframe with six aerodynamic panels and one engine. It is the first robot in this
book that flies itself: onboard rate PIDs and an airspeed hold close the loop against
whatever `SET_ANGVEL` setpoint arrives, and left alone it cruises at **72.8 m/s** and needs
nothing from you.

| Quantity | Value |
|---|---|
| Panels | 6 |
| Surface deflection limit | 20 degrees, default |
| Maximum thrust | 20 kN |
| Trim cruise | 72.8 m/s under the onboard loop |
| Thrust at trim | about 3.8 kN |

Two control modes, selected by `SET_FW_CTRL_MODE`:

| Constant | Value | Behaviour |
|---|---|---|
| `FW_ONBOARD_RATE` | 0 | default; the simulator flies via a rate loop tracking `SET_ANGVEL` |
| `FW_DIRECT_SURFACE` | 1 | the rate loop is bypassed and the six panels take your radians verbatim |

The aircraft weighs 1100 kg by default. Mass and inertia are settable through
`set_physical_params` like any other robot's, and the full list is under
[Hardware](#hardware).

## Commands accepted

| Command | Method | Units and range | Status |
|---|---|---|---|
| `SET_ANGVEL` (51) | `send_cmd(cmd::SET_ANGVEL, ...)` with a `vec3` | rad/s, body rates | live, tracked by the onboard loop |
| `SET_FW_SURFACES` (307) | `set_fw_surfaces(&[f64])` | radians, one per panel | live in `FW_DIRECT_SURFACE` |
| `SET_FW_THRUST` (308) | `set_fw_thrust(f64)` | N, clamped to `[0, 20000]` | live |
| `SET_FW_THRUST_BIAS` (309) | `set_fw_thrust_bias(f64)` | N, signed trim | live in `FW_ONBOARD_RATE` only |
| `SET_FW_CTRL_MODE` (310) | `set_fw_ctrl_mode(i32)` | 0 or 1 | live |
| `SET_FW_EST_SOURCE` (311) | `set_fw_est_source(i32)` | 0 or 1 | live in `FW_ONBOARD_RATE` only |

`SET_ANGVEL` (51) has the typed wrapper `set_angvel(vec3)`; the generic `send_cmd` path
spells the same bytes, and ex33 keeps using it as the escape-hatch demonstration. It
carries a `vec3`, which means it is re-expressed from your header frame into the robot's
as an axial vector; connect with `coord_frame_id` set to `"frd"` and `[0, 0, r]` means
what it looks like.

There is no mixer in `FW_DIRECT_SURFACE`. Each entry drives its own panel, in this order,
and the length must equal the panel count exactly: a wrong-length array makes the simulator
drop the whole command rather than apply it partially.

| Index | Panel | What the onboard mixer does with it |
|---|---|---|
| 0 | left outboard flap | aileron, gain +1 |
| 1 | right outboard flap | aileron, gain -1 |
| 2 | left inner flap | nothing, gain 0 on all three channels |
| 3 | right inner flap | nothing |
| 4 | rear left ruddervator | elevator -1, rudder +1 |
| 5 | rear right ruddervator | elevator -1, rudder -1 |

Indices 2 and 3 are the proof that the per-panel path is real: the simulator's own mixer has
zero gain there and can never move them, so an inner flap that follows your command came
through `SET_FW_SURFACES`.

`set_fw_thrust` in `FW_ONBOARD_RATE` pins the engine off airspeed hold and clears any thrust
bias, last writer wins; send `set_fw_thrust_bias(0.0)` to release it back to airspeed hold.
In `FW_DIRECT_SURFACE` the bias is ignored entirely.

## Services

The common eight and nothing else. The Global Hawk adds no type-specific service: its
control surface is entirely command-level, so a `configure_*` call meant for another type
answers `VrError::NoResponder`.

`reset()` reverts **both** the control mode, to `FW_ONBOARD_RATE`, and the estimate source,
to `FW_EST_TRUTH`. That is deliberate: keeping direct control with the surface latches
zeroed would relaunch the aircraft unflyable. A direct-surface client must re-assert both
after every reset. A reset also clears the `SET_ANGVEL` setpoint, unlike every other latch
on this aircraft.

This type ships no skin catalog, so every skin request is a no-op.

## Hardware

The simulator's sources hold no layout drawing for the Global Hawk. Every sensor sits at the
robot origin, and the six panels are listed in actuator order below.

### Actuators

The report has **panels plus one** entries, which is the fact to carry away from the page:

```text
measured[0..=5]  per-panel deflection, RADIANS
measured[6]      the engine, NEWTONS, not normalised and not a pulse width
```

That echo is the only receipt for anything on this aircraft, including `set_fw_thrust`.

| # | Device | Kind | Driven by | `measured` | Range and rate limit |
|---|---|---|---|---|---|
| 0 | left outboard flap, 6.0 m² | surface | the onboard mixer (aileron), or entry 0 of `SET_FW_SURFACES` | deflection, rad | ±20 degrees by default |
| 1 | right outboard flap, 6.0 m² | surface | the onboard mixer (aileron), or entry 1 | deflection, rad | ±20 degrees |
| 2 | left inner flap, 3.8 m² | surface | entry 2 of `SET_FW_SURFACES` only | deflection, rad | ±20 degrees |
| 3 | right inner flap, 3.8 m² | surface | entry 3 only | deflection, rad | ±20 degrees |
| 4 | rear left ruddervator, 2.0 m² | surface | the onboard mixer (elevator and rudder), or entry 4 | deflection, rad | ±20 degrees |
| 5 | rear right ruddervator, 2.0 m² | surface | the onboard mixer (elevator and rudder), or entry 5 | deflection, rad | ±20 degrees |
| 6 | engine | unspecified | the airspeed hold, `SET_FW_THRUST` or `SET_FW_THRUST_BIAS` | thrust, N | 0 to 20 000 N |

<!-- VERIFY: whether a surface or the engine has a rate limit. None is documented in the sources read for this page. -->

### Sensors

The standard five are fitted, all at the robot origin. The aircraft flies at about 6 km,
so the barometer reads the pressure of that altitude, not of sea level.

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
| Mass | 1100 kg |
| Moments of inertia in `frd` | roll 8500, pitch 5000, yaw 12 000 kg·m² |
| Centre of mass in `frd` | (-0.5, 0, -1.4) m from the robot origin |
| Maximum thrust | 20 kN |
| Trim cruise | 72.8 m/s at about 6 km, air density 0.660 kg/m³ |
| Wing span and length | not documented |
| Default frame | `frd`: x forward, y right, z down |

<!-- VERIFY: Global Hawk mass, inertia and centre of mass are the shipped model's values, not measured live. -->

Everything is SI: radians for surfaces, newtons for thrust, rad/s for body rates. Pose is
world frame and twist is body frame, as on every robot.

The aircraft publishes `"frd"`, with `axis_convention` reading `Axes::FRD`, verified live on
2026-08-09 against simulator v3.0.0. The setpoint on `z/cmd` arrives in the sender's frame,
unconverted, while the in-game IMU panel stamps the aircraft's own `frd`, so it reads
`[p, q, r]` in rad/s and is directly subtractable from `kin.ang_vel`. No per-robot-type
native `coord_frame_id` is defined in the SDK source, so the tag on the snapshot is the
authority: in code, read `State::coord_frame_id`.

## Known quirks

> **Gotcha.** Bumpless is not zeroed. Entering `FW_DIRECT_SURFACE` seeds the surface and
> thrust latches from what the plant is doing now, including whatever thrust the airspeed
> hold happened to be holding, so nothing jolts. A client that wants a particular thrust
> must therefore send `set_fw_thrust` after **every** mode entry and after every reset. Skip
> it and the engine keeps the autopilot's value, usually about 3.8 kN at trim.

Three more:

- Everything latches and there is no watchdog. Stop sending and the aircraft keeps flying
  your last deflections forever, exactly like a dead PWM client on a multirotor.
- An estimate older than 0.5 s is treated as stale, and the onboard loop silently falls back
  to truth until a fresh one arrives. The SDK does not publish `z/estimate` yet, which
  bounds what `FW_EST_OBSERVER` can be used for today.
- Version skew is documented rather than fixed. An old SDK against a new simulator is
  unaffected, since the aircraft still defaults to onboard rate. A new SDK against an old
  simulator sees unknown command ids ignored silently, so `set_fw_surfaces` appears to
  succeed while nothing moves. Detect it from the actuator echo length: an old simulator
  publishes six entries rather than seven.

## Example

ex31 takes the surfaces off the autopilot and demonstrates the latch, the reset and the
bumpless entry. ex32 closes an external rate loop around the operator's stick, and ex33
switches the estimate source. All three require a `sys_id` argument:

```sh
cargo run -p vrobots-examples --bin ex31_globalhawk_direct -- 15
./target/cpp-build/ex31_globalhawk_direct 15
python examples/python/ex31_globalhawk_direct.py 15
cargo run -p vrobots-examples --bin ex32_fw_rate_controller -- 15
./target/cpp-build/ex32_fw_rate_controller 15
python examples/python/ex32_fw_rate_controller.py 15
cargo run -p vrobots-examples --bin ex33_fw_est_source -- 15
./target/cpp-build/ex33_fw_est_source 15
python examples/python/ex33_fw_est_source.py 15
```

Take the id from `vrobots topic list`, against the IMU scene. None of them deletes the
aircraft, and each hands it back to its autopilot on the way out; a Ctrl-C does not, and
leaves it flying the last deflections.

**Next:** [Forklift](08-forklift.md)

**See also:** [Fixed wing control](../ch04-commands/05-fixed-wing.md), [Reading someone else's commands](../ch04-commands/08-reading-commands.md), [The generic command](../ch04-commands/06-generic-cmd.md)
