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

Mass, inertia and wing geometry are not documented in the SDK; confirm against a live
simulator. Mass and inertia are settable through `set_physical_params` like any other
robot's.
<!-- VERIFY: default mass, moments of inertia and wing geometry for the Global Hawk. -->

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

The common seven and nothing else. The Global Hawk adds no type-specific service: its
control surface is entirely command-level, so a `configure_*` call meant for another type
answers `VrError::NoResponder`.

`reset()` reverts **both** the control mode, to `FW_ONBOARD_RATE`, and the estimate source,
to `FW_EST_TRUTH`. That is deliberate: keeping direct control with the surface latches
zeroed would relaunch the aircraft unflyable. A direct-surface client must re-assert both
after every reset. A reset also clears the `SET_ANGVEL` setpoint, unlike every other latch
on this aircraft.

This type ships no skin catalog, so every skin request is a no-op.

## Frame and units

Everything is SI: radians for surfaces, newtons for thrust, rad/s for body rates. Pose is
world frame and twist is body frame, as on every robot.

The actuator echo is shaped differently from every other robot's, and this is the fact to
carry away from the page. `actuator.measured` has **panels plus one** entries:

```text
measured[0..=5]  per-panel deflection, RADIANS
measured[6]      the engine, NEWTONS, not normalised and not a pulse width
```

That echo is the only receipt for anything on this aircraft, including `set_fw_thrust`.

The aircraft publishes `"frd"`, with `axis_convention` reading `Axes::FRD`, verified live on
2026-08-09 against simulator v3.0.0. The setpoint on `z/cmd` arrives in the sender's frame,
unconverted, while the in-game IMU panel stamps the aircraft's own `frd`, so it reads
`[p, q, r]` in rad/s and is directly subtractable from `kin.ang_vel`. No per-robot-type
native `coord_frame_id` is defined in the SDK source, so the tag on the snapshot is the
authority: in code, read `State::coord_frame_id`.

Which sensors it carries is not documented in the SDK; confirm against a live simulator.
<!-- VERIFY: sensor availability for the GlobalHawk. Nothing in the repository enumerates it. -->

## Cameras

Nothing is camera-specific to this robot type. Where a robot carries the default pair,
`front_left` and `front_right` at 720p rgba8, `open_camera` attaches to either without
changing anything; the IMU scene's own defaults are not documented in the SDK and may be
none, in which case `mount_camera` is the only way to get pixels off it. `vrobots topic
list` is the authority. See [Cameras and images](../ch05-cameras/00-intro.md).
<!-- VERIFY: which cameras, if any, the IMU scene's Global Hawk ships by default. -->

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

**Next:** [Known simulator issues](07-known-issues.md)

**See also:** [Fixed wing control](../ch04-commands/05-fixed-wing.md), [Reading someone else's commands](../ch04-commands/08-reading-commands.md), [The generic command](../ch04-commands/06-generic-cmd.md)
