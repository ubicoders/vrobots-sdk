# Known simulator issues

Simulator-side defects and licensing behaviour that change what the SDK can do.

Everything on this page is a property of the simulator, not of the SDK, so none of it can be
fixed by upgrading the crate. Each entry says how to detect the symptom, because in a system
where an acknowledgement is a receipt rather than a result, most of these present as nothing
happening.

## Created multirotors do not integrate physics

**Status: open, simulator side, v3.0.0.** Tracked in
`issues/created-multirotor-frozen-dynamics.md`.

A multirotor created through `vrobots/manager/z/srv/create` comes up publishing and serving
normally, and its actuator echo is live, but its rigidbody never integrates. The command
path, the actuator model and the state publisher all run; only physics integration is dead.

| Stimulus | Scene multirotor | Created multirotor |
|---|---|---|
| `SET_MR_PWM` 1800 µs on four rotors, 3 s | climbs 19.7 m | zero motion |
| gravity alone | falls, then rests | hangs at spawn altitude forever |
| `set_body_force`, 100 N up, 3 s | | zero motion |
| `configure_rotors` then 1800 µs | | zero motion |
| `set_physical_params` mass 1 kg, then 1800 µs | | zero motion |
| `srv/reset` then 1800 µs | | teleports home, then freezes again |
| actuator echo and rotor speed model | live | live |

The fault is type-specific. Created trucks show real measured turn radii and created mass
spring dampers show real oscillation physics, so only the multirotor spawn path is affected,
and scene-authored multirotors are fine.

**The workaround shipped in the examples** is an optional `sys_id` argument on `ex21_reset`,
`ex22_physical_params` and `ex27_rotor_config`: pass one and the example attaches to the
scene multirotor instead of creating a robot.

```sh
cargo run -p vrobots-examples --bin ex22_physical_params -- 1
./target/cpp-build/ex22_physical_params 1
python examples/python/ex22_physical_params.py 1
```

> **Gotcha.** Attaching has a price. Anything ex22 or ex27 configures stays configured on
> that shared robot until the scene reloads, because `reset()` is a state reset and neither
> `srv/params` nor `srv/rotors` has a getter to restore the old value from. A later program
> attaching to the same `sys_id` inherits whatever the last one left.

## HeliModel exists and serves nothing

The simulator contains a `HeliModel`, and a Zenoh GET probe of every `srv/*` key finds none
of the seven common services registered against it. It is a display model, not an IPC robot:
it has no `sys_id` you can attach to and no state topic.

The command id space agrees. `SET_HELI` (303) and `SET_OMROVER` (302) both exist in
`vrobots_sdk::cmd`, and neither robot type is in the simulator, so both are `absent` rather
than merely not yet acted on.

## Tier gating decides how many robots serve at once

Both behaviours were verified live on 2026-08-05.

| Tier | What serves |
|---|---|
| Pro | **all** scene robots publish and serve simultaneously |
| Guest | **only** the robot selected in the simulator's SYS-ID dropdown has its ports open |

Under Guest, a robot that is present in the scene and not selected in the dropdown is
indistinguishable from a robot that does not exist: no state topic, and every service GET
times out. If `vrobots topic list` shows one robot where you expect four, check the tier
before suspecting the SDK.

Skins are tier-gated separately. `srv/skin` is the only service in the whole API that ever
answers `ok = false`, and it does so for a tier refusal, carrying the simulator's own reason.
That surfaces as `VrError::Service` and must not be retried, because the answer will not
change.

## Global Hawk version skew

The fixed wing command ids were added to both sides at once, so an SDK and a simulator can
disagree about them. The skew is documented rather than prevented.

| Combination | Behaviour |
|---|---|
| Old SDK, new simulator | unaffected; the aircraft still defaults to `FW_ONBOARD_RATE` and tracks `SET_ANGVEL` |
| New SDK, old simulator | the old robot ignores unknown command ids silently, so `set_fw_surfaces` appears to succeed and nothing moves |

The second case is detectable, and the check costs one state read. A simulator too old for
this work publishes an actuator echo with **six** entries; a current one publishes **seven**,
because index 6 is the engine in newtons.

> **Note.** Ignoring an unknown command id is correct behaviour, not a bug: the id space is
> shared across every robot type and no robot implements all of it. That is also why a
> command sent to the wrong robot type is silently dropped rather than refused.

## Service coverage is closed

`issues/service-coverage.md` was closed on **2026-08-05**. Every service and every robot type
in the roster now has a Rust core implementation, a C API, C++ and Python bindings, and a
live-verified example, ex21 through ex33. The one piece of unfinished business moved to the
frozen-dynamics issue at the top of this page.

The issue is kept rather than deleted because its method is the reference for the next drift
check between the simulator and the SDK: probe every `vrobots/{sys_id}/z/srv/{segment}` key
with a payload-less Zenoh GET, since an acknowledgement means the service is registered and a
timeout means it is not.

> **Gotcha.** That probe is safe on every key except `srv/reset`, where a bare GET performs
> an actual reset rather than testing for a responder.

One related issue also closed on 2026-08-05: `issues/srv-cameras-add-remove.md`. Mounting or
unmounting a single camera is confirmed live not to disturb the scene default `front_left`
and `front_right` streams, so the older warning that mounting a camera wipes the robot's
defaults no longer applies.

**Next:** [Tooling and diagnostics](../ch08-tooling/00-intro.md)

**See also:** [Multirotor](01-multirotor.md), [Global Hawk](06-globalhawk.md), [The vrobots command](../ch08-tooling/01-cli.md), [Skins](../ch06-services/08-skins.md)
