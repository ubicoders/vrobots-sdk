# Known simulator issues

Simulator-side defects and licensing behaviour that change what the SDK can do.

Everything on this page is a property of the simulator, not of the SDK, so none of it can be
fixed by upgrading the SDK. Each entry says how to detect the symptom, because in a system
where an acknowledgement is a receipt rather than a result, most of these present as nothing
happening.

## Created multirotors do not integrate physics

**Status: open, simulator side, v3.0.0.**

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
`ex22_physical_params` and `ex27_rotor_config`, in Python, C++ and Rust alike: pass one and
the example attaches to the scene multirotor, and never deletes it, instead of creating a
robot. Without it, each example creates a multirotor and deletes it at the end, and nothing
the example commands moves that robot.

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
of the eight common services registered against it. It is a display model, not an IPC robot:
it has no `sys_id` you can attach to and no state topic.

The command id space agrees. `SET_HELI` (303) and `SET_OMROVER` (302) both exist in
`vrobots_sdk::cmd`, and neither robot type is in the simulator, so both are `absent` rather
than merely not yet acted on.

## Tier gating decides how many robots serve at once

Both behaviours were verified live on 2026-08-05.

| Plan | What serves |
|---|---|
| Pro | **all** scene robots publish and serve simultaneously |
| Community | **only** the robot selected in the simulator's SYS-ID dropdown has its ports open |

Community is the free plan, called Guest in earlier releases of this book. On it, a robot that is present in the scene and not selected in the dropdown is
indistinguishable from a robot that does not exist: no state topic, and every service GET
times out. If `vrobots topic list` shows one robot where you expect four, check the tier
before suspecting the SDK.

Skins are tier-gated separately. `srv/skin` answers `ok = false` for a tier refusal, carrying
the simulator's own reason. That surfaces as `VrError::Service` and must not be retried,
because the answer will not change.

## What the Community plan leaves off the wire

Beyond the one-robot rule, four more things are missing on the Community plan, and each
presents differently.

| Capability | On Community | How it presents |
|---|---|---|
| Robots on the wire | the focused robot only | every other robot has no state topic, no command topic and no services |
| Camera streams | one robot, its first camera only | `open_camera` on any other stream times out |
| Lidar | none | no lidar topic exists; `configure_lidars` is refused with `ok = false` and the plan's name |
| Creating a robot | refused | `connect(type, None)` fails with the simulator's reason before any id is assigned |
| Scenes | the Table Arm scene is not available | the scene card is locked |

The first camera is `front_left` on the forklift and the pallet robot, `table` on the table
arm and `left` on the turret, so stereo on the turret needs Pro. `srv/lidars` and `srv/skin`
are the two services that ever answer `ok = false`; both surface as `VrError::Service`.

A robot that loses focus mid-drive on Community keeps its last latched command, because its
ports close before a stop frame can reach it.

## Created robots appear at the scene origin

A robot created through the SDK spawns at the scene origin, unrotated, unless the connect
options carry a spawn pose. Two robots created without one overlap.

The pose is **raw Unity world coordinates**, applied exactly as sent. The frame pair on the
request is ignored, so nothing is converted: Unity's world is left-handed, in metres, with
+x right, +y up and +z forward, and the rotation is a quaternion `[x, y, z, w]`. A heading
of `yaw` radians about the up axis is `[0, sin(yaw / 2), 0, cos(yaw / 2)]`, clockwise seen
from above. The pose is also the robot's home, the one `reset()` returns it to, and it is
ignored when attaching to an existing `sys_id`.

| Language | How to pass it |
|---|---|
| C++ | `vrsdk::set_initial_pose(options, position, orientation)`, then `VirtualRobot::create(type, &options)` |
| Python | `VirtualRobot(type, initial_position=[x, y, z], initial_orientation=[x, y, z, w])` |

Wheeled robots and the turret settle under gravity. The two arms do not: an arm's base is
held exactly where it is placed, so give it a pose on the floor and clear of other bodies.
Measured live, a created forklift, pallet robot, rail arm and table arm each reported the
requested position to within 1 mm, and a created turret came to rest about 0.08 m above it.
A turret reports the identity quaternion whatever rotation it was spawned with, because its
`kin.quat` is its joint attitude and not a world attitude.

## A created table arm is not the Table Arm scene's

Two differences from the arm at `sys_id` 0 in the Table Arm scene:

- **It starts in the all-zero pose.** The sorting cell's home pose belongs to the scene, not
  to the robot type, so a created arm loads with every joint at 0 and that is the pose
  `reset()` returns it to. Read `table_arm_actuators().joints` after connecting and do not
  assume the scene's pose.
- **Create it with the name `tablearm`.** A robot created without a name is called
  `{catalog key}_{sys_id}` on the wire, `tablearm_7` for instance, and a tool that recognises a
  table arm by its wire name then miscounts its joints. In Python pass
  `robot_name="tablearm"`.

A created arm is the arm and gripper only. The board, the bins and the scoring belong to the
scene. See [Table arm](11-table-arm.md).

## No config service reads values back

Every configuration service is write-only. `srv/forklift`, `srv/palletrobot`, `srv/arm`,
`srv/turret` and `srv/lidars` reply with a receipt, exactly like the older services, and
none has a getter.

Three consequences:

- "Restore" means sending the default value back. A setting another client had changed comes
  back as the default, not as what that client set.
- An out-of-range value is healed by the simulator and still acknowledged `ok`. A value that
  is not finite is refused for that field alone while the rest of the request applies.
- The confirmation is behavioural. Read it off the actuator report, the next lidar scan, or
  `properties()`, which republishes when a setting changes.

Everything set this way survives `reset()` and nothing survives a simulator restart.

## Arm speed fallbacks change no speed

`fallback_speed_limits` on `srv/arm` is acknowledged and has no effect on either arm. A
joint runs at its own speed limit, and the fallback backs only a joint that reports none.
Every joint of the rail arm and of the table arm reports one.

The symptom is a request that succeeds and an arm that moves exactly as fast as before. The
joint speeds on the [rail arm](10-rail-arm.md) and [table arm](11-table-arm.md) pages are
therefore fixed from the SDK's side.

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

The service coverage audit closed on **2026-08-05**. Every service and every robot type in
the roster now has a Rust core implementation, a C API, C++ and Python bindings, and a
live-verified example, ex21 through ex33. The one piece of unfinished business moved to the
frozen-dynamics issue at the top of this page.

The audit's method is the reference for the next drift check between the simulator and the
SDK: probe every `vrobots/{sys_id}/z/srv/{segment}` key with a payload-less Zenoh GET, since
an acknowledgement means the service is registered and a timeout means it is not.

> **Gotcha.** That probe is safe on every key except two. On `srv/reset` a bare GET performs
> an actual reset rather than testing for a responder, and on `srv/activate` it releases a
> dormant robot's dynamics, which does nothing to a robot already running.

One related question also closed on 2026-08-05: mounting or unmounting a single camera is
confirmed live not to disturb the scene default `front_left` and `front_right` streams, so
the older warning that mounting a camera wipes the robot's defaults no longer applies.

**Next:** [Tooling and diagnostics](../ch08-tooling/00-intro.md)

**See also:** [Multirotor](01-multirotor.md), [Global Hawk](06-globalhawk.md), [Table arm](11-table-arm.md), [Lidar](../ch09-lidar/00-intro.md), [The vrobots command](../ch08-tooling/01-cli.md), [Skins](../ch06-services/08-skins.md)
