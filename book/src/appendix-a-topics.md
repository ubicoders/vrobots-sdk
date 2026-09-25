# Appendix A: Topic reference

Every topic pattern in one table: key, transport, direction, payload and rate.

Every wire name in the system has the shape `vrobots/<sys_id>/<transport>/<subject...>`,
where the segment after the id names the transport, `z` for zenoh and `i` for iceoryx2. The
words `manager` and `scene` sit where a `sys_id` would, so a scope key can never collide
with a robot key. Both ends of a wire must match byte for byte and a mismatch is silent: a
subscriber on a slightly wrong key never fires, and a GET to an unopened key returns
[`NoResponder`](appendix-c-errors.md). `vrobots topic list` prints what is really there,
tagged `[z]` or `[i]`.

## Every topic

| Key pattern | Wire | Direction | Contents | Rate |
|---|---|---|---|---|
| `vrobots/{sys_id}/z/state` | `[z]` | the simulator publishes | `swarmbotix.states.State`, the full snapshot: kinematics, wrench, actuator, sensors, environment, estimate | 25 Hz |
| `vrobots/{sys_id}/z/cmd` | `[z]` | the simulator subscribes | `swarmbotix.commands.Command`, one command id plus its `CmdArgs`. Many-to-many, so clients can read it too | set by the sender; the in-game IMU panel publishes `SET_ANGVEL` at 50 Hz |
| `vrobots/{sys_id}/z/frames` | `[z]` | the simulator publishes | `swarmbotix.coordinates.CoordFrameDef`, the resulting frame definitions, one message per distinct frame the robot and its devices use. Distinct from the `srv/frames` service, which sets them. Read with `frame_def` | on change, plus a slow keepalive |
| `vrobots/{sys_id}/z/estimate` | `[z]` | the simulator subscribes | `swarmbotix.states.EstimateState`, an attitude estimate you publish with `publish_estimate` or `publish_estimate_euler`. Read by the fixed wing under `FW_EST_OBSERVER`; nothing else consumes it, and the simulator never echoes it back into `State.estimate` | set by the sender; aged from arrival and stale after 0.5 s, so publish at 20 Hz or better |
| `vrobots/{sys_id}/z/srv/{segment}` | `[z]` | request and response | per-robot services; see the segment tables below | on demand |
| `vrobots/{sys_id}/i/cam/{name}/{res}_{fmt}` | `[i]` | the simulator publishes | raw camera frames, shared memory, same host only | not documented; measure with `vrobots topic hz` |
| `vrobots/manager/z/srv/create` | `[z]` | request and response | `SrvVRobotCreate` to `SrvVRobotCreated`; the reply carries the assigned `sys_id`. The one non-idempotent service in the system | on demand |
| `vrobots/manager/z/srv/delete` | `[z]` | request and response | `SrvVRobotDelete` to `SrvAck`. Answers `ok = false` for an unknown `sys_id` | on demand |
| `vrobots/scene/z/srv/frame` | `[z]` | request and response | payload-less GET to `SrvAck` whose `message` is the scene's active coordinate frame id. Scene scope: one answer however many robots are loaded | on demand |

Two wildcards the SDK subscribes with rather than publishes on: `vrobots/**`, which backs
topic discovery, and `vrobots/*/z/state`, which is the whole-swarm state subscribe.

## Common service segments

Every live robot serves all seven of these on `vrobots/{sys_id}/z/srv/{segment}`.

| Segment | Request to reply | Purpose | SDK entry point |
|---|---|---|---|
| `activate` | payload-less GET to `SrvAck` | releases a dormant robot's dynamics hold; an already-active robot acks and no-ops | `activate`, and `connect` when `activate_after_create` is set |
| `reset` | `SrvReset`, or a payload-less GET, to `SrvAck` | teleports to the first-physics-step pose, zeroes velocity, rests the actuators, re-latches the initial command | `reset` |
| `params` | `SrvVRobotPhysicalProperty` to `SrvAck` | mass and principal moments of inertia | `set_physical_params` |
| `skin` | `SrvVRobotSkin` to `SrvAck` | appearance; the only service that ever answers `ok = false` | `set_skin` |
| `sensors` | `SrvSensorConfig` to `SrvAck` | noise models and GPS quality, gated block by block | `configure_sensors` |
| `frames` | `SrvVRobotFrame` to `SrvAck` | which coordinate frame the robot and each device report in | `set_frames` |
| `cameras` | `SrvCameraConfig` to `SrvAck` | upsert, remove or replace the camera list | `mount_camera`, `mount_camera_with`, `unmount_camera` |

> **Gotcha.** `reset` and `activate` are the two keys where a payload-less GET performs the
> action instead of probing for a receipt. The simulator enqueues the empty request because
> the in-game Reset button cannot attach a payload. Probing `srv/reset` for reachability
> resets the robot; probing `srv/activate` releases a dormant robot's hold, which is
> idempotent and so harmless on one already running.

## Type-specific service segments

| Segment | Request to reply | Robot | SDK entry point |
|---|---|---|---|
| `drive` | `SrvUgvDriveConfig` to `SrvAck` | Truck | `configure_drive` |
| `rotors` | `SrvMultirotorRotorConfig` to `SrvAck` | Multirotor | `configure_rotors` |
| `msd` | `SrvMsdConfig` to `SrvAck` | Msd | `configure_msd` |
| `cartpole` | `SrvCartPoleConfig` to `SrvAck` | CartPole | `configure_cartpole` |

HalfDrone and GlobalHawk add nothing to the common seven. Querying a segment the robot's
type does not serve is not an error on the wire, only a GET nobody answers, which the SDK
reports as `NoResponder` and which is exactly how a capability probe works.

## One key, decomposed

`vrobots/1/i/cam/front_left/720p_rgba8`

| Segment | Value | Meaning |
|---|---|---|
| 1 | `vrobots` | the root every key in the system shares |
| 2 | `1` | the `sys_id`; `manager` or `scene` here means a scope rather than a robot |
| 3 | `i` | iceoryx2, so shared memory and same host only; `z` would be zenoh |
| 4 | `cam` | the subject |
| 5 | `front_left` | the camera name, from `CameraSpec::name` |
| 6 | `720p_rgba8` | `{res}_{fmt}`, from `CameraSpec::stream_segment()`: 1280 by 720, four bytes per pixel |

The format segment is part of the key, so changing a camera's resolution or pixel format
renames its stream. A reader opened on the old name goes quiet rather than failing.
`CameraStream::service_name()` returns the key that `vrobots topic list` prints, character
for character, which is the fastest way to check the two agree.

**Next:** [Appendix B: Command reference](appendix-b-commands.md)

**See also:** [The topic namespace](ch02-concepts/02-topics.md), [Discovery from code](ch08-tooling/02-discovery-from-code.md)
