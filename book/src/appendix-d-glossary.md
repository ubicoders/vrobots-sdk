# Appendix D: Glossary

One line each for the terms this book uses precisely.

The book uses these words in one sense and one sense only. Where a term looks like ordinary
English but is not, the third column names the page that pins it down. Terms are listed
alphabetically, ignoring backticks and capitalisation.

| Term | Meaning | Explained in |
|---|---|---|
| ack | A service reply. It says the request arrived and was packed, not that the change took effect; only `srv/skin` and `srv/lidars` ever answer `ok = false`. | [Services and configuration](ch06-services/00-intro.md) |
| actuator echo | The `actuator` block in the state stream: the command the robot is holding, as `pwm` and `normalized`, beside what the devices did, as `measured`. The only proof a command landed. | [Actuators](ch03-reading-state/05-actuator.md) |
| actuator kind | What one entry of the actuator arrays is, and so the unit of `measured` at that index: rotor, servo, wheel, surface, linear, vacuum, laser, gripper, contact or hold. Published per index on `z/properties`. | [Actuator layouts and typed readers](ch03-reading-state/10-actuator-layouts.md) |
| attach | Connecting to a robot that already exists by passing `Some(sys_id)`. Never touches `srv/create` and works for any robot the scene contains. | [System ids, and the two kinds of robot](ch02-concepts/03-sys-id.md) |
| axis convention | The `Axes` tag beside every vector: `UNSPECIFIED` 0, `UNITY` 1, `FRD` 2, `CV` 3. A tag, not the authority; `coord_frame_id` is. | [Frames, axes and units](ch02-concepts/07-frames-and-units.md) |
| azimuth | A lidar beam's angle about the up axis: 0 is forward, growing counter-clockwise seen from above. Column `c` is at `azimuth_start + c * azimuth_step`. | [Inside a scan](ch09-lidar/02-scan-layout.md) |
| capability probe | Asking a robot for a type-specific service and reading `NoResponder` as "this type does not serve it". | [Appendix C: Error reference](appendix-c-errors.md) |
| catalog key | The wire name of a robot type, such as `"multirotor"`. The catalog belongs to the scene, not the SDK, and only `catalog_key()` is ever put on the wire. | [Robot lifecycle](ch06-services/01-lifecycle.md) |
| column | One azimuth step of a lidar scan. A scan is `rings` rows of `columns` ranges, and beam `ring * columns + column` is one entry of `ranges`. | [Inside a scan](ch09-lidar/02-scan-layout.md) |
| Community plan | The simulator's free plan. Only the robot selected in the SYS-ID dropdown is on the wire, there is no lidar, and creating a robot is refused. | [Known simulator issues](ch07-robots/07-known-issues.md) |
| `coord_frame_id` | The authoritative string naming the frame a vector is expressed in. The only way to name a frame registered at runtime. | [Coordinate frames](ch06-services/04-frames-config.md) |
| created robot | A robot the SDK spawned through `manager/z/srv/create` by calling `connect` with `None`. The reply carries its fresh id. | [System ids, and the two kinds of robot](ch02-concepts/03-sys-id.md) |
| `elapsed` | Seconds since this robot's first state sample, monotonic, on one epoch shared by all of its streams including cameras. For printing and plotting. | [Timestamps and sequence numbers](ch03-reading-state/06-timestamps.md) |
| elevation | A lidar ring's angle above the horizontal. Ring 0 is the lowest, and ring `r` is at `elevation_start + r * elevation_step`. | [Inside a scan](ch09-lidar/02-scan-layout.md) |
| FlatBuffers | The wire format for every message on both transports. Decoding verifies the buffer before reading any field. | [Two transports, one simulator](ch02-concepts/01-transports.md) |
| FLU | The frame id `"flu"`: +x forward, +y left, +z up, right-handed. Every lidar's default frame, whatever frame its robot reports in, and a built-in with no `Axes` constant. | [Frames, axes and units](ch02-concepts/07-frames-and-units.md) |
| header frame | The frame **you** stamp on what you send, set by `ConnectOptions::coord_frame_id`, default `"unity"`. Distinct from the robot's own frame. | [Frames, axes and units](ch02-concepts/07-frames-and-units.md) |
| iceoryx2 | The shared-memory transport that carries camera frames and 3-D lidar scans. Same host only, and it has a registry, so an entry can exist with no live publisher. | [Two transports, one simulator](ch02-concepts/01-transports.md) |
| latch | What every command does: the last one received stays in effect until the next arrives. There is no watchdog and no expiry. | [Commands latch](ch04-commands/01-latching.md) |
| no return | A lidar beam that hit nothing within range. It reads `+infinity` in `ranges`, never 0, and `NaN` in the points built from it. | [Inside a scan](ch09-lidar/02-scan-layout.md) |
| observed versus registered | Why a topic appears in discovery. Zenoh entries are observed, meaning they published during your window and the counters are real. iceoryx2 entries are registered, and `live` tells a streaming one from a dead leftover. | [Discovery from code](ch08-tooling/02-discovery-from-code.md) |
| phase 0 | The point in the robot's next physics step at which a service change is applied, which is why an ack precedes the effect. | [Services and configuration](ch06-services/00-intro.md) |
| physics step | One simulator integration tick. Commands are drained from the queue at its start; service changes land in its phase 0. | [Five rules that explain everything](ch02-concepts/06-five-rules.md) |
| pinned id | A `sys_id` a scene fixes for one of its robots, the same on every load: the Warehouse forklift 0, pallet robot 1 and rail arm 2, and the table arm 0 in its scene. Ids allocated afterwards start above the highest pin. | [System ids, and the two kinds of robot](ch02-concepts/03-sys-id.md) |
| points frame | The frame a scan's computed points are expressed in, named by `points_frame_id`: `"flu"` until you override the device `lidar/<name>`. An override re-labels points and never changes the ranges. | [Points and frames](ch09-lidar/03-points-and-frames.md) |
| Pro plan | The simulator's plan above Community: every scene robot serves at once, lidar topics exist, and robots can be created at run time. | [Known simulator issues](ch07-robots/07-known-issues.md) |
| ring | One elevation row of a lidar scan. A 3-D unit has 16 and a 2-D unit has 1. | [Inside a scan](ch09-lidar/02-scan-layout.md) |
| safety event | One record of one incident between a robot and a person, another robot or an object: a near miss, a contact or a strike. Published once per incident on `z/event/safety`, queued by the SDK and handed out once each. | [Safety events](ch10-events/00-safety-events.md) |
| scan | One revolution of a lidar: `ranges` in beam order, the beam layout and mount that produced them, and a `seq` and `t_ns` of its own. | [Inside a scan](ch09-lidar/02-scan-layout.md) |
| scene-authored robot | A robot the scene placed, rather than one the SDK created. Attach to it by id; `cartpole`, `halfdrone` and `globalhawk` exist only this way. | [System ids, and the two kinds of robot](ch02-concepts/03-sys-id.md) |
| `seq` | A per-topic sequence number. A jump of more than one means dropped samples, counted in `stats()` as `seq_gaps` and `missed_samples`. | [Timestamps and sequence numbers](ch03-reading-state/06-timestamps.md) |
| setpoint | A command read back off the `z/cmd` bus as a `Setpoint`, with its `vec3` payload unconverted and in the sender's frame. | [Reading someone else's commands](ch04-commands/08-reading-commands.md) |
| snapshot | The `State` value `states()` returns: the latest complete sample, never torn, never blocking, never failing. | [Reading state](ch03-reading-state/00-intro.md) |
| spawn pose | Where a created robot appears, and the home a reset returns it to. Raw Unity world coordinates (+x right, +y up, +z forward) with the rotation as `[x, y, z, w]`; no frame conversion applies. | [Robot lifecycle](ch06-services/01-lifecycle.md) |
| `src_id` | Who published a message. Yours defaults to 122, must be non-zero, and 0 is reserved for the simulator; the in-game IMU panel publishes as 108. | [What connect actually does](ch02-concepts/05-connect.md) |
| `sys_id` | Which robot a message belongs to, and the segment after `vrobots/` in every per-robot key. Allocated at scene load and incrementing across loads. | [System ids, and the two kinds of robot](ch02-concepts/03-sys-id.md) |
| `t_ns` | Capture time in nanoseconds since the unix epoch, signed. The one clock shared by state snapshots and camera frames, so the two are directly subtractable. | [Timestamps and sequence numbers](ch03-reading-state/06-timestamps.md) |
| truth / measured / believed | The three views the schema keeps apart: `kin`, `wrench` and `env` are simulator-exact, `sensors` are noisy and robot-observable, `estimate` is the robot's own filter belief. | [Truth, measured and believed](ch03-reading-state/01-truth-measured-believed.md) |
| typed reader | A method such as `forklift_actuators()` that decodes the latest actuator report of one robot type into named fields. | [Actuator layouts and typed readers](ch03-reading-state/10-actuator-layouts.md) |
| upsert | The verb `mount_camera` uses: add the named camera or reconfigure it, leaving every other camera on the robot alone. | [Mount, open and unmount](ch05-cameras/01-mount-open-unmount.md) |
| zenoh | The pub/sub and query transport carrying states, commands, services, properties, safety events and 2-D lidar scans. Works across a network, and has no registry, so a topic appears in discovery only if it published during your window. | [Two transports, one simulator](ch02-concepts/01-transports.md) |

**Next:** [Introduction](introduction.md)

**See also:** [Appendix A: Topic reference](appendix-a-topics.md), [Five rules that explain everything](ch02-concepts/06-five-rules.md)
