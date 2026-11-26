# Appendix C: Error reference

Every VrError variant: what it means, what usually causes it, and what to do.

The SDK has one error type, `VrError`, and `VrResult<T>` is `Result<T, VrError>`. Each
variant carries a detail string naming the topic, service or field involved, and each has a
stable numeric code from `VrError::code()`: the code says what class of thing went wrong,
the string says which one. Those codes are part of the C and Python contract, so a code is
never reused and never renumbered, even if a variant is removed. Code `0` always means
success and is therefore never returned. `VrError::kind()` gives the short machine-friendly
name, `VrError::detail()` the string without the prefix that `Display` adds, and
`Display` prints `kind: detail`. The enum is `#[non_exhaustive]`, so match on it with a
catch-all arm.

## Variants

| Variant | Code | `kind()` | Means | Usual cause | What to do |
|---|---|---|---|---|---|
| `Session` | 1 | `session` | The zenoh session could not be opened, or has gone away. | Nothing listening at the router endpoint, a network path that closed, or the simulator's host unreachable. | Check the endpoint you passed as `router_endpoint`, then confirm the simulator is in Play mode with `vrobots topic list`. |
| `InvalidArgument` | 2 | `invalid_argument` | Bad input from the caller. **Nothing was sent.** | A wrong array length, an empty camera name, a pulse width outside 1100 to 2000, a non-finite value, a rate of zero, a wildcard key where an exact one is required, a second camera at a different resolution, a joint count that does not match the arm, a typed actuator reader called on a handle connected as another robot type. | Fix the call. The detail string names the field. This is the SDK refusing before the wire, so no simulator state changed. |
| `Timeout` | 3 | `timeout` | A wait ran out of time. | No first state sample, no reply to a service query, no new snapshot or scan before the deadline, a lidar topic that was not found, or no properties sample. | Depends entirely on the caller: see the note below. |
| `Decode` | 4 | `decode` | A payload arrived and did not decode as the message it should be. | Schema drift between the SDK and the simulator, a truncated payload, or a typed actuator reader whose robot published a different layout. | Compare `vrobots --version` against the simulator build. Decode failures on the state stream never reach you as an `Err`; they are counted in `stats()` and readable through `last_error()`, and the loop keeps running. |
| `Publish` | 5 | `publish` | Publishing to zenoh failed. | The session died between `connect` and the put. | Treat it as a lost link, not a rejected command. Nothing in the simulator validates a command by refusing the put. |
| `Service` | 6 | `service` | A service answered, and the answer was "no". | `srv/skin` for a tier refusal, `srv/lidars` on the Community plan, `manager/z/srv/create` for a type the scene does not offer or for any create on the Community plan, and `manager/z/srv/delete` on an unknown `sys_id`. | Do not retry a skin, lidar or create refusal: each is decided by the plan or the scene, not by timing. The detail string is the simulator's own reason. Remember that `ok = true` is a receipt, not a result, so the state stream is still the confirmation. |
| `NoResponder` | 7 | `no_responder` | Nobody is serving that key expression. | Asking a robot for a service its type does not serve, a wrong `sys_id`, or a robot that is not loaded. | See the note below: on a type-specific service this is an answer, not a fault. |
| `Deleted` | 8 | `deleted` | The robot was deleted by `VirtualRobot::delete`; the handle is spent. | Continuing to use a handle after deleting its robot. | Connect again. `is_deleted()` tests the local flag without touching the wire. |
| `Config` | 9 | `config` | The SDK could not be configured as asked. | A malformed router endpoint, or an unusable zenoh config. | Fix the `ConnectOptions`. The detail string names the setting; endpoints look like `tcp/192.168.1.10:7447`. |

## The two that are routinely misread

`Timeout` from `wait_new_state` and `wait_new_frame` is a **status, not a failure**. Both
calls exist to answer "has anything arrived since I last looked", and a timeout is the
honest "no": the simulator paused, the robot was deleted, or the camera stopped. `states()`
itself never fails and never blocks, and if the simulator stops it keeps returning the last
snapshot forever, so a timeout on the wait is how you detect a stall at all. Handle it as a
branch in the loop rather than propagating it with `?`. A timeout from `connect` or from a
service query is a genuine failure and reads the other way.

`NoResponder` is how a capability probe reports that a robot type does not serve a service.
A zenoh GET to a key nobody has opened is indistinguishable from a timeout, so the SDK
reports it as its own variant: `configure_drive` on anything that is not a truck answers
this, as does `configure_rotors` on anything that is not a multirotor. Asking and catching
the variant is the supported way to discover what a robot can do, and is what the Rust and
C++ versions of `ex30_hello_halfdrone` are built on. The same code with an unexpected `sys_id` in the detail
string means something else, that no robot with that id is loaded.

## Refusals from the warehouse robots, the arms and the lidars

| Call | Condition | Variant | What it tells you |
|---|---|---|---|
| `configure_lidars` | the Community plan | `Service` | the simulator's message, `lidars are not available on the <plan> plan`. Not transient; see [Lidar configuration](ch06-services/12-lidar-config.md) |
| `configure_lidars` | no entries, a bad or repeated name, an entry that sets nothing, a mount value that is not finite | `InvalidArgument` | refused before sending |
| `open_lidar_3d`, `open_lidar_2d` | the topic is not there | `Timeout` | a lidar that does not exist and a plan without lidar look the same, and the message says so; see [Opening a lidar stream](ch09-lidar/01-opening.md) |
| `open_lidar_2d` | samples arrived and none decoded as a scan | `Decode` | the topic exists, so neither the plan nor the name is the problem |
| `set_arm_joints` | a count other than 7 on a rail arm handle or 6 on a table arm handle, an empty list, a value that is not finite | `InvalidArgument` | refused before sending, because the simulator drops such a command whole and says nothing |
| `forklift_actuators`, `palletrobot_actuators`, `rail_arm_actuators`, `table_arm_actuators`, `turret_actuators` | the handle was connected as another robot type | `InvalidArgument` | the decode is by index, so another robot's report would read as wrong numbers |
| the same five readers | after `properties()` has been read, the published layout contradicts the reader's | `Decode` | the robot at that `sys_id` is not the type you connected as; see [Actuator layouts and typed readers](ch03-reading-state/10-actuator-layouts.md) |
| `connect(type, None)` | the Community plan, or a type outside the scene's catalog | `Service` | the manager's refusal, verbatim; attaching by id is unaffected |
| `properties` | no sample within the probe timeout | `Timeout` | the robot is not publishing, or the simulator is too old to publish the topic |

A timeout from `wait_new_scan` is a status in the same sense as one from `wait_new_state`:
handle it as a branch. `next_timeout` on a safety-event stream is not an error at all when
nothing arrives: it returns nothing, because silence is the normal state of that topic.

## The same codes everywhere

| Surface | How the code reaches you |
|---|---|
| Rust | `VrError::code()` on the variant |
| C | the `vrsdk_err_t` return value |
| C++ | `vrsdk::Error::code()` on the thrown exception |
| Python | an exception keyed off the same code |
| CLI | exit code 1 for a command that ran and failed, 2 for arguments that did not parse, 0 for success |

The CLI's exit codes are a separate scheme from `VrError::code()` and do not correspond to
it. The CLI never calls `process::exit`; it returns its code to whatever runs it.

**Next:** [Appendix D: Glossary](appendix-d-glossary.md)

**See also:** [When nothing happens](ch01-getting-started/08-troubleshooting.md), [Stream health](ch03-reading-state/08-health.md), [Services and configuration](ch06-services/00-intro.md)
