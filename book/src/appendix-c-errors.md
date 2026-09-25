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
| `InvalidArgument` | 2 | `invalid_argument` | Bad input from the caller. **Nothing was sent.** | A wrong array length, an empty camera name, a pulse width outside 1100 to 2000, a non-finite value, a rate of zero, a wildcard key where an exact one is required, a second camera at a different resolution. | Fix the call. The detail string names the field. This is the SDK refusing before the wire, so no simulator state changed. |
| `Timeout` | 3 | `timeout` | A wait ran out of time. | No first state sample, no reply to a service query, or no new snapshot before the deadline. | Depends entirely on the caller: see the note below. |
| `Decode` | 4 | `decode` | A payload arrived and did not decode as the message it should be. | Schema drift between the SDK and the simulator, or a truncated payload. | Compare `vrobots --version` against the simulator build. Decode failures on the state stream never reach you as an `Err`; they are counted in `stats()` and readable through `last_error()`, and the loop keeps running. |
| `Publish` | 5 | `publish` | Publishing to zenoh failed. | The session died between `connect` and the put. | Treat it as a lost link, not a rejected command. Nothing in the simulator validates a command by refusing the put. |
| `Service` | 6 | `service` | A service answered, and the answer was "no". | In practice only `srv/skin`, which is the one service that ever replies `ok = false`, plus `manager/z/srv/delete` on an unknown `sys_id`. | Do not retry a skin refusal: it is tier-gated, not transient. Remember that `ok = true` is a receipt, not a result, so the state stream is still the confirmation. |
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
the variant is the supported way to discover what a robot can do, and is what
`ex30_hello_halfdrone` is built on. The same code with an unexpected `sys_id` in the detail
string means something else, that no robot with that id is loaded.

## The same codes everywhere

| Surface | How the code reaches you |
|---|---|
| Rust | `VrError::code()` on the variant |
| C | the `vrsdk_err_t` return value |
| C++ | `vrsdk::Error::code()` on the thrown exception |
| Python | an exception keyed off the same code |
| CLI | exit code 1 for a command that ran and failed, 2 for arguments that did not parse, 0 for success |

The CLI's exit codes are a separate scheme from `VrError::code()` and do not correspond to
it. `run(args)` never calls `process::exit`.

**Next:** [Appendix D: Glossary](appendix-d-glossary.md)

**See also:** [When nothing happens](ch01-getting-started/08-troubleshooting.md), [Stream health](ch03-reading-state/08-health.md)
