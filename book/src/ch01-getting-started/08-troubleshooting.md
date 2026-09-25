# When nothing happens

A symptom-to-cause table for the failures that stop people on their first afternoon.

Almost every one of these is a correct behaviour presenting as a broken one. The SDK
reports what it can, but a great deal of the system communicates by silence: a command
that landed on the wrong robot, a topic name with a typo, and a simulator that is not
playing all look the same from inside your loop.

Two commands answer most of it before you read any further:

```sh
cargo run -p vrobots-sdk --bin vrobots -- topic list
cargo run -p vrobots-sdk --bin vrobots -- --version
```

## Symptoms and causes

| Symptom | Likely cause | What to do | Where it is explained |
|---|---|---|---|
| `topic list` shows `(no topics)` | The simulator is not in Play mode | Press Play; loading the project is not enough | [First contact](02-first-contact.md) |
| | The simulator is on another machine | Pass `--router tcp/<host>:7447` | [Two transports, one simulator](../ch02-concepts/01-transports.md) |
| | zenoh discovery had too short a window | Retry with `-t 5` | [The vrobots command](../ch08-tooling/01-cli.md) |
| | A `-k` filter excluded everything | Drop the flag | [The vrobots command](../ch08-tooling/01-cli.md) |
| `connect` hangs, then times out | The `sys_id` does not exist in this scene | Read the ids out of `topic list`; ids are reallocated on every scene load | [System ids](../ch02-concepts/03-sys-id.md) |
| | `connect(type, None)` was refused by the scene's catalog | Only `multirotor`, `truck` and `msd` are creatable in the sandbox; attach to the rest by id | [Robot lifecycle](../ch06-services/01-lifecycle.md) |
| | The first state sample never arrived within `probe_timeout` | Run with `RUST_LOG=vrobots_sdk=debug` to see which of the four connect steps stalled | [What connect actually does](../ch02-concepts/05-connect.md) |
| A command has no visible effect | It reached a different robot | Compare the `sys_id` you connected with against `topic list` | [System ids](../ch02-concepts/03-sys-id.md) |
| | The value is inside the band but does nothing useful, for example 1501 on a multirotor | Watch `s.actuator.pwm`: if it echoes your value, the command landed and the plant is the question | [Hello control](04-hello-control.md) |
| | The array length is wrong for that robot | The simulator logs and ignores it, keeping the previous latched value; the SDK cannot see that log | [Commands latch](../ch04-commands/01-latching.md) |
| | Nothing acts on that command id yet | `SET_MR_THROTTLE` and the body wrench commands are on the wire and unimplemented | [Commands nothing acts on](../ch04-commands/07-ignored-commands.md) |
| | A service acked `ok` and refused the value | Only `srv/skin` ever answers `ok = false`; measure the result in the state stream instead of trusting the ack | [Five rules that explain everything](../ch02-concepts/06-five-rules.md) |
| Camera frames never arrive | The simulator is not on this host | iceoryx2 is shared memory: frames cannot cross machines even when zenoh can | [Two transports, one simulator](../ch02-concepts/01-transports.md) |
| | The iceoryx2 pin differs from the simulator's | Compare `vrobots --version` against the simulator build; a patch mismatch delivers nothing and raises nothing | [Versions and pins](../ch08-tooling/03-version-and-pins.md) |
| | `open_camera` was given a name, resolution or format that does not match the publisher | Copy the stream key from `topic list` and split it back into its three parts | [Mount, open and unmount](../ch05-cameras/01-mount-open-unmount.md) |
| | You are calling `fresh()` faster than frames arrive | That is correct: `fresh()` returns `None` until a new frame lands. Use `latest()` if you want the current one regardless | [Freshness](../ch05-cameras/04-freshness.md) |
| Fields look like garbage | Schema drift between the SDK and the simulator | Compare `vrobots_msgs` and `schema_version` from `vrobots --version` against the simulator build | [Versions and pins](../ch08-tooling/03-version-and-pins.md) |
| | A vector was read in the wrong frame | Read `s.coord_frame_id`: the truck publishes `fru` while the multirotor and the Global Hawk publish `frd`, so the third component means opposite things | [Frames, axes and units](../ch02-concepts/07-frames-and-units.md) |
| | The quaternion was unpacked as `[w, x, y, z]` | The order is `[x, y, z, w]`, matching the wire's `Vec4` field order | [Frames, axes and units](../ch02-concepts/07-frames-and-units.md) |
| | Pose and twist were assumed to share a frame | Pose is world frame, twist and acceleration are body frame, in both the truth and the estimate blocks | [Kinematics](../ch03-reading-state/02-kinematics.md) |
| | An unconverged estimator is mirroring truth | Check `estimate.valid` before trusting `estimate.kin` | [Truth, measured and believed](../ch03-reading-state/01-truth-measured-believed.md) |
| A created multirotor will not move | Simulator bug: its rigidbody never integrates | Attach to the scene's multirotor by id instead of creating one | See the callout below |
| The numbers stopped changing | The simulator stopped, and `states()` keeps returning the last snapshot forever | Detect it with `wait_new_state(timeout)`, not by expecting an error | [Stream health](../ch03-reading-state/08-health.md) |
| The loop stutters at the right average rate | Samples are being dropped | `topic hz` reports max interval and gap count, which is what a mean rate hides | [Measuring rates](../ch08-tooling/05-rates.md) |

> **Sim bug.** In simulator v3.0.0 a client-created multirotor does not move. Its rigidbody
> never integrates, so it hangs where it spawned and ignores every pulse width and even a
> direct body force, while its actuator echo and rotor-speed model answer perfectly
> normally. Created trucks and mass-spring-dampers have live physics, and the scene's own
> multirotor flies. See `issues/created-multirotor-frozen-dynamics.md`. The examples that
> need a multirotor to move take an optional `sys_id` so you can attach to the scene's one
> instead.

## When the table does not have it

Turn the logs up. `RUST_LOG` overrides whatever filter the program passed to
`init_logging`, and the SDK's own tracing events name each connect step as it happens.

```sh
RUST_LOG=vrobots_sdk=debug cargo run -p vrobots-examples --bin ex01_hello_states
```

`RUST_LOG` reaches the Rust core, so the other two surfaces turn the volume up in their own
idiom instead. Python routes the same events into the standard `logging` module, so
`vrsdk.init_logging("debug")` (or raising the `vrobots_sdk` logger's level yourself) is the
equivalent. C++ registers a handler with `vrsdk::set_log_callback` and then calls
`vrsdk::set_log_level(vrsdk::LogLevel::Debug)`. [Logging](../ch08-tooling/04-logging.md)
covers all three.

Add zenoh's own view with `RUST_LOG=vrobots_sdk=debug,zenoh=info`. iceoryx2 logs to stderr
outside `tracing` entirely and is controlled by `IOX2_LOG_LEVEL`, which the SDK defaults to
errors only.

Two counters are worth printing from inside a loop that misbehaves without failing:
`stats()` carries received, decode error, sequence gap and missed sample counts, and
`last_error()` holds the most recent decode error. Neither tears down the session, which
is the point: a malformed payload is counted and the loop keeps running.

**Next:** [Concepts](../ch02-concepts/00-intro.md)

**See also:** [Logging](../ch08-tooling/04-logging.md), [Appendix C: Error reference](../appendix-c-errors.md), [Known simulator issues](../ch07-robots/07-known-issues.md)
