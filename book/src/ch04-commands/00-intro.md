# Sending commands

Every actuator in the simulator is driven the same way: you publish a command, nothing
replies, and the state stream is the only receipt you get.

## The path a command takes

A command is a one-way publication onto a shared bus. Nothing in the diagram below travels
back along the arrow it came in on.

```mermaid
sequenceDiagram
    participant You as Your loop
    participant SDK
    participant Cmd as z/cmd
    participant Robot
    participant St as z/state

    You->>SDK: set_mr_pwm([1501.0; 4])
    SDK->>SDK: validate client-side
    SDK->>Cmd: FlatBuffers put (zenoh)
    Note over Cmd: many-to-many: every peer's<br/>traffic to this robot lands here
    SDK-->>You: Ok(()) = published
    Cmd->>Robot: queued
    Note over Robot: next physics step:<br/>drain queue, latch, ack nothing
    Robot->>St: snapshot at 25 Hz
    St->>You: states().actuator echoes it
```

In words, five steps:

1. You call a typed method such as `set_mr_pwm`, or [`send_cmd`](06-generic-cmd.md) for an
   id the SDK has no method for.
2. The SDK validates what it can client-side. A pulse width outside the band, a non-finite
   float or an empty slice returns `VrError::InvalidArgument` and nothing reaches the wire.
3. The SDK publishes one FlatBuffers payload to `vrobots/{sys_id}/z/cmd` over zenoh.
4. The robot drains its command queue at the start of its next physics step, and
   acknowledges nothing.
5. The only observable consequence is the actuator echo in the next state snapshot.

## Ok means published, not acted on

`Ok(())` from any command method means zenoh accepted the put. It says nothing about
whether a robot read the message, understood the id, or did anything with it. The id space
is shared across robot types, so a robot receiving an id it does not implement ignores it,
and that is correct behaviour rather than a fault.

> **Gotcha.** Wrong command id, wrong `sys_id` and wrong array length all present
> identically from outside: the state stream does not change. To tell them apart, print
> `s.actuator.pwm` (or `s.actuator.measured` on the plants that have no pulse widths) every
> iteration and watch whether it follows what you sent.

## Which command drives which robot

| Robot type | `catalog_key()` | Drive command | Page |
|---|---|---|---|
| `Multirotor` | `"multirotor"` | `SET_MR_PWM` | [Driving a multirotor](02-multirotor.md) |
| `Truck` | `"truck"` | `SET_CAR` | [Driving the truck](03-truck.md) |
| `Msd` | `"msd"` | `SET_MSD` | [Single degree of freedom plants](04-single-dof.md) |
| `CartPole` | `"cartpole"` | `SET_INVPEN` | [Single degree of freedom plants](04-single-dof.md) |
| `HalfDrone` | `"halfdrone"` | `SET_MR_PWM` with exactly 2 values | [Driving a multirotor](02-multirotor.md) |
| `GlobalHawk` | `"globalhawk"` | `SET_ANGVEL` tracking, or direct surfaces | [Fixed wing control](05-fixed-wing.md) |

Only `catalog_key()` is ever put on the wire. `RobotType::from_catalog_key` is
case-insensitive and accepts synonyms (`"car"`, `"cart_pole"`, `"invpen"`,
`"mass_spring_damper"`, `"half_drone"`, `"global_hawk"`, `"rq4b"`), which matters when a
robot type comes from a configuration file rather than from Rust.

## Floats narrow on the way out

The API is `f64` everywhere, for the same reason the snapshots are: one float type across
Rust, C++ and Python. The wire's `float_val`, `float_arr`, `Vec3` and `Vec4` are `f32`, so
every float you send narrows once on the way out. Integer channels such as pulse widths do
not: they are `i32` on the wire and the SDK rounds into them.

## Every vector carries a frame

Commands whose payload is a vector (`vec3` or `vec4`) are stamped with the
`coord_frame_id` and `axis_convention` from your `ConnectOptions`, and the robot converts
them into its own axes using the physically correct rule for that command. A force and a
torque convert differently, because a torque is a pseudovector and carries the handedness
sign. Array payloads such as `SET_MR_PWM` and `SET_FW_SURFACES` carry no frame at all: they
are per-channel numbers, and nothing is re-expressed.

**Next:** [Commands latch](01-latching.md)

**See also:** [Five rules that explain everything](../ch02-concepts/06-five-rules.md), [Actuators](../ch03-reading-state/05-actuator.md), [Appendix B: Command reference](../appendix-b-commands.md), [Appendix D: Glossary](../appendix-d-glossary.md)
