# System ids, and the two kinds of robot

Scene-authored robots you attach to, created robots you spawn, and why an id is never a constant.

## Two kinds of robot

A `sys_id` is a `u32` that names one robot for as long as the scene is loaded. Every
topic carries it, every command is routed by it, and there are exactly two ways to
end up holding one.

**Scene-authored robots** are the ones the scene placed. They exist the moment the
simulator enters Play mode, before any client connects, and they are still there
after every client exits. You reach them by naming their id:
`VirtualRobot::connect(RobotType::Multirotor, Some(1))`.

**Created robots** are the ones the SDK spawned. `connect(type, None)` asks the
manager's `srv/create` and the reply carries a fresh id. They are yours in the sense
that you know their id, and not in any stronger sense: nothing stops another client
attaching to them, and they survive your process exiting.

| | Scene-authored | Created |
|---|---|---|
| How you get one | `connect(type, Some(id))` | `connect(type, None)` |
| Touches `srv/create` | never | once |
| Constrained by the spawn catalog | no | yes |
| Available types | whatever the scene contains | the running scene's catalog |
| Exists before you connect | yes | no |

The attach path is the more capable of the two. It never queries the catalog, so it
works for any robot the scene contains regardless of what can be created, and it is
the only way to reach three of the six robot types.

## Ids are allocated at load time and keep incrementing

On a fresh boot straight into the Flatworld scene, `sys_id 0` is the truck and
`sys_id 1` is the multirotor. Every example up to `ex20` names its id in a `const`
for exactly that reason.

Those constants are a convenience and **never a contract**. Ids are allocated as the
scene loads and keep incrementing across scene loads, so reloading the scene, or
creating and deleting a robot, moves the numbers. A `const SYS_ID: u32 = 1` that
worked this morning can address a robot that no longer exists this afternoon, and the
failure is a `connect` that times out waiting for a first state sample rather than
anything that names the id as the problem.

> **Gotcha.** `connect` reporting no state on `vrobots/<id>/z/state` within
> `probe_timeout` usually means that id is wrong, not that the simulator is down.
> Run `vrobots topic list` and read the ids that are actually publishing.

The habit that makes this a non-issue: take the id as a command-line argument rather
than compiling it in. That is what `ex29` through `ex33` do, because a cart-pole, a
half-drone and a Global Hawk cannot be created and their ids are not knowable when
the example is written.

## The spawn catalog belongs to the scene

The set of types `srv/create` will spawn is registered by the scene's own spawner,
not by the SDK. The sandbox scene registers three:

| Catalog key | `RobotType` | Creatable in the sandbox |
|---|---|---|
| `multirotor` | `RobotType::Multirotor` | yes |
| `truck` | `RobotType::Truck` | yes |
| `msd` | `RobotType::Msd` | yes |
| `cartpole` | `RobotType::CartPole` | no, scene-authored only |
| `halfdrone` | `RobotType::HalfDrone` | no, scene-authored only |
| `globalhawk` | `RobotType::GlobalHawk` | no, scene-authored only |

Keys are matched exactly and case-sensitively on the wire.
`RobotType::catalog_key()` produces the wire form, and
`RobotType::from_catalog_key(&str)` parses it back case-insensitively, accepting the
synonyms the examples and the simulator's UI use: `car` for a truck, `cart_pole` and
`invpen` for a cart-pole, `mass_spring_damper` for an MSD, `half_drone`,
`global_hawk` and `rq4b`. Only `catalog_key()` is ever put on the wire.
`RobotType::is_in_sandbox_catalog()` reports the third column above.

## The refusal message is the catalog

A key the running scene does not register is refused with `ok = false`, and the
refusal names every key that scene *does* know:

```text
unknown type 'globalhawk' (known: multirotor, truck, msd)
```

`connect` surfaces that string verbatim as `VrError::Service`. That makes a failed
create the **only live way to enumerate the catalog**: there is no list service, and
the table above is a measurement of the sandbox scene rather than a property of the
SDK. Another scene may register more, and the refusal from that scene will say so.

## The three you can only attach to

`cartpole`, `halfdrone` and `globalhawk` exist as scene-authored robots and must be
reached by id.

The Global Hawk is a further step removed: it lives in the **IMU scene**, not the
sandbox. Launching the sandbox and looking for one finds nothing, and no amount of
correcting the id helps. Load the right scene first, then read the ids off
`vrobots topic list`.

> **Note.** Tier gating changes what is reachable. Under Pro every robot in the
> scene serves simultaneously; under Guest only the robot selected in the simulator's
> SYS-ID dropdown has open ports, so a correct id can still time out.
> [Known simulator issues](../ch07-robots/07-known-issues.md) has the detail.

## Deletion is explicit, always

Nothing in the SDK removes a robot implicitly. Dropping a `VirtualRobot` closes the
session and leaves the robot exactly as it was, still running, still publishing,
still latched on its last command. Only `delete()` removes one, and a handle whose
robot has been deleted is spent: further commands return an error rather than
silently doing nothing.

That is the fourth of [the five rules](06-five-rules.md), and
[Robot lifecycle](../ch06-services/01-lifecycle.md) covers the full sequence.

**Next:** [The shape of a program](04-program-shape.md)

**See also:** [Robot lifecycle](../ch06-services/01-lifecycle.md), [Supported virtual robots](../ch07-robots/00-intro.md), [What connect actually does](05-connect.md)
