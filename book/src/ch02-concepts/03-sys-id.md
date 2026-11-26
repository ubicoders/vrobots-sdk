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
works for any robot the scene contains regardless of what can be created or which
plan you are on, and it is the only way to reach three of the eleven robot types.

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

## Some scenes pin their ids

A scene can author a robot with a **pinned** id. A pinned robot registers under that
number on every load, and ids handed out afterwards start above the highest pin, so
a created robot never collides with one.

| Scene | Robot | `sys_id` | Pinned |
|---|---|---|---|
| Warehouse | forklift | 0 | yes |
| Warehouse | pallet robot | 1 | yes |
| Warehouse | rail arm | 2 | yes |
| TableArm | table arm | 0 | yes |
| Sandbox | turret | allocated at load | no |

The examples for these robots default to the pinned id and still accept another on
the command line, because a robot of the same type that you created has an id of its
own. The turret has no pin, so its example finds it: it lists the robots on the wire
and reads each one's actuator layout from the
[properties topic](../ch06-services/13-properties.md) until one reports two servos
and a laser. That is the general answer to "which id is the robot I want", and it
works for any type.

> **Gotcha.** Two scenes pin `sys_id` 0 to different robots: the forklift in the
> Warehouse and the table arm in the TableArm scene. An id names a robot within the
> loaded scene and says nothing about its type, and `connect` takes the type you
> pass on trust. Read the layout once after connecting when the type matters.

## The spawn catalog belongs to the scene

The set of types `srv/create` will spawn is registered by the scene's own spawner,
not by the SDK. The sandbox scene registers eight:

| Catalog key | `RobotType` | Creatable in the sandbox |
|---|---|---|
| `multirotor` | `RobotType::Multirotor` | yes |
| `truck` | `RobotType::Truck` | yes |
| `msd` | `RobotType::Msd` | yes |
| `forklift` | `RobotType::Forklift` | yes |
| `palletrobot` | `RobotType::PalletRobot` | yes |
| `kr210_rail` | `RobotType::RailArm` | yes |
| `tablearm` | `RobotType::TableArm` | yes |
| `turret` | `RobotType::Turret` | yes |
| `cartpole` | `RobotType::CartPole` | no, scene-authored only |
| `halfdrone` | `RobotType::HalfDrone` | no, scene-authored only |
| `globalhawk` | `RobotType::GlobalHawk` | no, scene-authored only |

The rail arm is the one type whose key does not resemble its name: `RobotType::RailArm`
is `kr210_rail` on the wire. Python spells the variants in capitals, as
`RobotType.RAILARM`, and reads the key back with `RobotType.RAILARM.key`.

> **Note.** Creating a robot at runtime needs the Pro plan. On the Community plan the
> manager refuses every create, whatever the key, with a message that names the
> plan. Attaching by id is not affected.

Keys are matched exactly and case-sensitively on the wire.
`RobotType::catalog_key()` produces the wire form, and
`RobotType::from_catalog_key(&str)` parses it back case-insensitively, accepting the
synonyms the examples and the simulator's UI use: `car` for a truck, `cart_pole` and
`invpen` for a cart-pole, `mass_spring_damper` for an MSD, `half_drone`,
`global_hawk`, `rq4b`, `pallet_robot`, `railarm`, `rail_arm`, `kuka_kr210_rail` and
`table_arm`. Only `catalog_key()` is ever put on the wire.
`RobotType::is_in_sandbox_catalog()` reports the third column above.

## The refusal message is the catalog

A key the running scene does not register is refused with `ok = false`, and the
refusal names every key that scene *does* know:

```text
unknown type 'bogus_type' (known: multirotor, truck, msd, forklift, palletrobot, kr210_rail, tablearm, turret)
```

That is the manager's reply to a create for the made-up key `bogus_type`. `connect`
carries the same text, word for word, inside its `VrError::Service`. Asking the sandbox
for a cart pole fails with:

```text
service: manager refused to create a cartpole: unknown type 'cartpole' (known: multirotor, truck, msd, forklift, palletrobot, kr210_rail, tablearm, turret). A robot type the scene does not register cannot be created, and runtime create needs the Pro tier; attach to an existing one with connect(.., Some(sys_id)) instead
```

That makes a failed create the **only live way to enumerate the catalog**: there is no
list service, and
the table above is a measurement of the sandbox scene rather than a property of the
SDK. Another scene may register a different set, and the refusal from that scene
will say so.

## A created robot spawns where you put it

A create carries an optional pose, and without one the robot appears at the world
origin, unrotated, so two robots created that way overlap. The pose is in **raw Unity
world coordinates**: metres, left-handed, +x right, +y up, +z forward, with the
rotation as a Unity quaternion `[x, y, z, w]`. Nothing converts it. The frame your
connection names for its outgoing vectors does not apply to the spawn pose, which is
the one place where [your frame](07-frames-and-units.md) is ignored.

The pose is set on the connection, before `connect`:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`crates/vrobots-sdk/src/options.rs`:

```rust
pub struct SpawnPose {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

pub fn new(position: [f64; 3], orientation: [f64; 4]) -> SpawnPose
pub fn at(position: [f64; 3]) -> SpawnPose

pub fn with_initial_pose(mut self, pose: SpawnPose) -> Self
```

{{#endtab }}
{{#tab name="C++" }}

`include/vrobots_sdk.hpp`:

```cpp
inline void set_initial_pose(vrsdk_connect_options_t& options, const Vec3& position,
                             const Quat& orientation = IDENTITY_QUAT)
```

{{#endtab }}
{{#tab name="Python" }}

`vrsdk/_vrsdk.pyi`, two of the keyword arguments of `VirtualRobot(...)`:

```python
initial_position: Optional[Sequence[float]] = None,
initial_orientation: Optional[Sequence[float]] = None,
```

{{#endtab }}
{{#endtabs }}

Those are signatures, so there is no output. In Rust the pose is a `SpawnPose`:
`SpawnPose::new(position, orientation)`, or `SpawnPose::at(position)` for an unrotated
robot, handed to `ConnectOptions::with_initial_pose`. A heading of `yaw` radians about the up
axis is the quaternion `[0, sin(yaw / 2), 0, cos(yaw / 2)]`, and a positive `yaw`
turns +z toward +x, which is clockwise seen from above. The quaternion is normalized
before it is sent, and a non-finite value or a zero-length rotation is refused by
`connect` before anything reaches the simulator.

The spawn pose is also the robot's home, the pose a reset returns it to. It is
ignored when you attach to an existing id. The two arms are held in place rather
than simulated as free bodies, so they stay exactly where the pose puts them: give
an arm a pose on the floor and clear of everything else.

> **Gotcha.** A created table arm starts with every joint at zero, which is not the
> pose the TableArm scene's own arm starts in. Read the joints after connecting
> rather than assuming a home pose.

## The three you can only attach to

`cartpole`, `halfdrone` and `globalhawk` exist as scene-authored robots and must be
reached by id.

The Global Hawk is a further step removed: it lives in the **IMU scene**, not the
sandbox. Launching the sandbox and looking for one finds nothing, and no amount of
correcting the id helps. Load the right scene first, then read the ids off
`vrobots topic list`.

> **Note.** The plan changes what is reachable. On the Pro plan every robot in the
> scene serves simultaneously; on the free Community plan only the robot selected in
> the simulator's SYS-ID dropdown has open ports, so a correct id can still time out.
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
