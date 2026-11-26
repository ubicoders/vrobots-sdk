# Configuring a lidar

Moving a lidar on its robot and choosing how a revolution is sampled, by name, through the lidar configuration service.

`configure_lidars` is the one call in this chapter that changes the simulator. Everything
before this page subscribes and reads. This page writes, and a write on the Community plan
is refused.

## What you can change, and what you cannot

Each entry of the request names one lidar the robot already carries and sets one or more of
three things. A setting you leave out is left alone, and a lidar you do not name is not
touched.

| Setting | Type | Units | Default | Notes |
|---|---|---|---|---|
| `mount` | position and rotation, three numbers each | m, rad | per robot, listed under Hardware on each [robot page](../ch07-robots/00-intro.md) | where the lidar sits, in the **robot's** frame. Position and rotation travel together: a pose is one value. A value that is not finite refuses the mount, and the entry's other settings still apply |
| `segments` | unsigned integer | | 1 | pose snapshots per revolution. Clamped by the simulator to 1 through 5 on a 2-D unit and 1 through 2 on a 3-D unit. Applies from the next revolution |
| `layer_mask` | integer bit mask, 32 bits | | every layer except layer 2 | which of the simulator's collision layers the beams can hit, one bit per layer. 0 restores the default. See [The layer mask](#the-layer-mask) |

**A lidar's name and kind are fixed.** Nothing here mounts a new lidar, renames one, removes
one, or turns a 2-D unit into a 3-D one. The ring count, the column count, the range limits
and the scan rate belong to the unit and are not settings either.

`segments` is how many times the simulator samples the robot's pose while it casts one
revolution. With 1 the whole revolution is cast from a single pose, so a moving robot
produces a scan with no spin distortion. A higher count spreads the revolution over several
poses, the way a real spinning unit sees the world while its carrier moves. It never changes
the scan rate.

## The layer mask

Every object in a scene sits on one of 32 collision layers, numbered 0 to 31. `layer_mask`
selects the layers a lidar's beams can return from: bit `n` of the integer is layer `n`, and
a beam passes straight through anything on a layer whose bit is clear, as if it were not
there. Something that is hidden this way does not shorten the beam and does not shadow what
is behind it.

| Value | Meaning |
|---|---|
| 0 | restore the default. It does not mean "no layers": the simulator replaces 0 with the default mask |
| the default | every layer except layer 2, the layer reserved for things rays are meant to ignore. As a signed 32-bit integer that is `-5` |
| `1 << n` | layer `n` only |
| a sum of such bits | exactly those layers |

The shipped scenes name five layers and leave the rest unassigned:

| Layer | Name | Bit value |
|---|---|---|
| 0 | default: the ground, buildings, props and the robots | 1 |
| 1 | transparent effects | 2 |
| 2 | ignored by rays | 4 |
| 4 | water | 16 |
| 5 | user interface | 32 |

<!-- VERIFY: which layer each kind of scene object sits on. The layer names are the simulator's; that the ground, props and robots of every shipped scene are on layer 0 is not confirmed scene by scene. -->

You rarely need to touch it. The two cases that call for it are a scene that puts a class of
objects on a layer of its own, which you then hide from one lidar by clearing that bit, for
instance `-5 & ~16` to see through water, and undoing an earlier change, for which you send
0. The mask is per lidar, so two units on one robot can see different sets.

The lidar's own robot is looked through whatever the mask says, so there is no bit to clear
for that.

> **Gotcha.** A mask without bit 0 blinds the lidar to nearly everything in the shipped
> scenes: every range comes back `+infinity` and nothing is refused. The scan carries the
> mount and the segment count it was cast with, and it does not carry the mask, so an
> all-infinite scan after a `layer_mask` change is the only evidence. Send 0 to recover.

## The mount is in the robot's frame

A lidar mount is the exception to the rule that vectors you send are read in your own header
frame. The request names no frame, and the simulator reads the mount in the robot's own
frame: `"fru"` on the truck, the forklift and the pallet robot, `"frd"` on the multirotor.
Rotation goes out in **radians**, about that frame's x, y and z axes.

That is exactly the frame and the shape in which every scan reports its mount, so the safe
way to move a lidar is to read the mount off a scan, change it, and send it back. Raising
the forklift's lidar by 10 cm, where z is up in the forklift's frame:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

Written against the `vrobots-sdk` crate:

```rust
let lidar = robot.open_lidar_3d("lidar")?;
lidar.wait_new_scan(Duration::from_secs(1))?;
if let Some(scan) = lidar.latest() {
    let p = &scan.params;
    let [x, y, z] = p.mount_position;
    // 10 cm up: z is up in a forklift's FRU.
    let entry = LidarConfigEntry::new("lidar").with_mount([x, y, z + 0.10], p.mount_euler_rad);
    robot.configure_lidars(&[entry])?;
}
```

{{#endtab }}
{{#tab name="C++" }}

From the `configure_lidars` documentation in `include/vrobots_sdk.hpp`:

```cpp
vrsdk::LidarStream lidar = robot.open_lidar_3d("lidar");
lidar.wait_new_scan(1.0);
vrsdk::LidarConfigEntry entry;
entry.name = "lidar";
entry.mount = lidar.latest().value().mount();
entry.mount->position[2] += 0.10;   // 10 cm up: z is up in a forklift's FRU
robot.configure_lidars({entry});
```

{{#endtab }}
{{#tab name="Python" }}

From the `LidarConfigEntry` documentation in the `vrsdk` package:

```python
# Raise the forklift's lidar 10 cm: z is up in its FRU frame.
p = lidar.latest.params
x, y, z = p.mount_position
forklift.configure_lidars(
    [vrsdk.LidarConfigEntry("lidar", mount=((x, y, z + 0.10), p.mount_euler_rad))]
)
```

{{#endtab }}
{{#endtabs }}

Rust and C++ have a `mount()` helper on the scan that returns the pose as one value. Rust's
`with_mount` takes the two arrays as arguments, and `with_lidar_mount` takes that value.
Python passes the pose as a pair, `(position, euler_rad)`. In all three an entry is addressed by the lidar's name and nothing else.

The call prints nothing and returns once the simulator has acknowledged the request.

<!-- VERIFY: the two fragments above are the SDK's own documentation examples; neither has been run against a live simulator from this book. -->

## Changing the sampling

`segments` is set the same way, alone or beside a mount. One request can carry several
entries, one per lidar, which is how you tune both of the pallet robot's units at once:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

Written against the `vrobots-sdk` crate:

```rust
palletrobot.configure_lidars(&[
    LidarConfigEntry::new("lidar_front_left").with_segments(5),
    LidarConfigEntry::new("lidar_rear_right").with_segments(5),
])?;
```

{{#endtab }}
{{#tab name="C++" }}

Written against `include/vrobots_sdk.hpp`:

```cpp
vrsdk::LidarConfigEntry front;
front.name = "lidar_front_left";
front.segments = 5;
vrsdk::LidarConfigEntry rear;
rear.name = "lidar_rear_right";
rear.segments = 5;
palletrobot.configure_lidars({front, rear});
```

{{#endtab }}
{{#tab name="Python" }}

Written against `vrsdk/_vrsdk.pyi`:

```python
palletrobot.configure_lidars(
    [
        vrsdk.LidarConfigEntry("lidar_front_left", segments=5),
        vrsdk.LidarConfigEntry("lidar_rear_right", segments=5),
    ]
)
```

{{#endtab }}
{{#endtabs }}

Nothing is printed here either. A count above the unit's maximum is not an error: the
simulator clamps it and carries on.

<!-- VERIFY: the two fragments above are written against the public API and were not compiled or run; no example calls configure_lidars. -->

## Where the confirmation is

The acknowledgement is a receipt. It says the simulator accepted the request, not what it
did with each entry. **The confirmation is the next scan**: every scan's `params` carries
the mount and the `segments` it was cast with, so read them back and compare.

That read-back matters because the simulator skips what it cannot apply and still
acknowledges the request:

| Request | What happens |
|---|---|
| An entry naming a lidar the robot does not carry | skipped, the other entries apply, acknowledged as ok |
| Any entry, on a robot that carries no lidar | skipped, acknowledged as ok |
| A mount with a value that is not finite | the mount is refused, the entry's other settings apply |
| `segments` outside the unit's range | clamped to the nearest allowed value |

A typo in a name is therefore invisible in the reply. The only symptom is a scan whose
`params` did not change.

## What the SDK refuses before sending

Five requests never reach the simulator. Each fails with an invalid-argument error that
names what was wrong: `VrError::InvalidArgument` in Rust, `VRSDK_ERR_INVALID_ARGUMENT` in
C++ and `vrsdk.err.INVALID_ARGUMENT` in Python.

| Request | Why it is refused |
|---|---|
| No entries | there is nothing to configure |
| An empty name, or one with a character other than a letter, a digit, `_` or `-` | it cannot be a lidar's name |
| The same name twice | the simulator would apply the first and ignore the rest |
| An entry that sets nothing | it would change nothing and still be acknowledged |
| A mount value that is not finite | the simulator would refuse the mount |

## On the Community plan

Lidar is a Pro plan feature, and this service is the one place where the simulator says so
in words. On Community every request that carries entries is refused in the reply, and the
call fails with a service error: `VrError::Service` in Rust, `VRSDK_ERR_SERVICE` in C++ and
`vrsdk.err.SERVICE` in Python. The error carries the simulator's message unchanged, with
the name of the plan in it:

```text
lidars are not available on the <plan> plan
```

Do not retry it. Nothing about the request is wrong, and the answer does not change until
the plan does. This is a clearer signal than the open calls can give, since a missing topic
cannot say why it is missing.

> **Note.** The reference for this service, with its key and the full request layout, is on
> [the lidar configuration service page](../ch06-services/12-lidar-config.md). This page is
> the narrative; that one is the table to come back to.

**Next:** [Safety events](../ch10-events/00-safety-events.md)

**See also:** [Points and frames](03-points-and-frames.md), [Coordinate frames](../ch06-services/04-frames-config.md), [Services and configuration](../ch06-services/00-intro.md)
