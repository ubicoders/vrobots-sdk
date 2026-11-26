# Lidar configuration

The reference for the lidars service: one entry per lidar, addressed by name, and three
settings per entry.

This page is the lookup table. [Configuring a lidar](../ch09-lidar/04-configuring.md) is the
narrative, with a worked program that reads a mount off a scan, changes it and sends it
back.

## Who serves it

`configure_lidars` sends `srv/lidars`. Every robot serves it, whether or not it carries a
lidar, and it needs the Pro plan.

| Situation | What comes back |
|---|---|
| Pro plan, the named lidar is mounted | `ok`; the change applies on the next physics step |
| Pro plan, the name matches no mounted lidar | `ok`; that entry is skipped, the others still apply |
| Pro plan, the robot carries no lidar | `ok`; every entry is skipped |
| Community plan | a refusal, surfaced as [`VrError::Service`](../appendix-c-errors.md) carrying the simulator's message, `lidars are not available on the <plan> plan` |

That refusal makes this one of the two services that ever answer no; [skins](08-skins.md) is
the other. For everything else on this page the
[rules every config service shares](00-intro.md#rules-every-config-service-shares) apply.

## The signature


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`crates/vrobots-sdk/src/lidar.rs`:

```rust
pub struct LidarConfigEntry {
    pub name: String,
    pub mount: Option<LidarMount>,
    pub segments: Option<u32>,
    pub layer_mask: Option<i32>,
}

pub fn new(name: impl Into<String>) -> LidarConfigEntry
pub fn with_mount(mut self, position: [f64; 3], euler_rad: [f64; 3]) -> Self
pub fn with_lidar_mount(mut self, mount: LidarMount) -> Self
pub fn with_segments(mut self, segments: u32) -> Self
pub fn with_layer_mask(mut self, mask: i32) -> Self

pub fn configure_lidars(&self, entries: &[LidarConfigEntry]) -> VrResult<()>
```

{{#endtab }}
{{#tab name="C++" }}

`include/vrobots_sdk.hpp`:

```cpp
struct LidarConfigEntry {
    std::string name;
    std::optional<LidarMount> mount;
    std::optional<std::uint32_t> segments;
    std::optional<std::int32_t> layer_mask;
};

void configure_lidars(const std::vector<LidarConfigEntry>& entries)
```

{{#endtab }}
{{#tab name="Python" }}

`vrsdk/_vrsdk.pyi`:

```python
class LidarConfigEntry:
    def __init__(
        self,
        name: str,
        *,
        mount: Optional[Sequence[Sequence[float]]] = None,
        segments: Optional[int] = None,
        layer_mask: Optional[int] = None,
    ) -> None: ...

def configure_lidars(self, entries: Sequence[LidarConfigEntry]) -> None: ...
```

{{#endtab }}
{{#endtabs }}

In Rust and C++ a `LidarMount` holds `position` and `euler_rad`, three numbers each, and
Rust's `with_mount(position, euler_rad)` builds one in place. In Python `mount` is the same
pair as two sequences, position first. `None` in Rust, an empty optional in C++ or an
omitted keyword in Python leaves that setting alone.

## One entry

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `name` | string | | required | the name the lidar is mounted under, which is also its topic segment; matched exactly |
| `mount` | position and rotation | m, rad | untouched | the optical centre's offset from the robot origin and the rotation about the robot frame's x, y and z axes; in the **robot's** frame, not your connection's; a value that is not finite refuses the mount while the entry's other settings still apply |
| `segments` | unsigned integer | | untouched | pose snapshots per revolution; clamped to 1 through 5 on a 2-D lidar and 1 through 2 on a 3-D lidar; applies from the next revolution and never changes the scan rate |
| `layer_mask` | integer bit mask, 32 bits | | untouched | which of the simulator's 32 collision layers the beams can return from, bit `n` for layer `n`; 0 restores the default, every layer except layer 2 (`-5` as a signed integer) |

Position and rotation travel together, because a pose is one value. A lidar's name and its
kind are fixed: this service never mounts, renames or unmounts one.

`layer_mask` decides what a lidar can see at all. A beam passes through anything on a layer
whose bit is clear, without being shortened or shadowed by it, and the lidar's own robot is
looked through whatever the mask says. Leave it alone unless a scene keeps a class of
objects on a layer of its own that one lidar should not see, and send 0 to undo a change.
A mask that leaves out layer 0 returns `+infinity` for nearly every beam in the shipped
scenes, is acknowledged like any other, and cannot be read back.
[The layer mask](../ch09-lidar/04-configuring.md#the-layer-mask) lists the layers and
their bit values.

## What the SDK refuses

`configure_lidars` refuses these before anything reaches the wire, with
`VrError::InvalidArgument`:

| Refused | Why |
|---|---|
| an empty list of entries | it would change nothing |
| a name that is empty or is not usable as a topic segment | no lidar could match it |
| the same name twice | the simulator applies the first entry and ignores the rest |
| an entry that sets nothing | it would change nothing |
| a mount value that is not finite | the simulator would refuse the mount and still ack `ok` |

## Where the confirmation is

Every scan carries the mount and the segment count it was cast with, in its parameter block.
Read them off the next scans after the call; the ack does not carry them. The mount comes
back in the robot's frame, in the shape you sent it.

## The frame a lidar reports in

This service moves a lidar. It does not choose the frame its points are labelled in: that is
the frames service, where a lidar is the device `lidar/<name>`. See
[Coordinate frames](04-frames-config.md#lidars-start-in-their-own-frame).

**Next:** [Robot properties](13-properties.md)

**See also:** [Configuring a lidar](../ch09-lidar/04-configuring.md), [Lidar](../ch09-lidar/00-intro.md), [Coordinate frames](04-frames-config.md), [Skins](08-skins.md)
