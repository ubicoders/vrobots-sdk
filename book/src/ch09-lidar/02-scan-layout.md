# Inside a scan

The ranges, the order they come in, the layout that places each one, and when a scan arrives.

```sh
cargo run -p vrobots-examples --bin ex42_lidar
./target/cpp-build/ex42_lidar
python examples/python/ex42_lidar.py
```

A `LidarScan` is one full revolution: an owned, immutable snapshot that stays valid for as
long as you hold it. It is the same type from a 3-D unit and from a 2-D one.

## Every field

| Field | Type | Units | Notes |
|---|---|---|---|
| `t_ns` | `i64` | ns since the unix epoch | when the revolution **completed**. The same clock as `State::t_ns` and `Frame::t_ns` |
| `elapsed` | `f64` | s | counts from the robot's first state sample, the same epoch as `State::elapsed` |
| `seq` | `u64` | | the simulator's scan counter for this topic. Restarts at 0 when the topic reopens |
| `sys_id` | `u32` | | the robot the lidar is on |
| `coord_frame_id` | `String` | | the lidar's frame, `"flu"` unless overridden; see [Points and frames](03-points-and-frames.md) |
| `axis_convention` | `Axes` | | the enum tag beside the id; unspecified for `"flu"`, which has no enum value |
| `params` | `LidarParams` | | the unit, its mount and its beam layout, below |
| `ranges` | `rings * columns` values of `f32` | m | one range per beam, from the optical centre; `+infinity` for no return |

The layout rides with **every scan**, for the reason a camera's intrinsics ride with every
frame: there is no separate topic to join by timestamp, so a re-mounted lidar cannot fall
out of step with its ranges.

| `params` field | Units | Notes |
|---|---|---|
| `name` | | the lidar's name on the robot, also its topic segment |
| `lidar_id` | | index of this lidar on the robot, from 0 |
| `rings` | | 16 for the 3-D unit, 1 for a 2-D unit |
| `columns` | | beams per ring per revolution: 900 or 360 |
| `azimuth_start` | rad | azimuth of column 0 |
| `azimuth_step` | rad | azimuth from one column to the next, counter-clockwise seen from above |
| `elevation_start` | rad | elevation of ring 0, the lowest |
| `elevation_step` | rad | elevation from one ring to the next, positive up |
| `min_range` | m | returns nearer than this are not seen |
| `max_range` | m | a beam that finds nothing within this reads `+infinity` |
| `scan_rate` | Hz | revolutions per second, fixed by the unit: 20 or 10 |
| `segments` | | pose snapshots the revolution was cast from; see [Configuring a lidar](04-configuring.md) |
| `mount_position`, `mount_euler_rad` | m, rad | where the lidar sits, in the **robot's** frame |
| `mount_coord_frame_id`, `mount_axis_convention` | | the frame the two mount fields are expressed in |

C++ reads the layout through `scan.params()`, a plain struct with these field names, and
keeps `name` and `coord_frame_id` as strings on the scan itself.

## Beam order

The ranges come ring by ring, lowest ring first. Each ring starts at column 0, which points
forward, and turns counter-clockwise seen from above. Beam (`ring`, `column`) is therefore at

```text
index     = ring * columns + column
azimuth   = azimuth_start   + column * azimuth_step
elevation = elevation_start + ring   * elevation_step
```

`rings * columns` is the length of `ranges`, checked when the scan is decoded. The layout
is physical and fixed: no setting, and no frame override, changes the ranges or their order.

| | Rust | C++ | Python |
|---|---|---|---|
| All ranges | `scan.ranges`, a flat `Vec<f32>` | `scan.ranges`, a flat `std::vector<float>` | `scan.ranges`, a numpy `float32` array of shape `(rings, columns)` |
| One beam | `scan.range(ring, column)` | `scan.range(ring, column)` | `scan.range(ring, column)` |
| Its angles | `scan.azimuth(column)`, `scan.elevation(ring)` | compute from `scan.params()` | `scan.azimuth(column)`, `scan.elevation(ring)` |

`range` checks its bounds and returns nothing for a beam outside the layout. In Python,
`scan.ranges[ring, column]` indexes the array directly and `scan.ranges.ravel()` is the flat
beam order the other two languages hold. `scan.ranges` needs numpy; `scan.range` and
`scan.data`, the same values as raw little-endian bytes, do not.

## No return is infinity, never zero

A beam that hits nothing within `max_range` reads `+infinity`. It never reads 0, and it
never reads `max_range`. Every search over a scan therefore tests for a finite value, and a
minimum taken without that test is still correct, because infinity loses every comparison.

`range(ring, column)` hands the infinity back as a value: it is the scan's "no return", not
a missing beam. The example counts the returns and finds the nearest one:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex42_lidar.rs`:

```rust
let mut hits = 0;
let mut nearest_index = 0;
let mut nearest_range = f32::INFINITY;
for (i, &r) in scan.ranges.iter().enumerate() {
    if r.is_finite() {
        hits += 1;
        if r < nearest_range {
            nearest_range = r;
            nearest_index = i;
        }
    }
}
println!("  returns   {hits} of {} beams", scan.ranges.len());
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex42_lidar.cpp`:

```cpp
std::size_t hits = 0;
std::size_t nearest_index = 0;
float nearest_range = std::numeric_limits<float>::infinity();
for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
    const float r = scan->ranges[i];
    if (std::isfinite(r)) {
        ++hits;
        if (r < nearest_range) {
            nearest_range = r;
            nearest_index = i;
        }
    }
}
std::printf("  returns   %zu of %zu beams\n", hits, scan->ranges.size());
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex42_lidar.py`:

```python
hits = 0
nearest_index = 0
nearest_range = math.inf
for i in range(beams):
    r = scan.range(i // p.columns, i % p.columns)
    if math.isfinite(r):
        hits += 1
        if r < nearest_range:
            nearest_range = r
            nearest_index = i
print(f"  returns   {hits} of {beams} beams")
```

The script reads one beam at a time so that it runs without numpy. With numpy the same
count is `numpy.isfinite(scan.ranges).sum()`.

{{#endtab }}
{{#endtabs }}

On the forklift's 3-D unit the example prints the layout it read off the scan, then the
count:

```text
  layout    16 ring(s) x 900 columns = 14400 ranges, 20 Hz, segments 1
  column c  azimuth +0.0 + c x 0.40 deg (0 forward, counter-clockwise from above)
  ring r    elevation -15.0 + r x 2.0 deg (ring 0 the lowest)
  range     0.40 to 100.0 m; no return reads +inf, never 0
  mount     (+0.37, +0.00, +2.90) m, euler (+0, -0, +0) deg, in "fru" -- the robot's frame; the scan's own frame is "flu"
  returns   11673 of 14400 beams
```

The `mount` line is explained on [Points and frames](03-points-and-frames.md). On the pallet
robot's 2-D unit the same lines read:

```text
  layout    1 ring(s) x 360 columns = 360 ranges, 10 Hz, segments 1
  column c  azimuth +0.0 + c x 1.00 deg (0 forward, counter-clockwise from above)
  ring r    elevation +0.0 + r x 0.0 deg (ring 0 the lowest)
  range     0.15 to 12.0 m; no return reads +inf, never 0
  mount     (+0.40, -0.30, +0.07) m, euler (+0, +0, -45) deg, in "fru" -- the robot's frame; the scan's own frame is "flu"
  returns   173 of 360 beams
```

Python prints the two frame ids in single quotes.

## When a scan arrives

`t_ns` is the moment the revolution completed, on the simulator's clock. It is not the
moment the first beam was cast, and it is not the time your program received the scan.

**3-D scans do not arrive evenly.** The simulator completes a revolution on a whole physics
step, so 3-D scans land alternately 40 ms and 60 ms apart, which averages 20 Hz. Code that
differentiates between consecutive scans divides by the difference of their `t_ns`, never by
a constant 0.05.

`seq` counts scans on one topic, and it **restarts at 0 when the topic reopens**: after a
plan change, or when the robot is removed and created again. A sequence number that goes
backwards therefore means "restarted", not "corrupt". Gaps going forward are counted for
you, in the stream's `seq_gaps` and `missed_scans`.

Because both numbers come from the simulator, the example measures the rate from the scans
themselves rather than from its own clock:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex42_lidar.rs`:

```rust
let revolutions = last.seq.saturating_sub(scan.seq);
let seconds = (last.t_ns - scan.t_ns) as f64 * 1e-9;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex42_lidar.cpp`:

```cpp
const std::uint64_t revolutions = last.seq() > scan->seq() ? last.seq() - scan->seq() : 0;
const double seconds = static_cast<double>(last.t_ns() - scan->t_ns()) * 1e-9;
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex42_lidar.py`:

```python
revolutions = max(last.seq - scan.seq, 0)
seconds = (last.t_ns - scan.t_ns) * 1e-9
```

{{#endtab }}
{{#endtabs }}

Those two lines print nothing; they feed the `stream` line shown on
[Opening a lidar stream](01-opening.md).

## The pallet robot's units see through their own robot

Each 2-D unit publishes its full 360 degrees. The two units sit on opposite corners of the
pallet robot, so the quarter of each ring that faces inward points across the robot's own
body. A beam passes through the robot it is mounted on, so those columns do not read the
chassis: they read whatever lies beyond the far side of it.

> **Gotcha.** The inward quarter is not marked in the scan. It is the 90 degrees centred on
> the unit's rear, so discard every beam whose azimuth, wrapped to plus or minus 180
> degrees, is more than 135 degrees from forward. Do it before any nearest-obstacle search.

The mask is a few lines, written here against the scan API:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
use std::f64::consts::{PI, TAU};

let p = &scan.params;
let mut ring = scan.ranges[..p.columns as usize].to_vec(); // a 2-D scan has one ring
for column in 0..p.columns {
    // rem_euclid wraps the azimuth to [-pi, pi).
    let az = (scan.azimuth(column) + PI).rem_euclid(TAU) - PI;
    if az.abs() > 135.0_f64.to_radians() {
        ring[column as usize] = f32::INFINITY;
    }
}
```

{{#endtab }}
{{#tab name="C++" }}

```cpp
const vrsdk_lidar_params_t& p = scan->params();
std::vector<float> ring(scan->ranges.begin(), scan->ranges.begin() + p.columns);
for (std::uint32_t c = 0; c < p.columns; ++c) {
    // std::remainder wraps the azimuth to [-pi, pi].
    const double az = std::remainder(p.azimuth_start + c * p.azimuth_step, 2.0 * PI);
    if (std::fabs(az) > 135.0 * DEG_TO_RAD) {
        ring[c] = std::numeric_limits<float>::infinity();
    }
}
```

{{#endtab }}
{{#tab name="Python" }}

```python
p = scan.params
az = np.array([scan.azimuth(c) for c in range(p.columns)])
az = (az + np.pi) % (2.0 * np.pi) - np.pi  # wrap to [-pi, pi)
ring = scan.ranges[0]  # a 2-D scan has one ring
ring[np.abs(az) > np.radians(135.0)] = np.inf
```

{{#endtab }}
{{#endtabs }}

None of the fragments prints anything. Each leaves `ring` holding the outward 270 degrees, with
the inward quarter set to the scan's own no-return value so that later code needs no second
rule. `PI` and `DEG_TO_RAD` are the constants `ex42_lidar.cpp` defines, and `np` is numpy.

<!-- VERIFY: the two mask fragments above are written against the public API and were not compiled or run; no example contains them. -->

**Next:** [Points and frames](03-points-and-frames.md)

**See also:** [Timestamps and sequence numbers](../ch03-reading-state/06-timestamps.md), [Pallet robot](../ch07-robots/09-palletrobot.md), [Inside a frame](../ch05-cameras/03-frames.md)
