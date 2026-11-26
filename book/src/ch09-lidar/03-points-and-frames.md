# Points and frames

How a range becomes a point, which frame that point is in, and what a frame override does and does not change.

```sh
cargo run -p vrobots-examples --bin ex42_lidar
./target/cpp-build/ex42_lidar
python examples/python/ex42_lidar.py
```

## From a range to a point

A range `r` on a beam with azimuth `az` and elevation `el` is the point

```text
x = r * cos(el) * cos(az)
y = r * cos(el) * sin(az)
z = r * sin(el)
```

in the lidar's default frame, **FLU**: x forward, y left, z up, right-handed, with the
origin at the lidar's optical centre. Azimuth 0 is forward and grows counter-clockwise seen
from above, which is why a positive azimuth lands on positive y, the left.

The example writes the formula out once, for one beam, so that nothing about it is hidden:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex42_lidar.rs`:

```rust
fn beam_point(scan: &LidarScan, index: usize) -> [f64; 3] {
    let p = &scan.params;
    let columns = p.columns as usize;
    let ring = (index / columns) as f64;
    let column = (index % columns) as f64;
    let az = p.azimuth_start + column * p.azimuth_step;
    let el = p.elevation_start + ring * p.elevation_step;
    let r = f64::from(scan.ranges[index]);
    [
        r * el.cos() * az.cos(),
        r * el.cos() * az.sin(),
        r * el.sin(),
    ]
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex42_lidar.cpp`:

```cpp
vrsdk::Vec3 beam_point(const vrsdk::LidarScan& scan, std::size_t index) {
    const vrsdk_lidar_params_t& p = scan.params();
    const std::size_t columns = p.columns;
    const double ring = static_cast<double>(index / columns);
    const double column = static_cast<double>(index % columns);
    const double az = p.azimuth_start + column * p.azimuth_step;
    const double el = p.elevation_start + ring * p.elevation_step;
    const double r = scan.ranges[index];
    return {r * std::cos(el) * std::cos(az), r * std::cos(el) * std::sin(az), r * std::sin(el)};
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex42_lidar.py`:

```python
def beam_point(scan, index):
    """The point beam `index` hit, metres, in FLU: the wire's own formula.

    `index` is `ring * columns + column`. Only for a finite range: `+inf` is no
    return, and has no point.
    """
    p = scan.params
    ring = index // p.columns
    column = index % p.columns
    az = p.azimuth_start + column * p.azimuth_step
    el = p.elevation_start + ring * p.elevation_step
    r = scan.range(ring, column)
    return (
        r * math.cos(el) * math.cos(az),
        r * math.cos(el) * math.sin(az),
        r * math.sin(el),
    )
```

{{#endtab }}
{{#endtabs }}

`beam_point` is only called for a finite range: a beam with no return has no point. The
example calls it on the lowest ring at four bearings, front, left, back and right, and
prints one line per bearing in one of two forms. On the forklift's 3-D unit every bearing
had a return:

```text
  ring 0, elevation -15.0 deg, at four bearings:
     az   +0.0 deg   11.19 m  -> (+10.81,  +0.00,  -2.90) m FLU
     az  +90.0 deg    3.04 m  -> ( -0.00,  +2.93,  -0.79) m FLU
     az +180.0 deg   11.21 m  -> (-10.82,  -0.00,  -2.90) m FLU
     az +270.0 deg   11.20 m  -> ( +0.00, -10.82,  -2.90) m FLU
```

On the pallet robot's 2-D unit, in the same run, three of the four had none:

```text
  ring 0, elevation +0.0 deg, at four bearings:
     az   +0.0 deg     inf    -> no return
     az  +90.0 deg    8.37 m  -> ( +0.00,  +8.37,  +0.00) m FLU
     az +180.0 deg     inf    -> no return
     az +270.0 deg     inf    -> no return
```

The signs in those lines show the frame at a glance. A return straight ahead has a positive
`x`, one at +90 degrees has a positive `y` because left is positive, and every return on
ring 0 of a 3-D unit has a negative `z` because that ring looks 15 degrees down. Three of
the forklift's four returns are the floor, 2.90 m below the lidar.

## Letting the SDK build the points

You rarely need to write that formula. The scan builds points for you, one beam or all of
them.

| | Rust | C++ | Python |
|---|---|---|---|
| One beam | `scan.point(ring, column)`, `None` for no return | not provided; use the formula above | `scan.point(ring, column)`, `None` for no return |
| Every beam | `scan.points()`, a flat `Vec<f32>` of `3 * ranges.len()` | `scan.points()`, a flat `std::vector<float>` of `3 * ranges.size()` | `scan.points()`, a numpy `float32` array of shape `(rings, columns, 3)` |
| Frame of the result | `scan.points_frame_id`, a field | `scan.points_frame_id` | `scan.points_frame_id` |

`points()` keeps the cloud **organised**: point `i` is beam `i`, in the beam order of
[Inside a scan](02-scan-layout.md). A beam with no return is `(NaN, NaN, NaN)` rather than
being left out, so the array always has one point per beam. Drop the NaN rows when you want
an unorganised cloud. The points are computed on every call, so keep the result if you use
it twice.

{{#tabs global="lang" }}
{{#tab name="Rust" }}

Written against the `vrobots-sdk` crate:

```rust
let xyz = scan.points(); // [x0, y0, z0, x1, y1, z1, ...]
let hits = xyz.chunks_exact(3).filter(|p| !p[0].is_nan()).count();
println!("{hits} returns in {:?}", scan.points_frame_id);
```

{{#endtab }}
{{#tab name="C++" }}

Written against `include/vrobots_sdk.hpp`:

```cpp
const std::vector<float> xyz = scan->points();  // {x0, y0, z0, x1, y1, z1, ...}
std::size_t hits = 0;
for (std::size_t i = 0; i < xyz.size(); i += 3) {
    if (!std::isnan(xyz[i])) {
        ++hits;  // xyz[i], xyz[i + 1], xyz[i + 2] is one return
    }
}
std::printf("%zu returns in \"%s\"\n", hits, scan->points_frame_id.c_str());
```

{{#endtab }}
{{#tab name="Python" }}

Written against `vrsdk/_vrsdk.pyi`:

```python
xyz = scan.points().reshape(-1, 3)  # one row per beam, (x, y, z) in metres
hits = xyz[~np.isnan(xyz[:, 0])]  # drop the beams with no return
print(f"{len(hits)} returns in {scan.points_frame_id!r}")
```

{{#endtab }}
{{#endtabs }}

On a lidar in its default frame, all three print the number of returns and the frame id:

```text
<hits> returns in "flu"
```

Python prints the id in its own quotes, `'flu'`.

<!-- VERIFY: the two points() fragments above are written against the public API and were not compiled or run; no example calls points(). -->

## Points are in the lidar's frame, not the robot's

Every point above has its origin at the lidar's optical centre and its axes along the
lidar. **The mount is not applied.** A point at `(2, 0, 0)` is two metres in front of the
lidar, wherever the lidar sits on the robot.

The mount rides with every scan, in `params`, for a program that wants points on the body:

| Field | Units | Notes |
|---|---|---|
| `mount_position` | m | the optical centre's offset from the robot origin |
| `mount_euler_rad` | rad | the mount rotation about the robot frame's x, y and z axes, in that frame's own Euler order |
| `mount_coord_frame_id` | | the frame both are expressed in: the **robot's**, `"fru"` on the truck, the forklift and the pallet robot, `"frd"` on the multirotor |

Two frames are therefore in play in one scan, and they are different on purpose. The points
are in the lidar's frame (`coord_frame_id`, FLU by default). The mount is in the robot's
frame (`mount_coord_frame_id`). The example prints both on one line:

```text
  mount     (+0.37, +0.00, +2.90) m, euler (+0, -0, +0) deg, in "fru" -- the robot's frame; the scan's own frame is "flu"
```

That is the forklift's 3-D unit; Python prints the two ids in single quotes.

Read `mount_coord_frame_id` off the scan rather than assuming it. Taking points from the
lidar onto the body is then a change of frame, a rotation and an offset, in your own code;
[Rotation conversions](../ch02-concepts/08-rotation-conversions.md) has the tools for the
first two.

## Overriding the lidar's frame

A lidar is a device of the frames service, under the name `lidar/<name>`. Overriding it asks
for points in another frame, for example FRD to match a multirotor's state:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

Written against the `vrobots-sdk` crate:

```rust
robot.set_frames(None, &[DeviceFrame::new(device::lidar("lidar"), "frd")])?;
```

{{#endtab }}
{{#tab name="C++" }}

Written against `include/vrobots_sdk.hpp`:

```cpp
const std::vector<vrsdk::DeviceFrame> devices = {{vrsdk::device::lidar("lidar"), "frd"}};
robot.set_frames(devices);
```

{{#endtab }}
{{#tab name="Python" }}

Written against `vrsdk/_vrsdk.pyi`:

```python
robot.set_frames(None, [DeviceFrame(device.lidar("lidar"), "frd")])
```

{{#endtab }}
{{#endtabs }}

The call prints nothing. The confirmation is the `coord_frame_id` on the scans that follow,
which changes from `"flu"` to `"frd"`.

<!-- VERIFY: the two set_frames fragments above are written against the public API and were not compiled or run against a simulator. -->

**An override re-labels points. It never changes the ranges or their order.** The beam
layout is physical: column 0 still points along the lidar's forward axis, azimuth still
grows counter-clockwise seen from above, and `ranges` is the same array of numbers. What
changes is the frame `point()` and `points()` express their result in, and
`points_frame_id` names it. The hand-written `beam_point` above keeps returning FLU, since
it never looks at the frame.

`points_frame_id` and `coord_frame_id` agree except in one case. A frame the scene
registered at run time is one the SDK cannot resolve on its own, so the points stay in
`"flu"`, `points_frame_id` says `"flu"`, and `coord_frame_id` still names the override.

> **Gotcha.** A lidar does not start by inheriting its robot's frame: it starts in FLU.
> Clearing an override with `INHERIT_FRAME` therefore does not put a lidar back to FLU. It
> makes the lidar follow the robot's frame. To return to the default, override it to
> `"flu"` explicitly.

**Next:** [Configuring a lidar](04-configuring.md)

**See also:** [Frames, axes and units](../ch02-concepts/07-frames-and-units.md), [Coordinate frames](../ch06-services/04-frames-config.md), [Rotation conversions](../ch02-concepts/08-rotation-conversions.md)
