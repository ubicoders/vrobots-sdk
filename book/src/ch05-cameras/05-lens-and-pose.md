# Lens and mount pose

Where the camera sits, what it sees, and what the simulator substitutes for values it does not like.

```sh
cargo run -p vrobots-examples --bin ex17_camera_pose
./target/cpp-build/ex17_camera_pose
python examples/python/ex17_camera_pose.py
```

This is the page where a camera gets created rather than opened, and `ex17_camera_pose` is
the one example in the book that does it. Everywhere else the assumption holds that
**every vrobot already ships `front_left` and `front_right` at 720p rgba8**, and a reader
just opens one. You reach for `mount_camera` when that pair cannot serve: a camera
somewhere else on the robot, pointing somewhere else, through a different lens, or in a
different format.

`mount_camera` uses the defaults: at the robot origin, looking along its forward axis, 600
px focal length. `mount_camera_with` takes a `CameraOptions` and configures the mount and
the lens in the same call. Whatever it creates is yours to remove, and `unmount_camera` at
the end of the run is what keeps the robot's own two cameras the only ones left.

## What you can ask for

| Field | Units | Default | Simulator substitution |
|---|---|---|---|
| `mount_position` | m, in **your** header frame | `[0, 0, 0]` | re-expressed into the robot's frame |
| `mount_euler_deg` | **degrees**, Unity-local | `[0, 0, 0]` | taken as written, never re-expressed |
| `fx` | px at the current resolution | 600.0 | substitutes 600 for anything `<= 0` |
| `fy` | px at the current resolution | 600.0 | substitutes 600 for anything `<= 0` |
| `near_clip` | m | 0.5 | substitutes 0.5 for anything `<= 0` |
| `far_clip` | m | 1000.0 | substitutes 1000 for anything at or below the near plane |

The two mount fields are tagged differently on purpose, matching the simulator. Position is
a frame-tagged vector, re-expressed from the frame your headers declare into the robot's
own. The euler triple is a Unity-local mount convention and is taken as written, in
degrees, whatever frame you declared.

The lens is specified as intrinsics, not as a field of view. `fx != fy` renders anamorphic.
A smaller focal length is a wider angle: `fov_y = 2 * atan(height / (2 * fy))`, which puts
the 600 px default at about 61.9 degrees on a 720p frame and 400 px at about 84.0.

| Builder | Sets |
|---|---|
| `with_mount_position([f64; 3])` | `mount_position` |
| `with_mount_euler_deg([f64; 3])` | `mount_euler_deg` |
| `with_focal_length(f)` | `fx` and `fy` together, the square-pixel case |
| `with_clip(near, far)` | `near_clip` and `far_clip` |

From `examples/rust/src/bin/ex17_camera_pose.rs`, the whole configuration:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
let options = CameraOptions::default()
    .with_mount_position(MOUNT_POSITION)
    .with_mount_euler_deg(MOUNT_EULER_DEG)
    .with_focal_length(FOCAL_PX)
    .with_clip(0.2, 500.0);
println!("requested: {options:?}");

let cam = robot.mount_camera_with(CAMERA, RESOLUTION, FORMAT, &options)?;
println!("camera stream: {}", cam.service_name());
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex17_camera_pose.cpp`:

```cpp
// Start from the documented defaults, then override.
vrsdk_camera_options_t options;
vrsdk_camera_options_default(&options);
for (int i = 0; i < 3; ++i) {
    options.mount_position[i] = MOUNT_POSITION[i];
    options.mount_euler_deg[i] = MOUNT_EULER_DEG[i];
}
options.fx = FOCAL_PX;
options.fy = FOCAL_PX;
options.near_clip = 0.2;
options.far_clip = 500.0;

vrsdk::CameraStream cam = robot.mount_camera(CAMERA, RESOLUTION, FORMAT, &options);
std::printf("camera stream: %s\n", cam.service_name().c_str());
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex17_camera_pose.py`:

```python
cam = mr.mount_camera(
    CAMERA,
    RESOLUTION,
    FORMAT,
    mount_position=MOUNT_POSITION,
    mount_euler_deg=MOUNT_EULER_DEG,
    fx=FOCAL_PX,
    fy=FOCAL_PX,
    near_clip=0.2,
    far_clip=500.0,
)
print(f"camera stream: {cam.service_name}")
```

{{#endtab }}
{{#endtabs }}

Only Rust needs a second entry point. C++ takes an optional fourth argument on
`mount_camera`, defaulting to a null pointer, and Python takes the same settings as
keyword-only arguments, so there is no `mount_camera_with` in either.

**Start the C++ struct from `vrsdk_camera_options_default`, never from `{}`.** A zeroed
struct asks for a zero focal length and zero clip planes, which is not the defaults; Rust's
`CameraOptions::default()` and Python's omitted keywords are the equivalents.

With `MOUNT_EULER_DEG = [0.0, 0.0, 180.0]` the camera is upside down, and the stream name
is the usual one:

```text
camera stream: vrobots/1/i/cam/tilt/720p_rgb8
```

## What comes back

Both blocks ride with **every frame**, so a gimballed or re-mounted camera cannot desync
from its images and there is no camera-info topic to join by timestamp.

`frame.intrinsics` is read back from the live camera rather than echoed from your request,
because the simulator round-trips `fx` and `fy` through the Unity field of view. The
numbers can differ slightly from the ones you sent, and the read-back is the authority on
what rendered the frame.

| `Intrinsics` field | Units | Notes |
|---|---|---|
| `fx`, `fy` | px | read back from the camera, not echoed |
| `cx`, `cy` | px | always the image centre |
| `fov_y` | **radians** | `to_degrees()` before printing |
| `near_clip`, `far_clip` | m | |

Rendered images are an ideal pinhole. **There are no distortion coefficients**, because
there is no distortion.

| `MountPose` field | Units | Notes |
|---|---|---|
| `position` | m | in the **robot's** frame, not the frame the rest of your header is tagged with |
| `euler_rad` | **radians** | the request took degrees; the simulator converts |
| `axis_convention` | `Axes` | the convention `position` is expressed in |
| `coord_frame_id` | `String` | the frame id `position` is expressed in |

Degrees on the way in, radians on the way out, so the read-back needs converting before it
is comparable to what you asked for:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
// Degrees on the way in, radians on the way out: the wire is SI.
let euler_deg = [
    frame.mount.euler_rad[0].to_degrees(),
    frame.mount.euler_rad[1].to_degrees(),
    frame.mount.euler_rad[2].to_degrees(),
];
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex17_camera_pose.cpp`:

```cpp
const vrsdk_mount_pose_t& m = frame->info.mount;
const vrsdk_intrinsics_t& in = frame->info.intrinsics;

// Degrees on the way in, radians on the way out: the wire is SI.
const double euler_deg[3] = {m.euler_rad[0] * RAD2DEG, m.euler_rad[1] * RAD2DEG,
                             m.euler_rad[2] * RAD2DEG};
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex17_camera_pose.py`:

```python
m, i = frame.mount, frame.intrinsics

# Degrees on the way in, radians on the way out: the wire is SI.
euler_deg = tuple(round(math.degrees(a), 1) for a in m.euler_rad)
```

{{#endtab }}
{{#endtabs }}

The asymmetry is the wire's, not the binding's: `mount_euler_deg` goes out in degrees and
`mount.euler_rad` comes back in radians, in all three. Every surface converts at the same
place, and the field names carry their units for exactly this reason.

That fragment prints nothing itself; it feeds the pose line `ex17_camera_pose` prints
whenever the mount differs from the previous frame's.

## The read-back is not the numbers you sent

You express the mount in your frame; the robot converts it into its frame and reports it
back tagged with that frame. Against the test scene, the components move and change sign:

```text
requested  [0.10, 0.20, 0.30] m  "unity"
read back  (-0.20, +0.30, -0.10) m  "frd"    -- permuted and signed
```

Never assume your triple survives intact. `frame.mount` says where the camera actually is,
and comparing it against your request is the only way to confirm what the simulator did.
Read `frame.mount.coord_frame_id` rather than assuming a convention.

> **Note.** The service acks immediately, but the camera has to be rebuilt and re-rendered
> before it publishes. For a new camera the stream does not exist until that is done, so
> its very first frame already carries the requested pose. Re-mounting an existing name is
> the case to watch: the change ends the old stream and starts a new one, and anything
> still holding the old handle is reading a dead service.

The last thing `ex17_camera_pose` prints is a check on what was rendered rather than on
what the simulator was told. With the camera rolled 180 degrees, the sky lands in the
bottom rows of a buffer whose row 0 is still, always, the top:

```text
sky-ness (B-R) top=-25 bottom=+98 -> sky is at the BOTTOM: the camera really is upside down
```

<!-- VERIFY: the two sky-ness magnitudes above are the values ex03 measured upright, shown swapped for the rolled camera; they have not been re-measured against a live run of ex17. -->

**Next:** [Two cameras at once](06-two-cameras.md)

**See also:** [Inside a frame](03-frames.md), [Frames, axes and units](../ch02-concepts/07-frames-and-units.md), [Coordinate frames](../ch06-services/04-frames-config.md)
