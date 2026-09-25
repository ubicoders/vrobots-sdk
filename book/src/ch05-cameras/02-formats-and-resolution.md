# Formats and resolution

The resolutions and pixel formats on offer, and the one setting that is robot-wide.

```sh
cargo run -p vrobots-examples --bin ex15_camera_formats
./target/cpp-build/ex15_camera_formats
python examples/python/ex15_camera_formats.py
```

## Resolution

| Variant | Accepted strings | Width x height |
|---|---|---|
| `Resolution::P360` | `"360p"`, `"360"` | 640 x 360 |
| `Resolution::P720` | `"720p"`, `"720"` | 1280 x 720, the simulator's default |
| `Resolution::P1080` | `"1080p"`, `"1080"` | 1920 x 1080 |

Bare heights are accepted because the service field is a number while the stream segment is
a `p`-suffixed string. Widths are the 16:9 partners of the height: `width() == height * 16 / 9`.

## Pixel format

| Variant | Accepted string | Bytes per pixel | Notes |
|---|---|---|---|
| `PixelFormat::Mono8` | `"mono8"` | 1 | 8-bit greyscale, the integer BT.601 luma of the render |
| `PixelFormat::Rgb8` | `"rgb8"` | 3 | R, G, B in that order, not BGR |
| `PixelFormat::Rgba8` | `"rgba8"` | 4 | R, G, B, A; the renderer's native readback and the cheapest for the simulator to publish |

Parsing is case-insensitive. Both strings are baked into the iceoryx2 service name, so a
format change is an unmount and remount rather than a field write, and every consumer has
to resubscribe under the new name.

## What a frame costs

Resolution times format is the whole story, since the pixels are tightly packed and there
is no compression anywhere on the path.

| resolution | mono8 (1 B) | rgb8 (3 B) | rgba8 (4 B) |
|---|---|---|---|
| 360p | 230 400 | 691 200 | 921 600 |
| 720p | 921 600 | 2 764 800 | **3 686 400**, what every vrobot ships |
| 1080p | 2 073 600 | 6 220 800 | 8 294 400 |

`CameraSpec::data_size()` returns that number for any combination. Sixteen times between the
corners is worth having for anything that needs only luminance and geometry: optical flow,
fiducials, horizon detection. The stream rides shared memory, so the saving is memory
bandwidth rather than network, but at 60 fps it is 200 MB/s against 13 MB/s.

`ex15_camera_formats` opens `front_left` and prices the alternatives against the frames it
is actually receiving. From `examples/rust/src/bin/ex15_camera_formats.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
const CAMERA: &str = "front_left"; // every vrobot ships front_left and front_right
const RESOLUTION: &str = "720p"; // 360p | 720p | 1080p -- robot-wide
const FORMAT: &str = "rgba8"; // mono8 | rgb8 | rgba8 -- per camera
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex15_camera_formats.cpp`:

```cpp
constexpr const char* CAMERA = "front_left";  // every vrobot ships front_left and front_right
constexpr const char* RESOLUTION = "720p";    // 360p | 720p | 1080p -- robot-wide
constexpr const char* FORMAT = "rgba8";       // mono8 | rgb8 | rgba8 -- per camera
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex15_camera_formats.py`:

```python
CAMERA = "front_left"  # every vrobot ships front_left and front_right
RESOLUTION = "720p"  # 360p | 720p | 1080p -- robot-wide
FORMAT = "rgba8"  # mono8 | rgb8 | rgba8 -- per camera
```

{{#endtab }}
{{#endtabs }}

The three strings are the stream identity on the wire, so they are literal strings in every
surface: there is no enum for either, and a typo in one of them is a stream that does not
exist rather than a compile error.

```text
camera stream: vrobots/1/i/cam/front_left/720p_rgba8

frame 1280x720 = 921600 px, rgba8 at 4 B/px, step=5120 B/row, 3686400 B/frame
  mono8:    921600 B/frame
   rgb8:   2764800 B/frame
  rgba8:   3686400 B/frame  <- this stream
```

**Getting the cheap end means creating a camera**, since the pair the robot ships is rgba8
and nothing reconfigures a camera you do not own. That is `mount_camera`, covered on
[Lens and mount pose](05-lens-and-pose.md) and used by `ex17_camera_pose` -- the one example
in the book that mounts.

## Resolution is robot-wide

**Format is per camera. Resolution is one knob for the whole robot**, shared by every
camera on it, and it is the only part of a mount request that is not confined to the camera
being named. Changing it restarts every stream on that robot under a new service name.

Three consequences, in the order people meet them:

1. Mounting a second camera at a different resolution from one **this handle** already
   holds is refused client-side with `VrError::InvalidArgument`, before anything reaches
   the wire. Mount them all at one resolution, or unmount the others first.
2. The guard can only see this handle's cameras. Mounting a camera at 360p does not unmount
   the robot's `front_left` and `front_right`, but it does move the resolution knob, so
   they come back at 360p under new names: `vrobots/1/i/cam/front_left/360p_rgba8`.
   Whatever was reading the 720p name is reading a service that no longer exists.
3. Unmounting does not put the knob back. `unmount_camera` removes one camera; resolution
   is a separate setting, and nothing in the SDK restores it. The robot streams at 360p
   until something sets it again or the simulator restarts.

The refusal names the cameras in the way:

```text
invalid_argument: camera "wide" asks for 720p but "mono" is already mounted at 360p -- resolution is one setting for the WHOLE robot, and changing it restarts every stream under a new name. Mount them all at the same resolution, or unmount the others first
```

> **Gotcha.** If a camera example times out on `front_left` at 720p, something mounted a
> camera at 360p and moved the robot with it. Open `front_left` at `"360p"` instead, or
> restart the simulator. `ex17_camera_pose` is the only example here that can cause it, and
> it mounts at 720p precisely so that it does not.

## What is checked before anything is sent

`CameraSpec::parse` validates all three strings at the call site, because every one of them
fails later as a stream that never arrives rather than as an error.

| Input | Rule | On failure |
|---|---|---|
| `name` | non-empty, and only letters, digits, `_` and `-` | `VrError::InvalidArgument`, naming the offending character |
| `resolution` | one of the accepted strings above | `VrError::InvalidArgument`, naming the accepted set |
| `format` | one of the accepted strings above | `VrError::InvalidArgument`, naming the accepted set |

The name has to survive being pasted into an iceoryx2 service name. The simulator does not
check it and does not error on a bad one: it mounts a camera whose service can never be
opened.

**Next:** [Inside a frame](03-frames.md)

**See also:** [Mount, open and unmount](01-mount-open-unmount.md), [Two cameras at once](06-two-cameras.md), [Appendix C: Error reference](../appendix-c-errors.md)
