# Inside a frame

Row order, stride, channel order and the metadata that rides with every image.

A `Frame` is an owned, immutable snapshot. The reader thread copies the pixels out of the
shared-memory sample and releases the sample immediately, so a `Frame` you hold stays valid
for as long as you keep it, however long that is.

## Every field

| Field | Type | Units | Notes |
|---|---|---|---|
| `t_ns` | `i64` | ns since the unix epoch | capture time, stamped when the simulator requested the GPU readback. The same clock as `State::t_ns`, so the two subtract directly |
| `elapsed` | `f64` | s | counts from the robot's first state sample, the same epoch as `State::elapsed` |
| `seq` | `u64` | | per stream, contiguous by construction: the simulator increments it only on a successful send, so a skipped render leaves no gap and any gap is a genuine shared-memory drop. Restarts at 0 when the stream restarts |
| `width` | `u32` | px | |
| `height` | `u32` | px | |
| `format` | `PixelFormat` | | `Mono8`, `Rgb8` or `Rgba8` |
| `step` | `u32` | bytes | bytes per row, always `width * bytes_per_pixel()`; wire padding has been removed |
| `data` | `FramePixels` | | `height * step` bytes, row-major, top-down, tightly packed; dereferences to `&[u8]` |
| `sys_id` | `u32` | | the robot this camera is on |
| `camera_name` | `String` | | the camera's name on the robot |
| `camera_id` | `u32` | | the camera's numeric id on the robot |
| `intrinsics` | `Intrinsics` | | the lens this frame was rendered through |
| `mount` | `MountPose` | | where the camera was when the frame was taken |
| `axis_convention` | `Axes` | | the convention the camera's own frame uses |
| `coord_frame_id` | `String` | | the camera's resolved coordinate frame id |
| `schema_version` | `u32` | | stamped by the simulator |

`intrinsics` and `mount` ride with **every frame**, which is the point: a gimballed or
re-mounted camera cannot desync from its images, and there is no camera-info topic to join
by timestamp. Page [Lens and mount pose](05-lens-and-pose.md) covers both.

| Method | Returns |
|---|---|
| `bytes_per_pixel()` | `u32`, 1, 3 or 4 |
| `row(y)` | `Option<&[u8]>`, one row top-down; `None` when `y >= height` |

## Three facts about the pixels

The SDK normalises geometry and nothing else.

**Rows are top-down.** Row 0 is the top of the picture. The wire is bottom-up, in Unity's
render order, and the SDK flips while copying, which costs nothing: it is the same memcpy
per row, in reverse order.

**Stride is tight.** `step == width * bytes_per_pixel()`, always, whatever padding the wire
carried. `data[y * step + x * bpp]` is the first byte of pixel `(x, y)` with no special
cases.

**Channels are never swapped.** `rgb8` is R, G, B and `rgba8` is R, G, B, A, exactly as the
renderer produced them. Converting for the consumers that want BGR would tax the ones that
do not, so the conversion happens at the call site that needs it. OpenCV users convert once,
in their own code: `cvtColor(..., COLOR_RGBA2BGR)` for the default `rgba8` streams and
`COLOR_RGB2BGR` for `rgb8`.

> **Gotcha.** Brightness is the wrong way to check orientation outdoors. Measured on the
> test scene, the sky rows run `B - R = +98` and the pale desert floor runs `-25`, so the
> ground is the brighter of the two and a brightness test reports the picture upside down.
> Compare blue against red instead.

## Reading a row

`row(n)` is the shortest way to sanity-check orientation and channel order at once. From
`examples/rust/src/bin/ex03_hello_image.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
/// Mean `blue - red` across one row: strongly positive for sky, negative for
/// most ground. `0.0` for mono8, which has no channels to compare.
fn blueness(frame: &Frame, row: u32) -> f64 {
    let bpp = frame.bytes_per_pixel() as usize;
    if bpp < 3 {
        return 0.0;
    }
    let Some(pixels) = frame.row(row) else {
        return 0.0;
    };
    let mut sum = 0.0;
    let mut count = 0.0;
    // Channel order is R,G,B(,A) -- the SDK normalises rows and stride, never
    // channel order, so this is the renderer's own layout.
    for pixel in pixels.chunks_exact(bpp) {
        sum += f64::from(pixel[2]) - f64::from(pixel[0]);
        count += 1.0;
    }
    if count == 0.0 { 0.0 } else { sum / count }
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex03_hello_image.cpp`:

```cpp
/// Mean `blue - red` across one row: strongly positive for sky, negative for
/// most ground. 0 for mono8, which has no channels to compare.
static double blueness(const vrsdk::Frame& frame, std::uint32_t row) {
    const std::uint32_t bpp = frame.bytes_per_pixel();
    const std::uint8_t* pixels = frame.row(row);
    if (bpp < 3 || pixels == nullptr) {
        return 0.0;
    }
    double sum = 0.0;
    // Channel order is R,G,B(,A) -- the SDK normalises rows and stride, never
    // channel order, so this is the renderer's own layout.
    for (std::uint32_t x = 0; x < frame.width(); ++x) {
        sum += static_cast<double>(pixels[x * bpp + 2]) - static_cast<double>(pixels[x * bpp]);
    }
    return frame.width() > 0 ? sum / frame.width() : 0.0;
}
```

{{#endtab }}
{{#tab name="Python" }}

The Python `ex03_hello_image.py` does not measure a row; it shows the frame in an OpenCV
window instead. The same measure is the `sky_ness` helper of the mount example,
`examples/python/ex17_camera_pose.py`:

```python
def sky_ness(img, row):
    # mean blue - red across one row: positive for sky, negative for ground
    if img.shape[2] < 3:
        return 0.0
    line = img[row].astype(np.int16)
    return float(np.mean(line[:, 2] - line[:, 0]))
```

{{#endtab }}
{{#endtabs }}

Rust and C++ walk the raw bytes: `frame.row(y)` hands back one row and `bytes_per_pixel`
gives the stride within it. Python does not walk bytes at all, because `cam.image` and
`frame.image` are numpy `(h, w, c)` uint8 arrays, so the same subtraction is one slice. All
three index channel 2 minus channel 0, which is blue minus red in the renderer's own RGB
order.

Called on row 0 and row `height - 1` of a forward-facing camera, it separates sky from
ground and therefore confirms both facts at once:

```text
Image front_left t=3.214 size=(1280x720) seq=42 lag_vs_state=8.4 ms
      sky-ness (B-R) top=+98 bottom=-25 (top-down: sky above ground), fov_y=61.9 deg
```

<!-- VERIFY: t, seq and lag_vs_state above are one run's values and have not been re-measured against a live simulator. -->

That is the Rust and C++ output. The Python `ex03_hello_image.py` prints only the `Image`
line, and `ex17_camera_pose.py` calls `sky_ness` once, on its last frame, to check a camera it
has rolled upside down.

`mono8` has one byte per pixel, so there is no channel order and no RGB against BGR
question at all: `data[y * step + x]` is the intensity. Getting one means mounting a camera
of your own, since the pair every vrobot ships is `rgba8`; `ex15_camera_formats` prices
that trade, and `ex17_camera_pose` is the example that mounts.

## A recorded slice is not a frame

`vrobots record --camera` writes what the reader thread receives, not what it hands you:
each file is one raw shared-memory slice, the 5760-byte prefix followed by the pixels, byte
for byte. The decoding that turns such a slice into a `Frame` is internal to the SDK. Its own
test suite runs it on recorded slices with the simulator closed, but no surface exposes it,
in Rust, C++ or Python. To keep frames for later, save the pixels you already hold, as
[Saving a frame](07-saving-frames.md) does.

**Next:** [Freshness](04-freshness.md)

**See also:** [Saving a frame](07-saving-frames.md), [Timestamps and sequence numbers](../ch03-reading-state/06-timestamps.md), [Recording and testing without the simulator](../ch08-tooling/07-recording-and-testing.md)
