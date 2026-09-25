# Saving a frame

Write one image to disk and stop, without pulling in an image library.

```sh
cargo run -p vrobots-examples --bin ex14_camera_save
./target/cpp-build/ex14_camera_save
python examples/python/ex14_camera_save.py
```

The shortest complete camera program there is: open, wait for one frame, write it, exit.
There is no loop, which makes it the right place to see what a `Frame` holds and what it
takes to get pixels out of the process. Like every camera example it reads `front_left`,
the camera every vrobot already ships at 720p rgba8, so there is no mount and no cleanup.

## One frame, blocking

`open_camera` has already waited for the stream to exist by the time it returns, but the
next frame still has to be rendered. Block for it rather than polling. From
`examples/rust/src/bin/ex14_camera_save.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
// open_camera already waited for the stream to exist, but the next frame
// still has to be rendered. Block for it rather than polling.
cam.wait_new_frame(TIMEOUT)?;
let frame = cam
    .fresh()
    .expect("wait_new_frame returned Ok, so one is waiting");
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex14_camera_save.cpp`:

```cpp
// ===== the one frame =====
// open_camera already waited for the stream to exist, but the next
// frame still has to be rendered. Block for it rather than polling.
cam.wait_new_frame(TIMEOUT_S);
const std::optional<vrsdk::Frame> frame = cam.fresh();
if (!frame) {
    std::fprintf(stderr, "wait_new_frame returned but no frame was waiting\n");
    return 1;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex14_camera_save.py`:

```python
# ===== the one frame =====
# open_camera already waited for the stream to exist, but the next frame
# still has to be rendered. Block for it rather than polling.
cam.wait_new_frame(TIMEOUT)
frame = cam.read()
assert frame is not None, "wait_new_frame returned, so one is waiting"
```

{{#endtab }}
{{#endtabs }}

Each surface asserts the same invariant in its own idiom. None of them has any unwinding to
do: nothing was created, so an early return leaves the simulator exactly as it found it.

`TIMEOUT` is 2 s here, and the `?` propagates a `VrError::Timeout` rather than swallowing
it: for a one-shot program, no frame is a failure and not a condition to retry through.

The two lines it prints are the geometry and the lens the frame was rendered through:

```text
frame seq=3 t=1.842s 1280x720 rgba8 (4 B/px, step=5120, 3686400 bytes)
intrinsics fx=600.0 fy=600.0 cx=640.0 cy=360.0 fov_y=61.9 deg  clip 0.50..1000 m
```

<!-- VERIFY: seq and t above are illustrative; fx and fy are the requested defaults and the live read-back can differ slightly, since the simulator round-trips them through the Unity field of view. -->

`step` is `width * 4` exactly for rgba8, and `data.len()` is `height * step` exactly.
Nothing on this path is padded or compressed.

## Writing it out

A binary PPM (P6) is a short text header followed by the raw RGB bytes, and that is the
entire format. It needs no image library, and every viewer reads it.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
/// Write the frame as a binary PPM (P6) -- the simplest image format there is.
/// Mono8 is expanded to grey RGB; rgba8 drops alpha.
fn write_ppm(frame: &Frame, path: &str) -> std::io::Result<()> {
    let mut out = std::io::BufWriter::new(std::fs::File::create(path)?);
    write!(out, "P6\n{} {}\n255\n", frame.width, frame.height)?;

    let bpp = frame.bytes_per_pixel() as usize;
    let mut rgb = Vec::with_capacity(frame.width as usize * frame.height as usize * 3);
    // Row-major and top-down already, so a straight walk is the right order.
    for pixel in frame.data.chunks_exact(bpp) {
        match bpp {
            1 => rgb.extend_from_slice(&[pixel[0], pixel[0], pixel[0]]),
            _ => rgb.extend_from_slice(&pixel[..3]),
        }
    }
    out.write_all(&rgb)?;
    out.flush()
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex14_camera_save.cpp`:

```cpp
/// Write the frame as a binary PPM (P6) -- the simplest image format there is.
/// Mono8 is expanded to grey RGB; rgba8 drops alpha.
static bool write_ppm(const vrsdk::Frame& frame, const std::string& path) {
    std::FILE* out = std::fopen(path.c_str(), "wb");
    if (out == nullptr) {
        return false;
    }
    std::fprintf(out, "P6\n%u %u\n255\n", frame.width(), frame.height());
    const std::uint32_t bpp = frame.bytes_per_pixel();
    std::vector<std::uint8_t> rgb;
    rgb.reserve(static_cast<std::size_t>(frame.width()) * frame.height() * 3);
    // Row-major and top-down already, so a straight walk is the right order.
    for (std::size_t i = 0; i + bpp <= frame.data.size(); i += bpp) {
        if (bpp == 1) {
            rgb.insert(rgb.end(), {frame.data[i], frame.data[i], frame.data[i]});
        } else {
            rgb.insert(rgb.end(), {frame.data[i], frame.data[i + 1], frame.data[i + 2]});
        }
    }
    const bool ok = std::fwrite(rgb.data(), 1, rgb.size(), out) == rgb.size();
    std::fclose(out);
    return ok;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex14_camera_save.py`:

```python
def save(img: np.ndarray, path: str) -> str:
    """Write the frame to disk, with or without OpenCV. Returns the path used."""
    if cv2 is not None:
        code = cv2.COLOR_RGBA2BGR if img.shape[2] == 4 else cv2.COLOR_RGB2BGR
        cv2.imwrite(path, img if img.shape[2] == 1 else cv2.cvtColor(img, code))
        return path
    # No OpenCV: a binary PPM needs no image library at all and every viewer
    # reads it. Mono8 is expanded to grey RGB; rgba8 drops alpha.
    path = path.rsplit(".", 1)[0] + ".ppm"
    rgb = img[:, :, :3] if img.shape[2] >= 3 else np.repeat(img, 3, axis=2)
    with open(path, "wb") as f:
        f.write(f"P6\n{rgb.shape[1]} {rgb.shape[0]}\n255\n".encode())
        f.write(rgb.tobytes())
    return path
```

{{#endtab }}
{{#endtabs }}

Python writes a PNG through OpenCV when it is installed and falls back to the same PPM when
it is not, which is why its `OUTPUT` is `frame.png` where the other two are `frame.ppm`. The
PPM path is identical in all three, and it needs no image library because the frame is
already row-major, top-down and tightly packed. What Python must do and the others must not
is the RGB to BGR conversion: OpenCV wants BGR and the SDK never swaps channels.

It prints nothing: the caller reports either the path it wrote or the `io::Error` it got
back. Three properties of `Frame` are doing the work here, and all three are why the loop is a
single straight walk over `data` with no row arithmetic:

- Rows are top-down, and PPM is top-down, so no flip is needed.
- Stride is tight, so `chunks_exact(bpp)` never walks into padding.
- Channels are R, G, B, A in that order, and PPM wants the first three of them. A format
  that wanted BGR would convert here, at the call site, and nowhere else.

`mono8` is expanded to grey by repeating the single byte three times, and `rgba8` drops
alpha by taking `pixel[..3]`.

The run ends as soon as the file is on disk, with nothing to put back:

```text
attached to vrobots/1/i/cam/front_left/720p_rgba8
wrote frame.ppm
```

Ctrl-C during the two-second wait is equally safe here, which it would not be in a program
that had mounted a camera of its own.

## When one file is not enough

Writing images from a loop makes this the slow part of your program: a 720p rgba8 frame is
3.69 MB, and 60 of those per second is more than most disks want. For capture rather than
inspection, record the raw slices instead and decode them later with `Frame::decode`, which
is what `vrobots record --camera` and the fixture workflow are for. That path is covered in
[Recording and testing without the simulator](../ch08-tooling/07-recording-and-testing.md),
along with why those recordings are the thing that keeps `cargo test` meaningful with the
simulator closed.

**Next:** [Showing frames in a window](08-showing-frames.md)

**See also:** [Inside a frame](03-frames.md), [Freshness](04-freshness.md), [The vrobots command](../ch08-tooling/01-cli.md)
