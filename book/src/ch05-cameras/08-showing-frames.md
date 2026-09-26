# Showing frames in a window

Put the live camera on screen with OpenCV, and convert the one property the SDK deliberately leaves alone.

```sh
cargo run -p vrobots-examples --features opencv --bin ex34_camera_view
./target/cpp-build/ex34_camera_view
python examples/python/ex34_camera_view.py
```

## The OpenCV dependency

In Rust and C++, every other example in this book needs nothing but the SDK. This one needs
OpenCV, so all three languages keep it opt-in rather than making everyone install it. In
Python, `ex03_hello_image.py` and `ex14_camera_save.py` import `cv2` as well, so one
`opencv-python` install serves all three scripts:

| Language | How it is opted into | Without OpenCV installed |
|---|---|---|
| Rust | the `opencv` cargo feature, off by default | a build without the feature skips the binary |
| C++ | `find_package(OpenCV)` in `examples/cpp/CMakeLists.txt` | CMake prints `skipping ex34_camera_view` and builds the rest |
| Python | `pip install opencv-python` | `import cv2` raises `ModuleNotFoundError` and the script stops there |

That is why the Rust command above carries `--features opencv` and no other command in this
book does. [Hello image](../ch01-getting-started/06-hello-image.md) is the version with no
dependency in Rust and C++; its Python script shows the frames in an OpenCV window too.

## The loop

Setup is the same `open_camera` on `front_left` as
[Hello image](../ch01-getting-started/06-hello-image.md), followed by one `named_window`.
Nothing is mounted, so nothing has to be torn down. The loop is where the two lessons of this page live. From
`examples/rust/src/bin/ex34_camera_view.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    // ===== loop =====
    // Frame-paced: wait_new_frame blocks until the next render, so imshow runs
    // once per frame rather than redrawing one it has already shown.
    let mut seen = 0u64;
    loop {
        if let Err(VrError::Timeout(_)) = cam.wait_new_frame(TIMEOUT) {
            // A status, not a failure: the sim is paused, or the camera stopped.
            // Still pump the GUI so the window stays responsive.
            if quit_requested()? {
                break;
            }
            continue;
        }
        let Some(frame) = cam.fresh() else { continue };
        seen += 1;

        // The pixels are already row-major, top-down and tightly packed, so this
        // is a straight copy into a Mat of the same shape.
        let mut rgba = Mat::new_rows_cols_with_default(
            frame.height as i32,
            frame.width as i32,
            CV_8UC4,
            Scalar::all(0.0),
        )?;
        rgba.data_bytes_mut()?.copy_from_slice(&frame.data);

        // The SDK never does this for you: RGBA is what Unity rendered, BGR is
        // what OpenCV displays.
        let mut bgr = Mat::default();
        imgproc::cvt_color_def(&rgba, &mut bgr, imgproc::COLOR_RGBA2BGR)?;

        highgui::imshow(WINDOW, &bgr)?;
        if quit_requested()? {
            break;
        }
    }
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex34_camera_view.cpp`:

```cpp
        // ===== loop =====
        // Frame-paced: wait_new_frame blocks until the next render, so imshow
        // runs once per frame rather than redrawing one it has already shown.
        std::uint64_t seen = 0;
        for (;;) {
            try {
                cam.wait_new_frame(TIMEOUT_S);
            } catch (const vrsdk::Error& e) {
                if (e.code() != VRSDK_ERR_TIMEOUT) {
                    throw;
                }
                // A status, not a failure: the sim is paused, or the camera
                // stopped. Still pump the GUI so the window stays responsive.
                if (quit_requested()) {
                    break;
                }
                continue;
            }

            const std::optional<vrsdk::Frame> frame = cam.fresh();
            if (!frame) {
                continue;
            }
            ++seen;

            // A header over the frame's own bytes -- no copy. Row-major,
            // top-down and tightly packed is exactly what cv::Mat wants.
            const cv::Mat rgba(static_cast<int>(frame->height()), static_cast<int>(frame->width()),
                               CV_8UC4, const_cast<std::uint8_t*>(frame->data.data()),
                               static_cast<std::size_t>(frame->step()));

            // The SDK never does this for you: RGBA is what Unity rendered, BGR
            // is what OpenCV displays.
            cv::Mat bgr;
            cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);

            cv::imshow(WINDOW, bgr);
            if (quit_requested()) {
                break;
            }
        }
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex34_camera_view.py`:

```python
while True:
    frame = cam.read()
    if frame is not None:
        cv2.imshow("vrobots camera", cv2.cvtColor(frame.image, cv2.COLOR_RGBA2BGR))
    if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
        break
```

The Python script is shorter. It polls `cam.read()` instead of blocking on
`wait_new_frame`, so it has no timeout to handle, and it creates no window up front:
`cv2.imshow` opens one on its first call.

{{#endtab }}
{{#endtabs }}

Two differences worth naming. Rust matches on `VrError::Timeout` where C++ compares
`e.code()` against `VRSDK_ERR_TIMEOUT`, which is the same distinction the whole book draws
between a timeout and a failure; the Python script never waits, so it never meets the
timeout. And the three reach the pixels differently: C++ wraps a `cv::Mat` header around the
frame's own bytes and copies nothing, Rust copies into an owned `Mat`, and Python passes the
numpy array from `frame.image` as it stands. The C++ header is valid only while that `Frame`
is alive, which is why it is built inside the loop body and never stored.

## RGBA in, BGR out

The SDK normalises geometry and nothing else. Rows arrive top-down, `step` is
`width * bytes_per_pixel` with no padding, and channels are the renderer's own order, so
the pixels map onto a `Mat` with no rearranging and then need exactly one colour
conversion, written at the call site that wants BGR.

Skip that conversion and the picture still appears, with the sky orange and the desert
blue. That is the fastest way to recognise the mistake.

The example opens `front_left`, which is `rgba8`, so the type is fixed. The other two
formats differ only here:

| Format | Mat type | Conversion for display |
|---|---|---|
| `rgba8` | `CV_8UC4` | `COLOR_RGBA2BGR`, what this example uses |
| `rgb8` | `CV_8UC3` | `COLOR_RGB2BGR` |
| `mono8` | `CV_8UC1` | none, one channel has no order |

## One imshow per rendered frame

`wait_new_frame` blocks until the next render, so the window redraws exactly once per frame
instead of re-showing a picture it has already drawn. A timeout is a status rather than an
error, exactly as in [Freshness](04-freshness.md), and the one thing that must still happen
on that path is the GUI pump: a window that never gets `wait_key` stops repainting and the
desktop reports the program as not responding.

The Python script reaches one `imshow` per frame by polling instead: `cam.read()` returns
`None` until a new frame has arrived, and `cv2.waitKey(1)` runs on every pass, so the window
is serviced whether or not a frame was fresh.

That pump is also how the keypress is read, which is why `quit_requested` in the Rust and C++
files does both in one call, and why the Python script tests the key that `cv2.waitKey(1)`
returns. `imshow` on its own queues an image and paints nothing, and the 1 ms argument is a
maximum rather than a delay: the call returns as soon as the window has been serviced.

## Quitting

The run prints one line on the way in and one on the way out:

```text
showing vrobots/1/i/cam/front_left/720p_rgba8 -- press q or Esc to quit
showed 412 frame(s), received=412 decode_errors=0 seq_gaps=0
```

<!-- VERIFY: the frame counts above are illustrative; they depend on how long the window is left open. -->

That is the Rust and C++ output. The Python script prints nothing: it opens the window, and
pressing `q` or Esc closes it.

Pressing `q` or Esc breaks the loop, and the only thing left to close is the window:

```rust
    // ===== cleanup =====
    // Only ours: the window. Dropping the stream ends this subscription and
    // nothing else -- front_left keeps rendering for everyone.
    highgui::destroy_all_windows()?;
```

The Python script does the same with `cv2.destroyAllWindows()` after its loop.

Ctrl-C is equally safe: this example never mounted anything, so there is no camera left
behind on a robot that outlives the process. An example that mounts one -- `ex17_camera_pose`
is the only one here -- does have that deadline, and page
[Mount, open and unmount](01-mount-open-unmount.md) spells it out.

`showed` counts what reached the window and `received` counts what the reader thread got,
so a gap between them means frames arrived while the loop was inside `cvtColor` or
`imshow`. `seq_gaps` is the different and more serious number: it counts frames the
publisher sent that never arrived at all.

**Next:** [Services and configuration](../ch06-services/00-intro.md)

**See also:** [Inside a frame](03-frames.md), [Freshness](04-freshness.md), [Saving a frame](07-saving-frames.md)
