# Freshness

fresh, latest and wait_new_frame, and which one a control loop wants.

```sh
cargo run -p vrobots-examples --bin ex13_open_camera
./target/cpp-build/ex13_open_camera
python examples/python/ex13_open_camera.py
```

## Three ways to read one stream

| Method | Returns | Blocks | Consumes freshness |
|---|---|---|---|
| `fresh()` | `Option<Arc<Frame>>`, `Some` only if a frame arrived since the last call | no | yes |
| `latest()` | `Option<Arc<Frame>>`, the current frame whether or not it is new | no | no |
| `wait_new_frame(timeout)` | `VrResult<()>`, `Ok` when a newer frame has landed | yes, up to `timeout` | no; call `fresh()` after it |

This is deliberately not how `states()` behaves. A control loop wants the current state
every iteration whether or not it changed, so `states()` always hands one back. An image
pipeline wants to do its work once per frame, so `fresh()` hands each frame out exactly
once. Polling `fresh()` at 100 Hz against a 60 fps stream returns a frame about 60 times a
second and `None` the rest of the time, which is the shape of the loop in
`ex03_hello_image`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
if let Some(frame) = cam.fresh() {
    // Some only if new since the last read
    seen += 1;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex03_hello_image.cpp`:

```cpp
// A value only if new since the last read.
if (auto frame = cam.fresh()) {
    ++seen;
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex03_hello_image.py`:

```python
if cam.fresh:
    frame = cam.frame  # metadata for the image we are about to read
    img = cam.image  # numpy (h, w, c) uint8, top-down, RGB(A)
    seen += 1
```

{{#endtab }}
{{#endtabs }}

Python is the one that splits the question from the answer. `cam.fresh` is a boolean
property that asks, and `cam.image` or `cam.read()` is what consumes the freshness; a loop
that tests `cam.fresh` and then never reads keeps seeing the same frame as new.

That fragment prints nothing on its own: it is the guard deciding whether the image half of
the loop runs at all, and `ex03_hello_image` prints a state line from the `else` arm instead.

Each frame is handed out once even when two threads race on the same stream: the claim is a
compare-and-exchange, so exactly one caller gets a given frame and the other gets `None`.
Reach for `latest()` when you want the current picture regardless, for a viewer or a status
line, and it will not steal a frame from the thread doing the real work.

## Pacing on frames instead of the clock

`wait_new_frame` blocks until a frame newer than the last stored one arrives, so the loop
body runs exactly once per rendered frame with no polling at all. From
`examples/rust/src/bin/ex13_open_camera.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
let mut seen = 0u64;
while seen < FRAMES {
    if let Err(VrError::Timeout(_)) = cam.wait_new_frame(Duration::from_millis(500)) {
        println!("no frame in 500 ms -- the camera stopped, or the sim is paused");
        continue;
    }
    let Some(frame) = cam.fresh() else { continue };
    seen += 1;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex13_open_camera.cpp`:

```cpp
std::uint64_t seen = 0;
while (seen < FRAMES) {
    try {
        cam.wait_new_frame(TIMEOUT_S);
    } catch (const vrsdk::Error& e) {
        if (e.code() != VRSDK_ERR_TIMEOUT) {
            throw;
        }
        std::printf("no frame in %.1fs -- the camera stopped, or the sim is paused\n",
                    TIMEOUT_S);
        continue;
    }
    const std::optional<vrsdk::Frame> frame = cam.fresh();
    if (!frame) {
        continue;
    }
    ++seen;
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex13_open_camera.py`:

```python
seen = 0
while seen < FRAMES:
    try:
        cam.wait_new_frame(TIMEOUT)
    except vrsdk.VrError as e:
        if e.code != vrsdk.err.TIMEOUT:
            raise
        print(f"no frame in {TIMEOUT}s -- the camera stopped, or the sim is paused")
        continue

    frame = cam.read()  # consumes freshness; None if someone else got it
    if frame is None:
        continue
    seen += 1
```

{{#endtab }}
{{#endtabs }}

`wait_new_frame` takes seconds as a `double` in C++ and Python where Rust takes a `Duration`,
and the timeout is the same status rather than a failure in all three. Note the second
guard: `wait_new_frame` returning does not guarantee the read succeeds, because another
thread may have taken the frame in between, so the empty case is still handled.

While frames are arriving, nothing is printed by the timeout arm:

```text
frame 1: seq=17 1280x720 3686400 bytes, mount=(+0.00,+0.00,+0.00) m
frame 11: seq=27 1280x720 3686400 bytes, mount=(+0.00,+0.00,+0.00) m
```

<!-- VERIFY: the seq values above are one run's values and have not been re-measured against a live simulator. -->

`wait_new_frame` returns `VrError::Timeout` when nothing new arrives in time, and the
message names the service and the deadline:

```text
timeout: no camera frame on vrobots/1/i/cam/front_left/720p_rgba8 within 500ms
```

> **Gotcha.** A timeout here never means the stream is broken, and it is also how a stopped
> camera presents: a paused simulator, a camera someone else unmounted, and a genuinely
> slow render are the same event. Treat it as a condition to handle, not an error to
> propagate, which is why the example loop continues rather than returning.

## The counters

`stats()` returns a `CameraStats` snapshot of the reader thread's counters.

| Field | Type | Notes |
|---|---|---|
| `received` | `u64` | frames received and published to the stream |
| `decode_errors` | `u64` | slices that did not decode; counted, never fatal |
| `seq_gaps` | `u64` | times the sequence number jumped forward by more than one |
| `missed_frames` | `u64` | total frames missed across all gaps |
| `last_seq` | `u64` | the last sequence number seen |

Because `seq` is contiguous by construction, a non-zero `seq_gaps` is a real shared-memory
drop rather than a skipped render. A malformed slice is counted and dropped, exactly like a
malformed state payload: one bad frame must not end a flight. When `decode_errors` is
non-zero, `last_error()` returns the most recent failure.

`received` counts what the thread stored, not what you read. A loop that polls more slowly
than the camera renders sees fewer frames than `received` reports, and that difference is
your loop falling behind, not a drop.

```text
read 60 frame(s): received=60 decode_errors=0 seq_gaps=0 missed_frames=0
```

## Stopping is not unmounting

`stop()` ends the reader thread, and `Drop` does the same and then joins it. Joining rather
than detaching is deliberate: the thread holds an iceoryx2 subscriber, and letting it
outlive the handle would leave a port attached to a service nobody can see. The wait is
bounded by one poll interval, about 2 ms, plus the frame in flight.

**Neither one unmounts the camera.** The robot keeps rendering and publishing for everyone
else, exactly as dropping a `VirtualRobot` leaves the robot running. `unmount_camera` is
the verb for that, and it works only on cameras this handle mounted.

`is_running()` reports whether the reader thread is still alive. It is `false` after
`stop()`, after `unmount_camera` removed this camera, or if the thread ended on its own.

**Next:** [Lens and mount pose](05-lens-and-pose.md)

**See also:** [Mount, open and unmount](01-mount-open-unmount.md), [Pacing your loop](../ch03-reading-state/07-pacing.md), [Stream health](../ch03-reading-state/08-health.md)
